#include "Goalkeeper.h"
#include "goalUtils.h"
#include "State.h"
#include "noReleaseAssert.h"
#include "physicsUtils.h"
#include "DebugRender.h"
#include "stringUtils.h"
#include "Simulator.h"

using namespace goals;
using namespace model;

Goalkeeper::Goalkeeper(State& s, GoalManager& goalManager)
    : Goal(s, goalManager)
{
    pushBackStep([this]() {return isGoalDone(); },  // abort if
                 Always{},                          // proceed if
                 [this]() {return repeat(); },
                 "repeat");

}

Goal::StepStatus Goalkeeper::repeat()
{
    pushBackStep([this]() {return isGoalDone(); },        // abort if
                 Always{},                                // proceed if
                 [this]() {return assignGoalkeeperId(); },
                 "assignGoalkeeperId");

    pushBackStep([this]() {return isGoalDone(); },                     // abort if
                 [this]() {return isDefendPhase() && canMove(); },     // proceed if
                 [this]() {return findDefendPos(); },
                 "findDefendPos");

    pushBackStep([this]() {return isGoalDone(); },                     // abort if
                 [this]() {return isDefendPhase() && canMove(); },     // proceed if
                 [this]() {return reachDefendPos(); },
                 "reachDefendPos");

    pushBackStep([this]() {return isGoalDone(); },        // abort if
                 Always{},                                // proceed if
                 [this]() {return repeat(); },
                 "repeat");

    return StepStatus::Done;

}



Goalkeeper::~Goalkeeper()
{
}

bool Goalkeeper::isGoalkeeper() const
{
    return state().me().id == m_keeperId;
}

bool Goalkeeper::canMove() const
{
    return canMoveImpl(state().me());
}

bool Goalkeeper::isDefendPhase() const
{
    const model::Robot& me  = state().me();
    const model::Ball& ball = state().game().ball;

    return isGoalkeeper() && ball.z < 0 && ball.velocity_z < 0;
}

bool Goalkeeper::isGoalDone() const
{
    return state().isChilloutPhase();
}

Goal::StepStatus Goalkeeper::assignGoalkeeperId()
{
    const Robot* nearTeammate = nullptr;

    for(const Robot& r : state().game().robots)
    {
        if(!r.is_teammate)
            continue;

        // choose robot which is the nearest to our gate
        if(nearTeammate == nullptr || nearTeammate->z > r.z)
            nearTeammate = &r;
    }

    m_keeperId= (nearTeammate != nullptr) ? nearTeammate->id : NONE;
    return m_keeperId != NONE ? StepStatus::Done : StepStatus::Ok;
}

Goal::StepStatus Goalkeeper::findDefendPos()
{
    assert(isDefendPhase());
    m_defendPos = {};

    const auto& predictions = state().ballPredictions();
    const model::Game& game = state().game();
    const model::Rules& rules = state().rules();
    Entity<Ball> ball = game.ball;
    Entity<Robot> me = state().me();

    constexpr static const int NO_MEETING = std::numeric_limits<int>::max();
    int meetingTick = NO_MEETING;

    const double thresholdY = 4.7;    // #todo - wrote in rush, full jump support
    const double thresholdZ_min = -rules.arena.depth / 2 - rules.BALL_RADIUS * 1.9;   // #todo - implement full support for case when ball intersects goal point with unreachanble height (seed = 1, 1st goal with no other 'goals')
    const double thresholdZ_max = -rules.arena.depth / 2 + rules.BALL_RADIUS * 2;

    for(const State::PredictedPos& prediction : predictions)
    {
        if(prediction.m_tick < game.current_tick || prediction.m_pos.y > thresholdY 
            || prediction.m_pos.z < thresholdZ_min || prediction.m_pos.z > thresholdZ_max)
            continue;

        meetingTick = game.current_tick + ticksToReach(prediction.m_pos, state());
        if(meetingTick > prediction.m_tick)
            continue;

        m_defendPos = prediction.m_pos;              // #todo after goalkeeper: don't use exact ball pos!
        break;
    }

    DebugRender::instance().shpere(m_defendPos, rules.BALL_RADIUS + .1, { 1, 0, 0.5, .25 });
    DebugRender::instance().text(R"(defend by #%id pos: %pos, tick: %tick)"_fs
        .format("%id", state().me().id)
        .format("%pos", m_defendPos)
        .format("%tick", meetingTick)
        .move());

    // in case if just stopped ball and it doesn't go into gates anymore
    if(linalg::length2(ball.position() - me.position()) < pow2(state().rules().ROBOT_MAX_RADIUS + state().rules().BALL_RADIUS))
    {
        model::Action action;
        action.jump_speed = state().rules().ROBOT_MAX_JUMP_SPEED;
        state().commitAction(action);
        return StepStatus::Ok;
    }

    return (m_defendPos != Vec3d{} && meetingTick != NO_MEETING) ? StepStatus::Done : StepStatus::Ok;
}

Goal::StepStatus Goalkeeper::reachDefendPos()
{
    const auto& predictions = state().ballPredictions();
    const model::Game& game = state().game();
    const model::Rules& rules = state().rules();

    Entity<model::Robot> me   = state().me();
    Entity<model::Ball>  ball = game.ball;

    auto itFound = std::find_if(predictions.begin(), predictions.end(), [this](const State::PredictedPos& prediction) {
        return prediction.m_pos == m_defendPos;
    });

    if(predictions.end() == itFound)
        return Goal::StepStatus::Done;     // abort this step, next step is entire goal repeat
    int ticksToArrive = itFound->m_tick - game.current_tick;

    Vec2d meXZ     = { me.position().x, me.position().z };
    Vec2d targetXZ = { m_defendPos.x, m_defendPos.z };
    Vec2d displacementXZ = targetXZ - meXZ;
    Vec2d directionXZ    = linalg::normalize(displacementXZ);

    double shorten = linalg::length(displacementXZ) / (linalg::length(displacementXZ) - rules.BALL_RADIUS - rules.ROBOT_MIN_RADIUS);
    if(shorten > 1)
        displacementXZ /= shorten;

    double distance = linalg::length(displacementXZ);
    double needSpeedSI = distance / static_cast<double>(ticksToReach(m_defendPos, state())) * rules.TICKS_PER_SECOND;  // actually, not so SI: length units per second
    Vec2d targetSpeedXZ = directionXZ * needSpeedSI;

    Action action;
    action.target_velocity_x = targetSpeedXZ[0];
    action.target_velocity_z = targetSpeedXZ[1];

    if(linalg::length2(ball.position() - me.position()) < pow2(state().rules().ROBOT_MAX_RADIUS + state().rules().BALL_RADIUS))
        action.jump_speed = state().rules().ROBOT_MAX_JUMP_SPEED;

    auto jumpInfo = jumpPrediction(m_defendPos.y, state());
    if(std::abs(jumpInfo->m_timeToReach - ticksToArrive) < k_Epsilon)
        action.jump_speed = state().rules().ROBOT_MAX_JUMP_SPEED;

    state().commitAction(action);
    return action.jump_speed == 0 ? StepStatus::Ok : StepStatus::Done;
}

