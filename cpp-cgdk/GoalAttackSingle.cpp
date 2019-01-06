#include "GoalAttackSingle.h"
#include "goalUtils.h"
#include "State.h"
#include "noReleaseAssert.h"
#include "physicsUtils.h"

using namespace goals;
using namespace model;


AttackSingle::AttackSingle(State& state, GoalManager& goalManager)
    : Goal(state, goalManager)
{
    pushBackStep(Never{},                           // abort if
                 Always{},                          // proceed if
                 [this]() {return repeat(); },
                 "repeat");
}


AttackSingle::~AttackSingle()
{
}

Goal::StepStatus AttackSingle::repeat()
{
    pushBackStep(Never{},                                 // abort if
                 Always{},                                // proceed if
                 [this]() {return assignAttackerId(); },
                 "assignAttackerId");

    pushBackStep(Never{},                                              // abort if
                 [this]() {return isAttackPhase() && canMove(); },     // proceed if
                 [this]() {return findAttackPos(); },
                 "findAttackPos");

    pushBackStep(Never{},                                              // abort if
                 [this]() {return isAttackPhase() && canMove(); },     // proceed if
                 [this]() {return reachAttackPos(); },
                 "reachAttackPos");

    pushBackStep(Never{},                                 // abort if
                 Always{},                                // proceed if
                 [this]() {return repeat(); },
                 "repeat");

    return StepStatus::Done;
}

bool AttackSingle::isAttacker() const
{
    return state().me().id == m_attackerId;
}

bool AttackSingle::isAttackPhase() const
{
    return isAttacker() && state().me().z <= state().game().ball.z;    // ball is between attacker and enemy gates
}

bool AttackSingle::canMove() const
{
    return state().me().touch  // on ground
        || state().me().nitro_amount > 0;
}

Goal::StepStatus AttackSingle::assignAttackerId()
{
    const Robot* farTeammate = nullptr;

    for(const Robot& r : state().game().robots)
    {
        if(!r.is_teammate)
            continue;

        // choose robot which is most far from our gate
        if(farTeammate == nullptr || farTeammate->z < r.z)
            farTeammate = &r;
    }

    m_attackerId = (farTeammate != nullptr) ? farTeammate->id : NONE;
    return m_attackerId != NONE ? StepStatus::Done : StepStatus::Ok;
}

Goal::StepStatus AttackSingle::findAttackPos()
{
    assert(isAttackPhase());
    m_attackPos = {};

    const auto& predictions = state().ballPredictions();
    const model::Game& game = state().game();
    const model::Rules& rules = state().rules();

    for(const State::PredictedPos& prediction : predictions)
    {
        const double thresholdY = rules.BALL_RADIUS + rules.ROBOT_MIN_RADIUS / 2;    // #todo - jump support
        if(prediction.m_tick < game.current_tick || prediction.m_pos.y > thresholdY)
            continue;

        int meetingTick = game.current_tick + ticksToReach(prediction.m_pos);
        if(meetingTick > prediction.m_tick)
            continue;

        m_attackPos = prediction.m_pos;
    }

    return m_attackPos != Vec3d{} ? StepStatus::Done : StepStatus::Ok;
}

int AttackSingle::ticksToReach(const Vec3d& target) const
{
    Entity<model::Robot> me = state().me();
    const model::Rules&  rules = state().rules();

    Vec2d meXZ     = { me.position().x, me.position().z };
    Vec2d targetXZ = { target.x, target.z };
    Vec2d displacementXZ = targetXZ - meXZ;
    Vec2d directionXZ    = linalg::normalize(displacementXZ);

    double shorten = linalg::length(displacementXZ) / (linalg::length(displacementXZ) - rules.BALL_RADIUS - rules.ROBOT_MIN_RADIUS);
    if(shorten > 1)
        displacementXZ /= shorten;

    Vec2d meVelocityXZ = Vec2d{ me.velocity().x, me.velocity().z };
    double approachSpeed = linalg::dot(meVelocityXZ, directionXZ);
    double maxApproachSpeed = rules.ROBOT_MAX_GROUND_SPEED;

    double distanceXZ = linalg::length(displacementXZ);
    double approachTimeSecs = uniform_accel::distanceToTime(distanceXZ, rules.ROBOT_ACCELERATION, approachSpeed, maxApproachSpeed); // #todo st.2 - nitro
    double approachTimeTics = approachTimeSecs * rules.TICKS_PER_SECOND;

    // #todo jump estimation support 

    return static_cast<int>(std::ceil(approachTimeTics));
}

Goal::StepStatus AttackSingle::reachAttackPos()
{
    const auto& predictions = state().ballPredictions();
    const model::Game& game = state().game();
    const model::Rules& rules = state().rules();

    Entity<model::Robot> me = state().me();
    Entity<model::Ball>  ball = state().game().ball;


    auto itFound = std::find_if(predictions.begin(), predictions.end(), [this](const State::PredictedPos& prediction) {
        return prediction.m_pos == m_attackPos;
    });

    if(predictions.end() == itFound)
        return Goal::StepStatus::Done;     // abort this step, next step is entire goal repeat

    Vec2d meXZ = { me.position().x, me.position().z };
    Vec2d targetXZ = { m_attackPos.x, m_attackPos.z };
    Vec2d displacementXZ = targetXZ - meXZ;
    Vec2d directionXZ = linalg::normalize(displacementXZ);

    double shorten = linalg::length(displacementXZ) / (linalg::length(displacementXZ) - rules.BALL_RADIUS - rules.ROBOT_MIN_RADIUS);
    if(shorten > 1)
        displacementXZ /= shorten;

    double distance = linalg::length(displacementXZ);
    double needSpeedSI = distance / static_cast<double>(ticksToReach(m_attackPos)) * rules.TICKS_PER_SECOND;  // actually, not so SI: length units per second
    Vec2d targetSpeedXZ = directionXZ * needSpeedSI;

    Action action;
    action.target_velocity_x = targetSpeedXZ[0];
    action.target_velocity_z = targetSpeedXZ[1];

    if(linalg::length2(ball.position() - me.position()) < pow2(state().rules().ROBOT_MAX_RADIUS + state().rules().BALL_RADIUS))
        action.jump_speed = state().rules().ROBOT_MAX_JUMP_SPEED;

    state().commitAction(action);
    return action.jump_speed == 0 ? StepStatus::Ok : StepStatus::Done;
}

