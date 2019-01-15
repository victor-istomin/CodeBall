#include "Goalkeeper.h"
#include "GoalAttackSingle.h"
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

bool Goalkeeper::isDefendPhase()
{
    const model::Robot& me  = state().me();
    const model::Ball& ball = state().game().ball;

    bool isDefend = isGoalkeeper() && 
        (   (ball.z < 0 && ball.velocity_z < 0)
          || state().goalPredictionTick().m_mineGates != State::INT_NONE);

    constexpr int CANCELING_LATENCY = 30;
    if (isDefend)
        m_lastDefendTick = state().game().current_tick;

    return isDefend || (ticksFromLastDefence() < CANCELING_LATENCY);
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
    m_defendMap.clear();

    const auto& predictions = state().ballPredictions();
    const model::Game& game = state().game();
    const model::Rules& rules = state().rules();
    const Entity<Ball> ball = game.ball;
    const Entity<Robot> me = state().me();

    const double thresholdY     = 4.7 + rules.BALL_RADIUS;    // #todo - wrote in rush, full jump support
    const double thresholdZ_min = -rules.arena.depth / 2 - rules.BALL_RADIUS * 1.9;   // #todo - implement full support for case when ball intersects goal point with unreachanble height (seed = 1, 1st goal with no other 'goals')
    const double thresholdZ_max = -rules.arena.depth / 2 + rules.BALL_RADIUS * 2;
    const double thresholdX     =  rules.arena.goal_width / 2 + rules.arena.goal_side_radius;

    auto teammates = state().teammates();

    // #todo #bug on seed = 2299944762, tag=v3:  WTF - 4th goal

    // calculate goalkeeper's defend pos first
    auto itKeeper = std::find_if(teammates.begin(), teammates.end(), [this](const model::Robot& r) { return r.id == m_keeperId; });
    if(itKeeper != teammates.end())
    {
        std::swap(*itKeeper, teammates.front());
        itKeeper = teammates.begin();
    }

    for(const Entity<Robot>& robot : teammates)
    {
        DefenceInfo defence = {};

        for(const State::PredictedPos& prediction : predictions)
        {
            const Vec3d& predictionPos = prediction.m_pos;

            if(prediction.m_tick < game.current_tick || predictionPos.y > thresholdY)
                continue;

            if(robot.id == m_keeperId)     // goalkeeper should remain near gates
            {
                if(predictionPos.z < thresholdZ_min || predictionPos.z > thresholdZ_max
                    || predictionPos.x < -thresholdX || predictionPos.x > thresholdX)
                {
                    continue;
                }
            }
            else if(itKeeper != teammates.end())
            {
                // should not interfere with goalkeeper
                Vec3d diffGK  = predictionPos - itKeeper->position();
                Vec3d diffGKD = getDefendPos(itKeeper->id);
                Rational diffThresholdSq = pow2(2 * rules.ROBOT_MAX_RADIUS + 2 * rules.BALL_RADIUS);

                Rational minDistanceSq = std::min(linalg::length2(diffGK), linalg::length2(diffGKD));
                if(predictionPos.z <= (-rules.arena.depth / 2) || minDistanceSq < diffThresholdSq)
                    continue;
            }

            int roughtEta = ticksToReach(robot, predictionPos, state().rules());
            int meetingTick = game.current_tick + roughtEta;
            if(meetingTick > prediction.m_tick)
                continue;

            defence.ballPos = predictionPos;              // #todo after goalkeeper: don't use exact ball pos!
            defence.meetingTick = meetingTick;
            defence.ticksToReach = roughtEta;
            break;
        }

        if(defence.meetingTick != NO_MEETING)
            m_defendMap[robot.id] = defence;

        DebugRender::instance().shpere(defence.ballPos, rules.BALL_RADIUS + .1, { 1, 0, 0.5, .25 });
        DebugRender::instance().text(R"(defend: goalkeeper #%gc, defence by #%id pos: %pos, tick: %tick)"_fs
                               .format("%gc",   m_keeperId)
                               .format("%id",   robot.id)
                               .format("%pos",  defence.ballPos)
                               .format("%tick", defence.meetingTick)
                               .move());
    }

    // alternate defence pos
    // #todo - temporary hack, because there is no such pos support in this->reachDefendPos();
    Vec3d myPos = { me.x, me.y, me.z };
    DefenceInfo& myDefence = m_defendMap[state().me().id];
    if(myDefence.meetingTick == NO_MEETING)
    {
        myDefence.ballPos = { 0.0, 0.0, -(rules.arena.depth / 2.0) + rules.arena.bottom_radius };
        Vec3d diff = myDefence.ballPos - myPos;

        Action action;
        action.target_velocity_x = diff.x;
        action.target_velocity_z = diff.z;

        state().commitAction(action);
    }

    // in case if just stopped ball and it doesn't go into gates anymore
    if(linalg::length2(ball.position() - myPos) < pow2(state().rules().ROBOT_MAX_RADIUS + state().rules().BALL_RADIUS))
    {
        model::Action action;
        action.jump_speed = state().rules().ROBOT_MAX_JUMP_SPEED;
        state().commitAction(action);
        return StepStatus::Ok;
    }

    return StepStatus::Done;
}

Goal::StepStatus Goalkeeper::reachDefendPos()
{
    const auto& predictions = state().ballPredictions();
    const model::Game& game = state().game();
    const model::Rules& rules = state().rules();

    Entity<model::Robot> me   = state().me();
    Entity<model::Ball>  ball = game.ball;

    const Vec3d defendPos = getDefendPos(me.id);
    auto itFound = std::find_if(predictions.begin(), predictions.end(), [&defendPos](const State::PredictedPos& prediction) {
        return prediction.m_pos == defendPos;
    });

    if(predictions.end() == itFound)
        return Goal::StepStatus::Done;     // abort this step, next step is entire goal repeat
    int ticksToBallArrive = itFound->m_tick - game.current_tick;

    // #todo - choose better desiredPos in order to bounce / intercept ball in better direction
    // #todo - modify attacker in order to don't collide with goalkeeper
    Vec3d desiredPos = defendPos;
    desiredPos.y -= rules.BALL_RADIUS;   // try hit by robot center, so not decrement by ROBOT_RADIUS

    Vec3d hitDisplacement = {};
    if((ball.y - rules.BALL_RADIUS) > (me.y + rules.ROBOT_MAX_RADIUS * 1.5)   // actually, ball is over me
        || ball.z >= me.z)                                                    //  ... or at least 'me' is closer to gates than actual ball 
    {
        hitDisplacement = { 0.0, 0.0, rules.ROBOT_RADIUS * 0.5 };
    }

    desiredPos -= hitDisplacement;

    Vec2d meXZ     = { me.position().x, me.position().z };
    Vec2d targetXZ = { desiredPos.x, desiredPos.z };
    Vec2d displacementXZ = targetXZ - meXZ;
    Vec2d directionXZ    = linalg::normalize(displacementXZ);

    double distance = linalg::length(displacementXZ);
    double ticksToArrive = static_cast<double>(ticksToReach(me, desiredPos, state().rules()));
    double needSpeedSI = distance / ticksToArrive * rules.TICKS_PER_SECOND;  // actually, not so SI: length units per second

    if(ticksToArrive > rules.TICKS_PER_SECOND / 4)
        needSpeedSI = rules.ROBOT_MAX_GROUND_SPEED;

    Vec2d targetSpeedXZ = directionXZ * needSpeedSI;

    Action action;
    action.target_velocity_x = targetSpeedXZ[0];
    action.target_velocity_z = targetSpeedXZ[1];

    double ballDistanceSq = linalg::length2(ball.position() - me.position());
    const double hitDistanceSq = pow2(state().rules().ROBOT_MAX_RADIUS + state().rules().BALL_RADIUS);
    const double nitroDistanceSq = pow2(state().rules().ROBOT_MAX_RADIUS * 1.3 + state().rules().BALL_RADIUS);    // #todo st. 2 something better

    Vec3d hitVelocity = linalg::normalize((ball.position() - hitDisplacement) - me.position()) * rules.MAX_ENTITY_SPEED;
    if(ballDistanceSq < hitDistanceSq)
    {
        action.jump_speed = state().rules().ROBOT_MAX_JUMP_SPEED;
        action.target_velocity_x = hitVelocity.x;
        action.target_velocity_y = 0;
        action.target_velocity_z = hitVelocity.z;
    }
    if(ball.z > me.z && ballDistanceSq < nitroDistanceSq)
    {
        action.target_velocity_x = hitVelocity.x;
        action.target_velocity_y = hitVelocity.y;
        action.target_velocity_z = hitVelocity.z;
        action.use_nitro = true;
    }


    auto scoring = [desiredPos, &rules](const PredictedJumpHeight& prediction)
    {
        static constexpr const double JUMP_OVER_PENALTY = 2;
        static constexpr const double JUMP_UNDER_PENALTY = 1.5;

        double diff = prediction.m_height - desiredPos.y;
        double weightedDiff = std::abs(diff * (diff > 0 ? JUMP_OVER_PENALTY : JUMP_UNDER_PENALTY));
        weightedDiff += prediction.m_timeToReach * (rules.ROBOT_MIN_RADIUS / 5); // 1 tick "costs" 1/5 * robot_radius error

        return 100 / weightedDiff;
    };

    // #bug should take into account current speed, not time only
    auto jumpInfo = jumpPrediction(state(), scoring);
    if(std::abs(jumpInfo->m_timeToReach - ticksToBallArrive) < k_Epsilon)
        action.jump_speed = jumpInfo->m_initialSpeed;

    state().commitAction(action);
    return action.jump_speed == 0 ? StepStatus::Ok : StepStatus::Done;
}

bool Goalkeeper::isCompatibleWith(const Goal* interrupted)
{
    return nullptr != dynamic_cast<const AttackSingle*>(interrupted);    // can do these goals in parallel
}

int Goalkeeper::ticksFromLastDefence() const
{
    return m_lastDefendTick != NONE ? state().game().current_tick - m_lastDefendTick : std::numeric_limits<int>::max();
}

Vec3d Goalkeeper::getDefendPos(Id id) const
{
    auto itFound = m_defendMap.find(id);
    return itFound != m_defendMap.end() ? itFound->second.ballPos : Vec3d{};
}

