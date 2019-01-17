#include "GoalAttackSingle.h"
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


AttackSingle::AttackSingle(State& state, GoalManager& goalManager)
    : Goal(state, goalManager)
{
    pushBackStep([this]() {return isGoalDone(); },  // abort if
                 Always{},                          // proceed if
                 [this]() {return repeat(); },
                 "repeat");
}


AttackSingle::~AttackSingle()
{
}

Goal::StepStatus AttackSingle::repeat()
{
    pushBackStep([this]() {return isGoalDone(); },        // abort if
                 Always{},                                // proceed if
                 [this]() {return assignAttackerId(); },
                 "assignAttackerId");

    pushBackStep([this]() {return isGoalDone(); },                     // abort if
                 [this]() {return isAttackPhase() && canMove(); },     // proceed if
                 [this]() {return findAttackPos(); },
                 "findAttackPos");

    pushBackStep([this]() {return isGoalDone(); },                     // abort if
                 [this]() {return isAttackPhase() && canMove(); },     // proceed if
                 [this]() {return reachAttackPos(); },
                 "reachAttackPos");

    pushBackStep([this]() {return isGoalDone(); },        // abort if
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
    const Robot& me    = state().me();
    const Ball&  ball  = state().game().ball;
    const Rules& rules = state().rules();

    const double thresholdZ = -rules.arena.depth / 2 + rules.BALL_RADIUS * 2;

    return isAttacker() 
        && (   (me.z > 0 && ball.z > 0) 
            || (me.z <= ball.z && ball.z > thresholdZ));    // we are on enemy side or the ball is between attacker and enemy gates
}

bool AttackSingle::canMove() const
{
    return canMoveImpl(state().me());
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

    int meetingTick = std::numeric_limits<int>::max();
    for(const State::PredictedPos& prediction : predictions)
    {
        const double thresholdY = 2 * rules.BALL_RADIUS + rules.ROBOT_MIN_RADIUS;    // #todo - jump support
        const double thresholdZ_min = -rules.arena.depth / 2 + rules.BALL_RADIUS * 4;    // #todo - reimplement, wrote in hurry. Purpose: don't mess with goalkeeper

        if(prediction.m_tick < game.current_tick || prediction.m_pos.y > thresholdY || prediction.m_pos.z < thresholdZ_min)
            continue;

        // #todo: ticksToReach3D
        meetingTick = game.current_tick + ticksToReach3D(state().me(), prediction.m_pos, state().rules());
        if(meetingTick > prediction.m_tick)
            continue;

        m_attackPos = prediction.m_pos;              // #todo after goalkeeper: don't use exact ball pos!
        break;
    }

    return m_attackPos != Vec3d{} ? StepStatus::Done : StepStatus::Ok;
}

Goal::StepStatus AttackSingle::reachAttackPos()
{
    const auto& predictions = state().ballPredictions();
    const model::Game& game = state().game();
    const model::Rules& rules = state().rules();

    Entity<model::Robot> me = state().me();
    Entity<model::Ball>  ball = game.ball;


    auto itFound = std::find_if(predictions.begin(), predictions.end(), [this](const State::PredictedPos& prediction) {
        return prediction.m_pos == m_attackPos;
    });

    if(predictions.end() == itFound)
        return Goal::StepStatus::Done;     // abort this step, next step is entire goal repeat

    double secondsToWait = static_cast<double>(itFound->m_tick - game.current_tick) / rules.TICKS_PER_SECOND;
    Vec3d target = m_attackPos;
    // #todo - does not work (decreases kick speed)
//     double activeAttackZ = rules.arena.depth / 2 - 8 * rules.BALL_RADIUS;
//     double activeAttackX = rules.arena.goal_width / 2;
//     if(target.z > activeAttackZ && std::abs(target.x) < activeAttackX && )
//         target.z -= rules.ROBOT_MIN_RADIUS;

    Vec2d meXZ = { me.position().x, me.position().z };
    Vec2d targetXZ = { target.x, target.z };
    Vec2d displacementXZ = targetXZ - meXZ;

    double shorten = linalg::length(displacementXZ) / (linalg::length(displacementXZ) - rules.BALL_RADIUS - rules.ROBOT_MIN_RADIUS);
    if(shorten > 1)
        displacementXZ /= shorten;

    Vec2d directionXZ = linalg::normalize(displacementXZ);

    double distance = linalg::length(displacementXZ);
    //double secondsToReach = static_cast<double>(ticksToReach2D(me, m_attackPos, state().rules())) / rules.TICKS_PER_SECOND;

    const double attackTime = 3 / rules.TICKS_PER_SECOND;  // #bug ! this is always 0 (int)
    double needSpeedSI = secondsToWait > attackTime ? distance / secondsToWait : 2 * rules.ROBOT_MAX_GROUND_SPEED;  // actually, not so SI: length units per second
    Vec2d targetSpeedXZ = directionXZ * needSpeedSI;

    Action action;
    action.target_velocity_x = targetSpeedXZ[0];
    action.target_velocity_z = targetSpeedXZ[1];

    int ticksToArrive2D = ticksToReach2D(me, target/*ball?*/, rules/* #todo: custom acceleration = 0?*/);
    double desiredHeight = target.y - rules.ROBOT_MIN_RADIUS;

    auto predictedJump = jumpPrediction(state(), 
        [desiredHeight, &rules](const PredictedJumpHeight& jump) 
        { 
            return jump.m_initialSpeed == rules.ROBOT_MAX_JUMP_SPEED ? 1 / std::abs(jump.m_height - desiredHeight) : 0;
        });

    if(predictedJump && ticksToArrive2D <= predictedJump->m_timeToReach)
    {
        action.jump_speed = predictedJump->m_initialSpeed;
    }

    // #todo - actually, this may fail detection of collision on next tick if robot speed is high
    if(linalg::length2(ball.position() - me.position()) < pow2(rules.ROBOT_MAX_RADIUS + rules.BALL_RADIUS))
    {
        action.jump_speed = rules.ROBOT_MAX_JUMP_SPEED;

        double thresholdZ = rules.arena.depth / 2 - 4 * rules.BALL_RADIUS - rules.ROBOT_MIN_RADIUS;
        double thresholdX = rules.arena.goal_width / 2;
        double minDiffZ   = rules.BALL_RADIUS;

        if(std::abs(me.x) < thresholdX && std::abs(ball.x) < thresholdX && me.z > thresholdZ && (ball.z - me.z) > minDiffZ )
        {
            Vec3d hitSpeed = linalg::normalize(target - me.position()) * rules.MAX_ENTITY_SPEED;
            action.use_nitro = true;
            action.target_velocity_x = hitSpeed.x;
            action.target_velocity_y = hitSpeed.y;
            action.target_velocity_z = hitSpeed.z;
        }
    }

    DebugRender::instance().shpere(m_attackPos, rules.BALL_RADIUS + .1, { 1, 0, 0, .25 });
    DebugRender::instance().text(R"(attack by #%id pos: %pos, tick: %tick)"_fs
                           .format("%id", state().me().id)
                           .format("%pos", m_attackPos)
                           .format("%tick", itFound->m_tick)
                           .move());

    state().commitAction(action);
    return action.jump_speed == 0 ? StepStatus::Ok : StepStatus::Done;
}

bool AttackSingle::isGoalDone() const
{
    return state().isChilloutPhase();
}

bool AttackSingle::isCompatibleWith(const Goal* interrupted)
{
    return nullptr != dynamic_cast<const Goalkeeper*>(interrupted);    // can do these goals in parallel
}

