#include "GoalTakeBallPair.h"
#include "Entity.h"
#include "State.h"
#include "Simulator.h"
#include "physicsUtils.h"
#include "goalUtils.h"

using namespace goals;


TakeBallPair::TakeBallPair(State& state, GoalManager& goalManager)
    : Goal(state, goalManager)
{
    pushBackStep([this]() { return this->isBallReached(); }, 
                  Always()/*proceed*/, 
                 [this]() { return this->rushIntoBall(); }, "rushIntoBall");
}

TakeBallPair::~TakeBallPair()
{
}

Goal::StepStatus TakeBallPair::rushIntoBall()
{
    Entity<model::Ball>  ball  = state().game().ball;
    Entity<model::Robot> me    = state().me();
    const model::Rules&  rules = state().rules();

    Vec2d meXZ   = { me.position().x, me.position().z };
    Vec2d ballXZ = { ball.position().z, ball.position().z };
    Vec2d displacementXZ = ballXZ - meXZ;
    Vec2d directionXZ    = linalg::normalize(displacementXZ);

    double shorten = linalg::length(displacementXZ) / (linalg::length(displacementXZ) - rules.BALL_RADIUS - rules.ROBOT_MIN_RADIUS);
    if(shorten > 1)
        displacementXZ /= shorten;

    Vec2d meVelocityXZ   = Vec2d{ me.velocity().x, me.velocity().z };
    Vec2d ballVelocityXZ = Vec2d{ ball.velocity().x, ball.velocity().z };
    double approachSpeed = linalg::dot(meVelocityXZ, directionXZ) - linalg::dot(ballVelocityXZ, directionXZ);
    double maxApproachSpeed = rules.ROBOT_MAX_GROUND_SPEED - linalg::dot(ballVelocityXZ, directionXZ);

    double distanceXZ = linalg::length(displacementXZ);
    double approachTimeSecs = uniform_accel::distanceToTime(distanceXZ, rules.ROBOT_ACCELERATION, approachSpeed, maxApproachSpeed); // #todo st.2 - nitro
    double approachTimeTics = approachTimeSecs * rules.TICKS_PER_SECOND;

    Vec3d newVelocity = linalg::normalize(Vec3d{ displacementXZ[0], 0, displacementXZ[1] }) * rules.ROBOT_MAX_GROUND_SPEED;         // #todo st.2 - nitro

    model::Action action;
    action.target_velocity_x = newVelocity.x;
    action.target_velocity_z = newVelocity.z;

    if(linalg::length2(ball.position() - me.position()) < pow2(state().rules().ROBOT_MAX_RADIUS + state().rules().BALL_RADIUS))
    {
        // first bot just touch ball, second one will hit it with jump speed
        action.jump_speed = state().rules().ROBOT_MAX_JUMP_SPEED;
        m_lastTick = state().game().current_tick;
    }

    // decide whether it's reasonable to jump

    static bool askTeammateToJump = false;
    int collisionTick = static_cast<int>(approachTimeTics) + state().game().current_tick;
    std::optional<State::PredictedPos> predictedBallPos = state().predictedBallPos(collisionTick);
    if(predictedBallPos.has_value() && me.touch)   // only if robot touched ground
    {
        double desiredHeight = predictedBallPos->m_pos.y - (rules.ROBOT_MAX_RADIUS - rules.ROBOT_MIN_RADIUS);
        std::optional<PredictedJumpHeight> predictedJump = jumpPrediction(desiredHeight, state());

        static constexpr double ACCURACY_THRESHOLD = 0.5;    // low accuracy because of a fixed jump speed
        if(predictedJump.has_value() && std::abs(predictedJump->m_height - desiredHeight) < ACCURACY_THRESHOLD)
        {
            double ticksToLift = predictedJump->m_timeToReach;
            //double ticksToLift2 = uniform_accel::distanceToTime(desiredHeight - me.radius, -rules.GRAVITY, rules.ROBOT_MAX_JUMP_SPEED) * rules.TICKS_PER_SECOND;
            if(ticksToLift > approachTimeTics)
            {
                action.jump_speed = state().rules().ROBOT_MAX_JUMP_SPEED;
                askTeammateToJump = true;
            }
        }
    }

    // #todo - this is a proof-of-concept quality code...
    if(askTeammateToJump && action.jump_speed == 0)
    {
        action.jump_speed = state().rules().ROBOT_MAX_JUMP_SPEED; 
        askTeammateToJump = false;
    }

    state().commitAction(action);

    return StepStatus::Ok;
}

bool TakeBallPair::isBallReached()
{
    if(m_lastTick == TICK_NONE && !state().isBallAtStartPos())
        m_lastTick = state().game().current_tick;    // in case if enemy or teammate has reached the ball

    return m_lastTick != TICK_NONE && state().game().current_tick > m_lastTick;
}

