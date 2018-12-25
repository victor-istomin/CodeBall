#include "GoalTakeBallPair.h"
#include "Entity.h"
#include "State.h"
#include "Simulator.h"
#include "physicsUtils.h"

using namespace goals;
using Vec2d = Simulator::Vec2d;
using Vec3d = Simulator::Vec3d;

struct Never  { bool operator()() const { return false; } };
struct Always { bool operator()() const { return true; } };

TakeBallPair::TakeBallPair(State& state, GoalManager& goalManager)
    : Goal(state, goalManager)
{
    pushBackStep(Never()/*abort*/, Always()/*proceed*/, [this]() { return this->rushIntoBall(); }, "rushIntoBall");
}

TakeBallPair::~TakeBallPair()
{
}

// #todo - move to MathUtils.h


Goal::StepStatus goals::TakeBallPair::rushIntoBall()
{
    Entity<model::Ball>  ball  = state().game().ball;
    Entity<model::Robot> me    = state().me();
    const model::Rules&  rules = state().rules();

    Vec2d meXZ   = { me.position().x, me.position().z };
    Vec2d ballXZ = { ball.position().z, ball.position().z };
    Vec2d displacementXZ = ballXZ - meXZ;
    Vec2d directionXZ    = linalg::normalize(displacementXZ);

    Vec2d meVelocityXZ   = Vec2d{ me.velocity().x, me.velocity().z };
    Vec2d ballVelocityXZ = Vec2d{ ball.velocity().x, ball.velocity().z };
    double approachSpeed = linalg::dot(meVelocityXZ, directionXZ) - linalg::dot(ballVelocityXZ, directionXZ);
    double maxApproachSpeed = rules.ROBOT_MAX_GROUND_SPEED - linalg::dot(ballVelocityXZ, directionXZ);

    // #todo_r2 - nitro support
    const double acceleration = approachSpeed < maxApproachSpeed ? (rules.ROBOT_ACCELERATION / rules.TICKS_PER_SECOND) : 0;

    double distanceXZ = linalg::length(displacementXZ);
    double approachTimeSecs = uniform_accel::distanceToTime(distanceXZ, rules.ROBOT_ACCELERATION, approachSpeed, maxApproachSpeed); // #todo - nitro
    double approachTimeTics = approachTimeSecs * rules.TICKS_PER_SECOND;

    Vec3d newVelocity = linalg::normalize(Vec3d{ displacementXZ[0], 0, displacementXZ[1] }) * (linalg::length(me.velocity()) + acceleration);

    model::Action action;
    action.target_velocity_x = newVelocity.x;
    action.target_velocity_z = newVelocity.z;

    if(linalg::length2(ball.position() - me.position()) < pow2(state().rules().ROBOT_MAX_RADIUS + state().rules().BALL_RADIUS))
        action.jump_speed = state().rules().ROBOT_MAX_JUMP_SPEED;

    static bool askTeammateToJump = false;

    // decide whether it's reasonable to jump
    std::optional<Simulator::Vec3d> predictedBallPos = state().predictedBallPos((int)approachTimeTics + state().game().current_tick);
    if(predictedBallPos.has_value())
    {
        double desiredHeight = predictedBallPos->y;
        double ticksToLift   = uniform_accel::distanceToTime(desiredHeight, -rules.GRAVITY, rules.ROBOT_MAX_JUMP_SPEED) * rules.TICKS_PER_SECOND;
        if(ticksToLift > approachTimeTics)
        {
            action.jump_speed = state().rules().ROBOT_MAX_JUMP_SPEED;
            askTeammateToJump = true;   // in order to approach ball simultaneously
        }
    }

    // #todo - this is a proof-of-concept quality code...
    if(askTeammateToJump && action.jump_speed == 0)
    {
        action.jump_speed = state().rules().ROBOT_MAX_JUMP_SPEED;
        askTeammateToJump = false;   // in order to approach ball simultaneously
    }

    state().commitAction(action);

    return StepStatus::Ok;
}

