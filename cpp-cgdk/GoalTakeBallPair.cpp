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
    Entity<model::Ball> ball = state().game().ball;
    Entity<model::Robot>  me = state().me();

    Vec2d meXZ   = { me.position().x, me.position().z };
    Vec2d ballXZ = { ball.position().z, ball.position().z };

    Vec2d approachSpeedXZ = Vec2d{ me.velocity().x, me.velocity().z } -Vec2d{ ball.velocity().x, ball.velocity().z };

    // #todo_r2 - nitro support
    static const double MAX_TICK_ACCELERATION = state().rules().ROBOT_ACCELERATION / state().rules().TICKS_PER_SECOND;

    double distanceXZ = linalg::distance(meXZ, ballXZ);
    double minTime = timeToApproach(distanceXZ, linalg::length(approachSpeedXZ), MAX_TICK_ACCELERATION);

    Vec3d displacement = ball.position() - me.position();

    Vec3d newVelocity = linalg::normalize(Vec3d{ displacement.x, 0, displacement.z }) * (linalg::length(me.velocity()) + MAX_TICK_ACCELERATION);

    model::Action action;
    action.target_velocity_x = newVelocity.x;
    action.target_velocity_z = newVelocity.z;

    if(linalg::length2(ball.position() - me.position()) < pow2(state().rules().ROBOT_MAX_RADIUS + state().rules().BALL_RADIUS))
        action.jump_speed = state().rules().ROBOT_MAX_JUMP_SPEED;

    state().commitAction(action);

    return StepStatus::Ok;
}

