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
    pushBackStep([this]() { return this->isFinished(); }, Always()/*proceed*/, [this]() { return this->rushIntoBall(); }, "rushIntoBall");
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

    double shorten = linalg::length(displacementXZ) / (linalg::length(displacementXZ) - rules.BALL_RADIUS - rules.ROBOT_MIN_RADIUS);
    displacementXZ /= shorten;    // #todo - leads to incorrect approachTimeTics in case of distance is very small

    Vec2d meVelocityXZ   = Vec2d{ me.velocity().x, me.velocity().z };
    Vec2d ballVelocityXZ = Vec2d{ ball.velocity().x, ball.velocity().z };
    double approachSpeed = linalg::dot(meVelocityXZ, directionXZ) - linalg::dot(ballVelocityXZ, directionXZ);
    double maxApproachSpeed = rules.ROBOT_MAX_GROUND_SPEED - linalg::dot(ballVelocityXZ, directionXZ);

    // #todo_r2 - nitro support
    const double acceleration = approachSpeed < maxApproachSpeed ? (rules.ROBOT_ACCELERATION / rules.TICKS_PER_SECOND) : 0;

    double distanceXZ = linalg::length(displacementXZ);
    double approachTimeSecs = uniform_accel::distanceToTime(distanceXZ, rules.ROBOT_ACCELERATION, approachSpeed, maxApproachSpeed); // #todo st.2 - nitro
    double approachTimeTics = approachTimeSecs * rules.TICKS_PER_SECOND;

    Vec3d newVelocity = linalg::normalize(Vec3d{ displacementXZ[0], 0, displacementXZ[1] }) * rules.ROBOT_MAX_GROUND_SPEED;         // #todo st.2 - nitro

    model::Action action;
    action.target_velocity_x = newVelocity.x;
    action.target_velocity_z = newVelocity.z;

    if(linalg::length2(ball.position() - me.position()) < pow2(state().rules().ROBOT_MAX_RADIUS + state().rules().BALL_RADIUS))
    {
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
        std::optional<State::PredictedJumpHeight> predictedJump = jumpPrediction(desiredHeight);

        static constexpr double ACCURACY_THRESHOLD = 0.5;    // low accuracy because of a fixed jump speed
        if(predictedJump.has_value() && std::abs(predictedJump->m_height - desiredHeight) < ACCURACY_THRESHOLD)
        {
            double ticksToLift = predictedJump->m_timeToReach;
            double ticksToLift2 = uniform_accel::distanceToTime(desiredHeight - me.radius, -rules.GRAVITY, rules.ROBOT_MAX_JUMP_SPEED) * rules.TICKS_PER_SECOND;
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

bool goals::TakeBallPair::isLastTick() const
{
    return m_lastTick != TICK_NONE && state().game().current_tick == m_lastTick;
}

bool goals::TakeBallPair::isFinished() const
{
    return m_lastTick != TICK_NONE && state().game().current_tick > m_lastTick;
}

std::optional<State::PredictedJumpHeight> TakeBallPair::jumpPrediction(double desiredHeight) const
{
    const std::vector<State::PredictedJumpHeight>& predictions = state().jumpPredictions();

    const State::PredictedJumpHeight* best = nullptr;
    for(const State::PredictedJumpHeight& predicted : predictions)
    {
        if(predicted.m_velocity_y < 0)
            break;   // look at upwards movement phase only

        static constexpr const double JUMP_OVER_PENALTY  = 2;
        static constexpr const double JUMP_UNDER_PENALTY = -1;

        double bestDifference = best == nullptr ? std::numeric_limits<double>::max() : (best->m_height - desiredHeight);
        double nextDifference = predicted.m_height - desiredHeight;

        bestDifference *= bestDifference > 0 ? JUMP_OVER_PENALTY : JUMP_UNDER_PENALTY;
        nextDifference *= nextDifference > 0 ? JUMP_OVER_PENALTY : JUMP_UNDER_PENALTY;

        if(best == nullptr || nextDifference < bestDifference)
        {
            best = &predicted;
        }
    }

    if(best != nullptr)
        return *best;
    
    return std::nullopt;
}

