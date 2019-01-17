#include "goalUtils.h"
#include "model/Robot.h"
#include "Simulator.h"
#include "State.h"
#include "physicsUtils.h"

using namespace goals;
using namespace model;

bool goals::canMoveImpl(const model::Robot& r)
{
    return r.touch  // on ground
        || r.nitro_amount > 0;
}

int goals::ticksToReach2D(const Entity<model::Robot>& robot, const Vec3d& target, const model::Rules& rules)
{
    Vec2d robotXZ        = { robot.position().x, robot.position().z };
    Vec2d targetXZ       = { target.x, target.z };
    Vec2d displacementXZ = targetXZ - robotXZ;
    Vec2d directionXZ    = linalg::normalize(displacementXZ);

    double shorten = linalg::length(displacementXZ) / (linalg::length(displacementXZ) - rules.BALL_RADIUS - rules.ROBOT_MIN_RADIUS);
    if(shorten > 1)
        displacementXZ /= shorten;

    Vec2d meVelocityXZ = Vec2d{ robot.velocity().x, robot.velocity().z };
    double approachSpeed = linalg::dot(meVelocityXZ, directionXZ);
    double maxApproachSpeed = rules.ROBOT_MAX_GROUND_SPEED;

    double distanceXZ = linalg::length(displacementXZ);
    double approachTimeSecs = uniform_accel::distanceToTime(distanceXZ, rules.ROBOT_ACCELERATION, approachSpeed, maxApproachSpeed); // #todo st.2 - nitro
    double approachTimeTics = approachTimeSecs * rules.TICKS_PER_SECOND;

    return static_cast<int>(std::ceil(approachTimeTics));
}

int goals::ticksToReach3D(const Entity<model::Robot>& robot, const Vec3d& target, const model::Rules& rules)
{
    // #todo - st.2 nitro
    int ticks2D = ticksToReach2D(robot, target, rules);
    
    double liftAmount = target.y - robot.y - (rules.BALL_RADIUS - rules.ROBOT_MIN_RADIUS);
    if(liftAmount <= 0)
        return ticks2D;

    Vec2d displacementXZ  = Vec2d(target.x, target.z) - Vec2d(robot.position().x, robot.position().z);
    Vec2d directionXZ     = linalg::normalize(displacementXZ);
    double v0             = linalg::dot(Vec2d{ robot.velocity().x, robot.velocity().z }, directionXZ);
    double distance2D     = linalg::length(displacementXZ) - rules.ROBOT_MIN_RADIUS - rules.BALL_RADIUS;
    double acceleration2D = distance2D > 0 ? rules.ROBOT_ACCELERATION : -rules.ROBOT_ACCELERATION;

    const double tickTime = 1.0 / rules.TICKS_PER_SECOND;
    double secondsToLift = uniform_accel::distanceToTime(liftAmount, -rules.GRAVITY, rules.ROBOT_MAX_JUMP_SPEED);
    double takeoffTick = std::max(0.0, ticks2D - secondsToLift * rules.TICKS_PER_SECOND);
    double takeoffSecond = takeoffTick * tickTime;

    double takeoffSpeed  = uniform_accel::timeToSpeed(v0, acceleration2D, takeoffSecond, rules.ROBOT_MAX_GROUND_SPEED);
    double runawayLength = uniform_accel::timeToDistance(v0, acceleration2D, takeoffSecond, rules.ROBOT_MAX_GROUND_SPEED);
    double runawayTicks  = uniform_accel::distanceToTime(runawayLength, acceleration2D, v0, rules.ROBOT_MAX_GROUND_SPEED) * rules.TICKS_PER_SECOND;
    int dbg_check = (int)std::abs(runawayTicks - takeoffTick);
    // assert(dbg_check < 1); #todo - debug this case

    double flyingDistance = distance2D - runawayLength;
    double flyingSeconds  = flyingDistance / takeoffSpeed;

    while(flyingSeconds > secondsToLift && (rules.ROBOT_MAX_GROUND_SPEED - std::abs(takeoffSpeed)) > k_Epsilon)
    {
        // takeoff speed is too low, adjust
        takeoffTick += 1;
        takeoffSecond = takeoffTick * tickTime;

        takeoffSpeed  = uniform_accel::timeToSpeed(v0, acceleration2D, takeoffSecond, rules.ROBOT_MAX_GROUND_SPEED);
        runawayLength = uniform_accel::timeToDistance(v0, acceleration2D, takeoffSecond, rules.ROBOT_MAX_GROUND_SPEED);

        flyingDistance = distance2D - runawayLength;
        flyingSeconds  = flyingDistance / takeoffSpeed;
    }

    return (int)std::ceil(takeoffTick + flyingSeconds * rules.TICKS_PER_SECOND);
}

std::optional<PredictedJumpHeight> goals::jumpPrediction(State& state, const JumpScorintgPredicate& scoring)
{
    const State::JumpPredictionMap& predictionsMap = state.jumpPredictions();

    const PredictedJumpHeight* best = nullptr;
    double bestScore = std::numeric_limits<double>::min();

    for(auto itVelocityJumpPair = predictionsMap.rbegin(); itVelocityJumpPair != predictionsMap.rend(); ++itVelocityJumpPair)
    {
        const std::vector<PredictedJumpHeight>& predictions = itVelocityJumpPair->second;

        for(const PredictedJumpHeight& predicted: predictions)
        {
            if(predicted.m_velocity_y < 0)
                break;   // look at upwards movement phase only

            double nextScore = scoring(predicted);

            if(best == nullptr || nextScore > bestScore)
            {
                bestScore = nextScore;
                best = &predicted;
            }
        }
    }

    if(best != nullptr)
        return *best;

    return std::nullopt;
}
