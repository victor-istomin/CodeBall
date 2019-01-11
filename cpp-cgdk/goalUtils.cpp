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

int goals::ticksToReach(const Vec3d& target, State& state)
{
    Entity<model::Robot> me    = state.me();
    const model::Rules&  rules = state.rules();

    Vec2d meXZ           = { me.position().x, me.position().z };
    Vec2d targetXZ       = { target.x, target.z };
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
