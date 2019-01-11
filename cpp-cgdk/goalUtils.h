#pragma once
#include "forwards.h"
#include "algebra.h"

namespace goals
{
    struct Never  { bool operator()() const { return false; } };
    struct Always { bool operator()() const { return true; } };

    bool canMoveImpl(const model::Robot& r);
    int ticksToReach(const Vec3d& target, State& state);

    std::optional<PredictedJumpHeight> jumpPrediction(double desiredHeight, State& state);

}


