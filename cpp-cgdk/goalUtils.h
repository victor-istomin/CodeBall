#pragma once
#include "forwards.h"
#include "algebra.h"
#include <functional>

namespace goals
{
    struct Never  { bool operator()() const { return false; } };
    struct Always { bool operator()() const { return true; } };

    bool canMoveImpl(const model::Robot& r);
    int ticksToReach(const Vec3d& target, State& state);

    using JumpScorintgPredicate = std::function<double(const PredictedJumpHeight&)>;

    std::optional<PredictedJumpHeight> jumpPrediction(State& state, const JumpScorintgPredicate& scoring);

}


