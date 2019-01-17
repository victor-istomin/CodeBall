#pragma once
#include "forwards.h"
#include "algebra.h"
#include <functional>

namespace goals
{
    struct Never  { bool operator()() const { return false; } };
    struct Always { bool operator()() const { return true; } };

    bool canMoveImpl(const model::Robot& r);
    int ticksToReach2D(const Entity<model::Robot>& robot, const Vec3d& target, const model::Rules& rules);
    int ticksToReach3D(const Entity<model::Robot>& robot, const Vec3d& target, const model::Rules& rules);

    /**/
    double timeToReach2D_Ex(const Vec3d& from, const Vec3d& target, const Vec3d& v0, double acceleration, double maxSpeed);
    double timeToReach3D_Ex(const Vec3d& from, const Vec3d& target, const Vec3d& v0, const model::Rules& rules);
    /**/

    using JumpScorintgPredicate = std::function<double(const PredictedJumpHeight&)>;

    std::optional<PredictedJumpHeight> jumpPrediction(State& state, const JumpScorintgPredicate& scoring);

}


