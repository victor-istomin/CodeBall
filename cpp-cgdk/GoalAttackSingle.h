#pragma once
#include "goal.h"
#include "Simulator.h"

namespace goals
{

class AttackSingle : public Goal
{
    using Vec2d = Simulator::Vec2d;
    using Vec3d = Simulator::Vec3d;

    bool isAttacker() const;
    bool canMove() const;
    bool isAttackPhase() const;
    bool isGoalDone() const;

    StepStatus assignAttackerId();
    StepStatus findAttackPos();
    StepStatus reachAttackPos();

    StepStatus repeat();

    int ticksToReach(const Vec3d& pos) const;

    static constexpr const int NONE = -1;

    int   m_attackerId = NONE;
    Vec3d m_attackPos  = {};

public:
    AttackSingle(State& state, GoalManager& goalManager);
    ~AttackSingle();
};

}

