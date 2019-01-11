#pragma once
#include "goal.h"
#include "goalUtils.h"

namespace goals
{

class AttackSingle : public Goal
{
    bool isAttacker() const;
    bool canMove() const;
    bool isAttackPhase() const;
    bool isGoalDone() const;

    StepStatus assignAttackerId();
    StepStatus findAttackPos();
    StepStatus reachAttackPos();

    StepStatus repeat();

    static constexpr const int NONE = -1;

    int   m_attackerId = NONE;
    Vec3d m_attackPos  = {};

public:
    AttackSingle(State& state, GoalManager& goalManager);
    ~AttackSingle();
private:
    virtual bool isCompatibleWith(const Goal* interrupted) override;

};

}

