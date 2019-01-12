#pragma once
#include "goal.h"
#include "goalUtils.h"

namespace goals
{

class Goalkeeper : public Goal
{
    bool isGoalkeeper() const;
    bool canMove() const;
    bool isDefendPhase();
    bool isGoalDone() const;

    StepStatus assignGoalkeeperId();
    StepStatus findDefendPos();
    StepStatus reachDefendPos();

    StepStatus repeat();

private:
    static constexpr const int NONE = -1;

    int   m_keeperId       = NONE;
    int   m_lastDefendTick = NONE;
    Vec3d m_defendPos      = {};

    virtual bool isCompatibleWith(const Goal* interrupted) override;

    int ticksFromLastDefence() const;

public:
    Goalkeeper(State& state, GoalManager& goalManager);
    ~Goalkeeper();
};


}

