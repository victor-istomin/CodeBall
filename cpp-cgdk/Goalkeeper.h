#pragma once
#include "goal.h"
#include "goalUtils.h"

namespace goals
{

class Goalkeeper : public Goal
{
    bool isGoalkeeper() const;
    bool canMove() const;
    bool isDefendPhase() const;
    bool isGoalDone() const;

    StepStatus assignGoalkeeperId();
    StepStatus findDefendPos();
    StepStatus reachDefendPos();

    StepStatus repeat();

private:
    static constexpr const int NONE = -1;

    int   m_keeperId  = NONE;
    Vec3d m_defendPos = {};

    virtual bool isCompatibleWith(const Goal* interrupted) override;

public:
    Goalkeeper(State& state, GoalManager& goalManager);
    ~Goalkeeper();
};


}

