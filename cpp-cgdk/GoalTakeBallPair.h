#pragma once
#include "goal.h"

namespace goals
{

class TakeBallPair : public Goal
{
    StepStatus rushIntoBall();

    static constexpr const int TICK_NONE = -1;

    int m_lastTick = TICK_NONE;

    bool isLastTick() const;
    bool isFinished() const;

public:
    TakeBallPair(State& state, GoalManager& goalManager);
    ~TakeBallPair();
};

}

