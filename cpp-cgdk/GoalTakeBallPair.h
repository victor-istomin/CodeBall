#pragma once
#include "goal.h"
#include "State.h"
#include <optional>

namespace goals
{

class TakeBallPair : public Goal
{
    StepStatus rushIntoBall();
    StepStatus rushIntoGround();

    static constexpr const int TICK_NONE = -1;

    int m_lastTick        = TICK_NONE;
    int m_steerGroundTick = TICK_NONE;

    bool isBallReached();

public:
    TakeBallPair(State& state, GoalManager& goalManager);
    ~TakeBallPair();
};

}

