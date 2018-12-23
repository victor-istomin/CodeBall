#pragma once
#include "goal.h"

namespace goals
{

class TakeBallPair : public Goal
{
    StepStatus rushIntoBall();

public:
    TakeBallPair(State& state, GoalManager& goalManager);
    ~TakeBallPair();
};

}

