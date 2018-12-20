#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include "Strategy.h"
#include "Simulator.h"
#include <memory>

class QuickStart_MyStrategy;

class MyStrategy : public Strategy 
{
    std::unique_ptr<Simulator> m_simulator;
    std::unique_ptr<QuickStart_MyStrategy> m_qsGuy;   // might be used for simulation

public:
    MyStrategy();

    void act(const model::Robot& me, const model::Rules& rules, const model::Game& world, model::Action& action) override;
};

#endif
