#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include "Strategy.h"
#include "state.h"

#include <memory>

class QuickStart_MyStrategy;
class GoalManager;
class Simulator;

template <typename T> class Entity;

class MyStrategy : public Strategy 
{
    std::unique_ptr<Simulator>   m_simulator;
    std::unique_ptr<State>       m_state;
    std::unique_ptr<GoalManager> m_goalManager;
    //std::unique_ptr<QuickStart_MyStrategy> m_qsGuy;   // might be used for simulation
    std::string m_renderHint;

public:
    MyStrategy();

    virtual void act(const model::Robot& me, const model::Rules& rules, const model::Game& world, model::Action& action) override;

    void debugRender(Entity<model::Ball>& simBall, int simUntil, const model::Game& game, double lastSimMs);

    virtual std::string custom_rendering() override { return m_renderHint; }

};

#endif
