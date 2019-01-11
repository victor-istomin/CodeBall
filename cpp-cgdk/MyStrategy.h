#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include "Strategy.h"
#include "State.h"
#include "Simulator.h"
#include "forwards.h"

#include <memory>


class MyStrategy : public Strategy 
{
    std::unique_ptr<Simulator>   m_simulator;
    std::unique_ptr<State>       m_state;
    std::unique_ptr<GoalManager> m_goalManager;
    //std::unique_ptr<QuickStart_MyStrategy> m_qsGuy;   // might be used for simulation

    int         m_lastSimTick = -1;

public:
    MyStrategy();

    virtual void act(const model::Robot& me, const model::Rules& rules, const model::Game& world, model::Action& action) override;

    void simulateBallPos(const model::Game &game, const int simFrom, const int simUntil);

    Simulator::CollisionFlags simulateTick(const double tickTime, int microticksCount, std::vector<Entity<model::Robot>>& simRobots, Entity<model::Ball>& simBall);

    void debugRender(int ghostTick, const model::Game& game, double lastSimMs);

    virtual std::string custom_rendering() override;

private:
    void initJumpPredictions();
};

#endif
