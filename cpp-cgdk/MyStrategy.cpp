#include "MyStrategy.h"
#include "QuickStart_MyStrategy.h"
#include <chrono>
#include "tests.h"

using namespace model;

MyStrategy::MyStrategy() 
{
}

void MyStrategy::act(const Robot& me, const Rules& rules, const Game& game, Action& action)
{
    if(!m_simulator)
    {
        m_simulator = std::make_unique<Simulator>(rules);
        m_simulator->Test_Collide();
    }

    Test_Simulator(me, game, action, rules, m_simulator);
    return;
}
