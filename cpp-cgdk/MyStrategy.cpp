#include "MyStrategy.h"

using namespace model;

MyStrategy::MyStrategy() 
{
}

void MyStrategy::act(const Robot&, const Rules& rules, const Game&, Action&)
{
    if(!m_simulator)
        m_simulator = std::make_unique<Simulator>(rules, 1);
}
