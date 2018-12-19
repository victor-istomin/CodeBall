#include "MyStrategy.h"

using namespace model;

MyStrategy::MyStrategy() { }

void MyStrategy::act(const Robot& me, const Rules& rules, const Game& game, Action& action)
{
    if(!m_simulator)
        m_simulator = std::make_unique<Simulator>(rules, MIN_HIT_E, MAX_HIT_E, ROBOT_MASS, BALL_MASS, BALL_ARENA_E, MAX_ENTITY_SPEED, GRAVITY, 1);
}
