#include "MyStrategy.h"
#include "QuickStart_MyStrategy.h"
#include <chrono>
#include <sstream>
#include "tests.h"

using namespace model;

MyStrategy::MyStrategy() 
{
}

namespace std
{
    std::string to_string(const Simulator::Vec3d& v)
    {
        return "(" + std::to_string(v.x) + "; " + std::to_string(v.y) + "; " + std::to_string(v.z) + ")";
    }
}

void MyStrategy::act(const Robot& me, const Rules& rules, const Game& game, Action& action)
{
    if(!m_simulator)
        m_simulator = std::make_unique<Simulator>(rules);
    if(!m_state)
        m_state = std::make_unique<State>();

    m_state->updateState(me, rules, game, action);


    if(me.id % 2)
        return;

    using EntityRobot = Entity<Robot>;
    using EntityBall  = Entity<Ball>;
    std::vector<EntityRobot> simRobots;
    EntityBall simBall = game.ball;

    for(const Robot& r : game.robots)
        simRobots.emplace_back(r);

    constexpr int SIM_TICKS = 50;
    int simUntil = (game.current_tick / SIM_TICKS * SIM_TICKS) + SIM_TICKS;
    const double tickTime = 1.0 / rules.TICKS_PER_SECOND;
    const double microtickTime = tickTime / rules.MICROTICKS_PER_TICK;

    for(int tick = game.current_tick; tick != simUntil; ++tick)
    {
        Action unitAction;

        for(int microtick = 0; microtick < rules.MICROTICKS_PER_TICK; ++microtick)
            m_simulator->update(simRobots, simBall, microtickTime);

        for(EntityRobot& r : simRobots)
            r.setAction(action);
    }

    debugRender(simBall, simUntil, game);

    return;
}

void MyStrategy::debugRender(Entity<model::Ball>& simBall, int simUntil, const model::Game& game)
{
#ifdef DEBUG_RENDER
    std::string s = R"(
    [
      {
        "Sphere": {
          "x": %x,
          "y": %y,
          "z": %z,
          "radius": 2,
          "r": 0.0,
          "g": 1.0,
          "b": 1.0,
          "a": 0.5
        }
      },
      {
        "Text": "p. tick: %pt, act. tick: %at"
      },
      {
        "Text": "Sphere x: %x, y: %y, z: %z,    p. velocity: %prv"
      },
      {
        "Text": "act. pos: %acp,    act. v: %acv"
      }
    ])";

    auto format = [](std::string& s, const std::string& spec, const auto& value)
    {
        auto pos = s.find(spec);
        auto val = std::to_string(value);
        while(pos != std::string::npos)
        {
            s.replace(pos, spec.size(), val);
            pos = s.find(spec);
        }
    };

    using EntityBall = Entity<Ball>;

    format(s, "%x", simBall.x);
    format(s, "%y", simBall.y);
    format(s, "%z", simBall.z);

    format(s, "%pt", simUntil);
    format(s, "%at", game.current_tick);
    format(s, "%pr", simBall.velocity());
    format(s, "%acv", Entity<model::Ball>(game.ball).velocity());
    format(s, "%acp", Entity<model::Ball>(game.ball).position());

    m_renderHint = std::move(s);
#else    // #ifdef DEBUG_RENDER
    m_renderHint.clear();
#endif   // #ifdef DEBUG_RENDER
}

