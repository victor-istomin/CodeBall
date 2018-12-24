#include "MyStrategy.h"
#include "QuickStart_MyStrategy.h"
#include "Simulator.h"
#include "goalManager.h"

#include <chrono>
#include <sstream>
#include "tests.h"

using namespace model;

static double g_simDurationMs = 0;
static int    g_simCount = 0;

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
    if(!m_goalManager)
        m_goalManager = std::make_unique<GoalManager>(*m_state);

    m_state->updateState(me, rules, game, action);

    if(0 == (me.id % 2) /*&& 0 == (game.current_tick % 2)*/)
    {

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

        auto startSimTime = std::chrono::high_resolution_clock::now();

        for(int tick = game.current_tick; tick != simUntil; ++tick)
        {
            Action unitAction;

            for(int microtick = 0; microtick < rules.MICROTICKS_PER_TICK; ++microtick)
                m_simulator->update(simRobots, simBall, microtickTime);

            for(EntityRobot& r : simRobots)
                r.setAction(unitAction);
        }

        auto finishSimTime = std::chrono::high_resolution_clock::now();
        double lastSimTime = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(finishSimTime - startSimTime).count();
        g_simDurationMs += lastSimTime;
        ++g_simCount;

        debugRender(simBall, simUntil, game, lastSimTime);
    }

    m_goalManager->tick();

    return;
}

void MyStrategy::debugRender(Entity<model::Ball>& simBall, int simUntil, const model::Game& game, double lastSimMs)
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
        "Text": "p. tick: %pt, act. tick: %at, last sim time: %lst, avg. sim time: %ast"
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
    format(s, "%lst", lastSimMs);
    format(s, "%ast", g_simCount == 0 ? 0 : (g_simDurationMs / g_simCount));
    format(s, "%pr", simBall.velocity());
    format(s, "%acv", Entity<model::Ball>(game.ball).velocity());
    format(s, "%acp", Entity<model::Ball>(game.ball).position());

    m_renderHint = std::move(s);
#else    // #ifdef DEBUG_RENDER
    m_renderHint.clear();
#endif   // #ifdef DEBUG_RENDER
}

