#include "MyStrategy.h"
#include "QuickStart_MyStrategy.h"
#include <chrono>
#include <sstream>
#include "tests.h"

using namespace model;
static double g_sumDuration = 0;
static int    g_simsMade = 0;

MyStrategy::MyStrategy() 
{
}

#include <fstream>
#include <iostream>
MyStrategy::~MyStrategy()
{
    auto out = std::ofstream("..\\log_sim.txt", std::ios::app);
    out << "simd 128d + normalize: " << g_sumDuration << " ms, " << g_simsMade << " sims made, " << g_sumDuration / g_simsMade << " avg. ms" << std::endl;
    std::cout << "simd 128d + normalize: " << g_sumDuration << " ms, " << g_simsMade << " sims made, " << g_sumDuration / g_simsMade << " avg. ms" << std::endl;
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

    if(!m_qsGuy)
        m_qsGuy = std::make_unique<QuickStart_MyStrategy>();

    if(me.id % 2)
    {
        m_qsGuy->act(me, rules, game, action);
        return;
    }

    using EntityRobot = Entity<Robot>;
    using EntityBall  = Entity<Ball>;
    std::vector<EntityRobot> simRobots;
    EntityBall simBall = game.ball;

    for(const Robot& r : game.robots)
        simRobots.emplace_back(r);

    constexpr int SIM_TICKS = 90;
    int simUntil = (game.current_tick / SIM_TICKS * SIM_TICKS) + SIM_TICKS;
    const double tickTime = 1.0 / rules.TICKS_PER_SECOND;
    const double microtickTime = tickTime / rules.MICROTICKS_PER_TICK;

    auto start = std::chrono::high_resolution_clock::now();

    for(int tick = game.current_tick; tick != simUntil; ++tick)
    {

        for(int microtick = 0; microtick < rules.MICROTICKS_PER_TICK; ++microtick)
            m_simulator->update(simRobots, simBall, microtickTime);

        for(EntityRobot& r : simRobots)
        {
            Action unitAction;
            unitAction.target_velocity_x = r.velocity_x;
            unitAction.target_velocity_y = r.velocity_y;
            unitAction.target_velocity_z = r.velocity_z;
            r.setAction(unitAction);
        }
    }

    auto finish = std::chrono::high_resolution_clock::now();

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
    "Text": "pred. tick: %pt, act. tick: %at, sim. time: %st ms"
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

    double milliseconds = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(finish - start).count();
    g_sumDuration += milliseconds;
    ++g_simsMade;

    format(s, "%x", simBall.x);
    format(s, "%y", simBall.y);
    format(s, "%z", simBall.z);

    format(s, "%pt", simUntil);
    format(s, "%at", game.current_tick);
    format(s, "%pr", simBall.velocity());
    format(s, "%acv", EntityBall(game.ball).velocity());
    format(s, "%acp", EntityBall(game.ball).position());
    format(s, "%st", milliseconds);

    m_renderHint = s;
    
    m_qsGuy->act(me, rules, game, action);

    return;
}

