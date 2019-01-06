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
static int    g_simTicks = 0;

MyStrategy::MyStrategy() 
{
}

namespace std
{
    std::string to_string(const Simulator::Vec3d& v)
    {
        return "(" + std::to_string(v.x) + "; " + std::to_string(v.y) + "; " + std::to_string(v.z) + ")";
    }

    std::string to_string(const std::string& s)
    {
        return s;
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
    if(m_state->jumpPredictions().empty())
        initJumpPredictions();

    if(m_lastSimTick < game.current_tick && m_state->roundLocalTick() >= 0)
    {
        m_state->invalidateBallPredictions();
        m_lastSimTick = game.current_tick;

        constexpr int SIM_TICKS = 4*60;
        const int simUntil  = game.current_tick + SIM_TICKS;
        const int simFrom = game.current_tick;
        const int ghostTick = (game.current_tick / SIM_TICKS * SIM_TICKS) + SIM_TICKS;

        auto startSimTime = std::chrono::high_resolution_clock::now();

        simulateBallPos(game, simFrom, simUntil);

        auto finishSimTime = std::chrono::high_resolution_clock::now();
        double lastSimTime = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(finishSimTime - startSimTime).count();
        g_simDurationMs += lastSimTime;
        ++g_simCount;
        g_simTicks += simUntil - simFrom;

        debugRender(ghostTick, game, lastSimTime);
    }

    m_goalManager->tick();

    return;
}

void MyStrategy::simulateBallPos(const Game &game, const int simFrom, const int simUntil)
{
    using EntityRobot = Entity<Robot>;
    using EntityBall = Entity<Ball>;
    std::vector<EntityRobot> simRobots, simRobotsBackup;
    EntityBall simBall = game.ball;
    EntityBall simBallBackup;

    for(const Robot& r : game.robots)
        simRobots.emplace_back(r);

    const Rules& rules = m_state->rules();
    const double tickTime = 1.0 / rules.TICKS_PER_SECOND;
    for(int tick = simFrom; tick != simUntil; ++tick)
    {
        double minDistance = std::numeric_limits<double>::max();
        double maxVelocity = 0;
        for(const EntityRobot& r : simRobots)
        {
            minDistance = std::min(minDistance, linalg::distance(r.position(), simBall.position()));
            maxVelocity = std::max(maxVelocity, linalg::length(r.velocity()));
        }

        maxVelocity += linalg::length(simBall.velocity());
        minDistance -= rules.ROBOT_MAX_RADIUS + rules.BALL_RADIUS;
        bool noCollisionExpected = minDistance > (maxVelocity / rules.TICKS_PER_SECOND * 2);
        bool noArenaCollisionExpected = noCollisionExpected;
        double arenaApproachSpeed = 0;
        if(noArenaCollisionExpected)
        {
            auto danToArena = m_simulator->dan_to_arena(simBall.position());
            arenaApproachSpeed = -1 * linalg::dot(simBall.velocity(), danToArena.normal) / rules.TICKS_PER_SECOND; 
            double distanceToArena = danToArena.distance - rules.BALL_RADIUS;

            noArenaCollisionExpected = distanceToArena > (1.2 * arenaApproachSpeed);
        }

        int microticksCount = rules.MICROTICKS_PER_TICK;
        if(noCollisionExpected && noArenaCollisionExpected)
            microticksCount /= 4;

        const int FAR_TICK = rules.TICKS_PER_SECOND / 2;
        constexpr int MAX_COLLISION_OPTIMIZATION_FACTOR = 4;
        int farTimeFactor = 1 + (tick - game.current_tick) / FAR_TICK;
        if(!noArenaCollisionExpected)
            farTimeFactor = std::min(MAX_COLLISION_OPTIMIZATION_FACTOR, farTimeFactor);
        if(farTimeFactor > 1)
            microticksCount /= farTimeFactor;

        if(microticksCount != rules.MICROTICKS_PER_TICK)
        {
            // try optimized simulation, revert if necessary
            simRobotsBackup = simRobots;
            simBallBackup   = simBall;

            auto collisions = simulateTick(tickTime, microticksCount, simRobots, simBall);

            const double velocityThreshold = 0.1 * farTimeFactor;
            if(collisions.ball && noArenaCollisionExpected && arenaApproachSpeed > velocityThreshold)
            {
                // simulate again with fair micro-tics amount
                std::swap(simRobotsBackup, simRobots);
                simBall = simBallBackup;
                simulateTick(tickTime, rules.MICROTICKS_PER_TICK, simRobots, simBall);
            }
        }
        else
        {
            // start with fair simulation
            simulateTick(tickTime, microticksCount, simRobots, simBall);
        }


        m_state->saveBallPos(tick, simBall.position(), simBall.velocity());

        Action unitAction;
        for(EntityRobot& r : simRobots)
            r.setAction(unitAction);
    }
}

Simulator::CollisionFlags MyStrategy::simulateTick(const double tickTime, int microticksCount, std::vector<Entity<Robot>>& simRobots, Entity<Ball>& simBall)
{
    Simulator::CollisionFlags collision;

    const double microtickTime = tickTime / microticksCount;
    for(int microtick = 0; microtick < microticksCount; ++microtick)
        collision.apply(m_simulator->update(simRobots, simBall, microtickTime));

    return collision;
}

void MyStrategy::debugRender(int ghostTick, const model::Game& game, double lastSimMs)
{
#ifdef DEBUG_RENDER
    std::string sphereTemplate = R"(
      {
        "Sphere": {
          "x": %x,
          "y": %y,
          "z": %z,
          "radius": 2,
          "r": 0.0,
          "g": 1.0,
          "b": 1.0,
          "a": 0.25
        }
      },
)";

    std::string s = R"(
    [
      %spheres
      {
        "Text": "p. tick: %pt, act. tick: %at, last sim time: %lst, avg. sim time: %ast, avg %apst per tick"
      },
      {
        "Text": "Sphere x: %x, y: %y, z: %z,    p. velocity: %prv"
      },
      {
        "Text": "act. ball pos: %acp,    act. ball v: %acv"
      },
      {
        "Text": "act. team#0 pos: %a0p,    act. team#0 v: %a0v"
      },
      {
        "Text": "act. team#1 pos: %a1p,    act. team#1 v: %a1v"
      },
      {
        "Text": "act. enemy#0 pos: %a2p,    act. enemy#0 v: %a2v"
      },
      {
        "Text": "act. enemy#1 pos: %a3p,    act. enemy#1 v: %a3v"
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

    auto predictedPos = m_state->predictedBallPos(ghostTick);

    std::string spheres;
    spheres.reserve(1024);
    for(const State::PredictedPos& pos : m_state->ballPredictions())
    {
        std::string next = sphereTemplate;

        format(next, "%x", pos.m_pos.x);
        format(next, "%y", pos.m_pos.y);
        format(next, "%z", pos.m_pos.z);

        spheres += next;
    }

    using EntityBall = Entity<Ball>;
    EntityBall simBall = m_state->game().ball;
    if (predictedPos.has_value())
    {
        simBall.setPosition(predictedPos->m_pos);
        simBall.setVelocity(predictedPos->m_velocity);
    }

    format(s, "%spheres", spheres);

    format(s, "%pt", ghostTick);
    format(s, "%at", game.current_tick);
    format(s, "%lst", lastSimMs);
    format(s, "%ast", g_simCount == 0 ? 0 : (g_simDurationMs / g_simCount));
    format(s, "%apst", g_simTicks == 0 ? 0 : (g_simDurationMs / g_simTicks));

    format(s, "%prv", simBall.velocity());
    format(s, "%acv", Entity<model::Ball>(game.ball).velocity());
    format(s, "%acp", Entity<model::Ball>(game.ball).position());

    std::vector<Robot> teammates;
    std::vector<Robot> enemies;
    for(const Robot& r : game.robots)
    {
        if(r.is_teammate)
            teammates.push_back(r);
        else
            enemies.push_back(r);
    }

    std::sort(teammates.begin(), teammates.end(), [](const Robot& a, const Robot& b) {return a.id < b.id; });
    std::sort(enemies.begin(), enemies.end(), [](const Robot& a, const Robot& b) {return a.id < b.id; });

    format(s, "%a0v", Entity<model::Robot>(teammates[0]).velocity());
    format(s, "%a0p", Entity<model::Robot>(teammates[0]).position());
    format(s, "%a1v", Entity<model::Robot>(teammates[1]).velocity());
    format(s, "%a1p", Entity<model::Robot>(teammates[1]).position());

    format(s, "%a2v", Entity<model::Robot>(enemies[0]).velocity());
    format(s, "%a2p", Entity<model::Robot>(enemies[0]).position());
    format(s, "%a3v", Entity<model::Robot>(enemies[1]).velocity());
    format(s, "%a3p", Entity<model::Robot>(enemies[1]).position());

    m_renderHint = std::move(s);
#else    // #ifdef DEBUG_RENDER
    m_renderHint.clear();
#endif   // #ifdef DEBUG_RENDER
}


void MyStrategy::initJumpPredictions()
{
    const Rules& rules = m_state->rules();

    using EntityRobot = Entity<Robot>;
    using EntityBall = Entity<Ball>;
    std::vector<EntityRobot> simRobots;
    EntityBall simBall = m_state->game().ball;
    simBall.setPosition({ 4, 4, 4 });      // don't collide with ball in this simulation

    EntityRobot ghost = m_state->me();
    ghost.setPosition({});
    ghost.setVelocity({});
    assert(ghost.touch);

    Action jumpAction;
    jumpAction.jump_speed = rules.ROBOT_MAX_JUMP_SPEED;
    ghost.setAction(jumpAction);

    simRobots.emplace_back(ghost);

    constexpr int SIM_TICKS = 50;
    const double tickTime = 1.0 / rules.TICKS_PER_SECOND;
    const double microtickTime = tickTime / rules.MICROTICKS_PER_TICK;

    for(int tick = 0; tick != SIM_TICKS; ++tick)
    {
        for(int microtick = 0; microtick < rules.MICROTICKS_PER_TICK; ++microtick)
            m_simulator->update(simRobots, simBall, microtickTime);

        m_state->saveJumpPrediction(tick, rules.ROBOT_MAX_JUMP_SPEED, simRobots.front().position(), simRobots.front().velocity());    // #todo - prepare table with multiple jump_speeds

        for(EntityRobot& r : simRobots)
            r.setAction({});
    }
}

