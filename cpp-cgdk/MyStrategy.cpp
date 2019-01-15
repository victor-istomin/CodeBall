#include "MyStrategy.h"
#include "QuickStart_MyStrategy.h"
#include "Simulator.h"
#include "goalManager.h"
#include "DebugRender.h"

#include <chrono>
#include <sstream>
#include "Tests.h"
#include "stringUtils.h"

using namespace model;

static double g_simDurationMs = 0;
static int    g_simCount = 0;
static int    g_simTicks = 0;

MyStrategy::MyStrategy() 
{
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

    // #todo - other deferred actions?
    action.use_nitro = false;

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
    DebugRender& render = DebugRender::instance();

    for(const State::PredictedPos& pos : m_state->ballPredictions())
        render.shpere(pos.m_pos, m_state->rules().BALL_RADIUS, DebugRender::RGB_BALL_PREDICTED);

    render.text(FormattedString(R"(p. tick: %pt, act. tick: %at, last sim time: %lst, avg. sim time: %ast, avg %apst per tick)")
        .format("%pt",   ghostTick)
        .format("%at",   game.current_tick)
        .format("%lst",  lastSimMs)
        .format("%ast",  g_simCount == 0 ? 0 : (g_simDurationMs / g_simCount))
        .format("%apst", g_simTicks == 0 ? 0 : (g_simDurationMs / g_simTicks))
        .move());

    render.text(FormattedString(R"(act. ball pos: %acp,    act. ball v: %acv)")
        .format("%acv", Entity<model::Ball>(game.ball).velocity())
        .format("%acp", Entity<model::Ball>(game.ball).position())
        .move());



    const auto& teammates = m_state->teammates();
    const auto& enemies   = m_state->enemies();

    render.text(FormattedString(R"(act. team #0 pos:%t0p, v:%t0v, n:%t0n, act. team #1 pos: %t1p, v: %t1v, n:%t1n)")
        .format("%t0v", teammates[0].velocity())
        .format("%t0p", teammates[0].position())
        .format("%t0n", teammates[0].nitro_amount)
        .format("%t1v", teammates[1].velocity())
        .format("%t1p", teammates[1].position())
        .format("%t1n", teammates[1].nitro_amount)
        .move());

    render.text(FormattedString(R"(act. enemy #0 pos: %e0p, v: %e0v, act. enemy #1 pos: %e1p, v: %e1v)")
        .format("%e0v", enemies[0].velocity())
        .format("%e0p", enemies[0].position())
        .format("%e1v", enemies[1].velocity())
        .format("%e1p", enemies[1].position())
        .move());

#else    // #ifdef DEBUG_RENDER
#endif   // #ifdef DEBUG_RENDER
}


std::string MyStrategy::custom_rendering()
{
#ifdef DEBUG_RENDER
    if(DebugRender::instance().isEnabled())
        return DebugRender::instance().commit();
#endif
    return {};
}

void MyStrategy::initJumpPredictions()
{
    const Rules& rules = m_state->rules();

    using EntityRobot = Entity<Robot>;
    using EntityBall = Entity<Ball>;

    constexpr int SIM_TICKS = 50;
    const double tickTime = 1.0 / rules.TICKS_PER_SECOND;
    const double microtickTime = tickTime / rules.MICROTICKS_PER_TICK;

    for(double jumpSpeed = rules.ROBOT_MAX_JUMP_SPEED; jumpSpeed > 1; jumpSpeed -= 0.2)
    {
        std::vector<EntityRobot> simRobots;
        EntityBall simBall = m_state->game().ball;
        simBall.setPosition({ 4, 4, 4 });      // don't collide with ball in this simulation

        EntityRobot ghost = m_state->me();
        ghost.setPosition({});
        ghost.setVelocity({});
        assert(ghost.touch);

        Action jumpAction;
        jumpAction.jump_speed = jumpSpeed;
        ghost.setAction(jumpAction);

        simRobots.emplace_back(ghost);

        for(int tick = 0; tick != SIM_TICKS; ++tick)
        {
            for(int microtick = 0; microtick < rules.MICROTICKS_PER_TICK; ++microtick)
                m_simulator->update(simRobots, simBall, microtickTime);

            m_state->saveJumpPrediction(tick, jumpSpeed, simRobots.front().position(), simRobots.front().velocity());    // #todo - prepare table with multiple jump_speeds

            for(EntityRobot& r : simRobots)
                r.setAction({});
        }

        // speedup decreasing near interval center
        const double halfSpeed = rules.ROBOT_MAX_JUMP_SPEED / 2;
        double gap = (halfSpeed - std::abs(jumpSpeed - halfSpeed)) / (halfSpeed);
        jumpSpeed -= gap;
    }

}

