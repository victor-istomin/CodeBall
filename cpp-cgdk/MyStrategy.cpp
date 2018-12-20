#include "MyStrategy.h"
#include "QuickStart_MyStrategy.h"
#include <chrono>
#include <numeric>
#include <cmath>

#include <iostream>
#include <fstream>

#include "tests.h"

using namespace model;

template <typename R> R pow2(R x) { return x * x; }

MyStrategy::MyStrategy() 
{
}

auto logFile = std::ofstream("..\\log_attack.txt", std::ios_base::trunc | std::ios_base::out);

void MyStrategy::act(const Robot& me, const Rules& rules, const Game& game, Action& action)
{
    if(!m_simulator)
        m_simulator = std::make_unique<Simulator>(rules);

    if(!m_qsGuy)
        m_qsGuy = std::make_unique<QuickStart_MyStrategy>();

    if(0 != (me.id % 2))
        return;

//     m_qsGuy->act(me, rules, game, action);
//     return;
    
    using EntityRobot = Entity<Robot>;
    using EntityBall  = Entity<Ball>;
    std::vector<EntityRobot> enemies;
    std::vector<EntityRobot> teammates;
    EntityBall ball = game.ball;

    for(const Robot& r : game.robots)
    {
        if(r.is_teammate)
            teammates.emplace_back(r);
        else
            enemies.emplace_back(r);
    }

    double attackerDistance = std::numeric_limits<double>::max();
    const EntityRobot* enemyAttacker = nullptr;
    for(const EntityRobot& enemy : enemies)
    {
        double ballDistance = linalg::length(enemy.position() - ball.position());
        if(enemyAttacker == nullptr || ballDistance < attackerDistance)
        {
            attackerDistance = ballDistance;
            enemyAttacker = &enemy;
        }
    }

    // get ball speed after collision with attacker 
    constexpr int TICKS_TO_SIM = 100;
    const double tickTime = 1.0 / rules.TICKS_PER_SECOND;
    const double microtickTime = tickTime / rules.MICROTICKS_PER_TICK;

    std::vector<EntityRobot> simRobots;
    simRobots.emplace_back(*enemyAttacker);
    EntityRobot& simAttacker = simRobots.front();
    EntityBall   simBall     = ball;
    const double attackDistanceSquare = pow2(rules.ROBOT_MAX_RADIUS + rules.BALL_RADIUS);

    struct HitInfo
    {
        int tick = 0;
        EntityRobot attacker;
        EntityBall ball;
    };

    HitInfo hit;

    for(int i = 0; i < TICKS_TO_SIM; ++i)
    {
        Action nextAction;

//         using PointXZ = linalg::vec<double, 2>;
//         PointXZ ballXZ     = { simBall.x, simBall.z };
//         PointXZ attackerXZ = { simAttacker.x, simAttacker.z };
//         PointXZ displacementXZ = ballXZ - attackerXZ;
// 
//         nextAction.target_velocity_x = displacementXZ[0];
//         nextAction.target_velocity_y = 0;
//         nextAction.target_velocity_z = displacementXZ[1];
// 
//         double distanceSqare = linalg::length2(simAttacker.position() - simBall.position());
//         if(distanceSqare < attackDistanceSquare && simAttacker.y < simBall.y)
//             nextAction.jump_speed = rules.ROBOT_MAX_JUMP_SPEED;

        m_qsGuy->attackerAction(simAttacker, simBall, rules, nextAction);
        simAttacker.setAction(nextAction);

        for(int microtick = 0; microtick < rules.MICROTICKS_PER_TICK; ++microtick)
            m_simulator->update(simRobots, simBall, microtickTime);

        if(simBall.velocity_x > 0 || simBall.velocity_z > 0)
        {
            // attacker hit ball on this tick
            hit.tick = game.current_tick + i;
            hit.attacker = simAttacker;
            hit.ball     = simBall;
            break;
        }
    }

    auto outPos = [](std::ostream& out, const auto& entity)
    {
        out << " p: {" << entity.x << "," << entity.y << "," << entity.z << "}"
            << " v: {" << entity.velocity_x << "," << entity.velocity_y << "," << entity.velocity_z << "}";
    };

    if(hit.tick != 0)
    {
        logFile << "t: " << game.current_tick << "; enemy hit t: " << hit.tick
            << "; enemy: ";

        outPos(logFile, hit.attacker);
        logFile << "; ball: ";
        outPos(logFile, hit.ball);
        logFile << std::endl;
    }

    logFile << "t: " << game.current_tick << "; actual attacker: ";
    outPos(logFile, *enemyAttacker);
    logFile << "; actual ball: ";
    outPos(logFile, ball);
    logFile << std::endl;

    if(game.current_tick > 100)
        exit(1);

    //Test_Simulator(me, game, action, rules, m_simulator);
    return;
}
