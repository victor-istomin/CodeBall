#include "Tests.h"
#include "MyStrategy.h"
#include "Simulator.h"

#include <fstream>
#include <iostream>
#include <chrono>

void log_captions(std::ostream& out)
{
    out << "tick,id,{x;y;z},{velocity_x;velocity_y;velocity_z}" << std::endl;

}

template <typename EntityType>
void log(const EntityType& e, int tick, std::ostream& out)
{
    out << tick << "," << e.getId() << ","
        << "{" << e.x << ";" << e.y << ";" << e.z << "},"
        << "{" << e.velocity_x << ";" << e.velocity_y << ";" << e.velocity_z << "}" << std::endl;
}

using namespace model;

void Test_Simulator(const Robot &me, const Game &game, Action &action, const Rules &rules, const std::unique_ptr<Simulator>& m_simulator)
{
    static std::ofstream g_actual = std::ofstream("..\\log_actual.txt", std::ios_base::trunc | std::ios_base::out);

    if(me.id != 1)
    {
        action.target_velocity_z = -rules.ROBOT_MAX_GROUND_SPEED;
        return;
    }

    action.target_velocity_x = rules.ROBOT_ACCELERATION;

    Entity<Robot> robot = me;
    Entity<Ball> ball = game.ball;

    if(game.current_tick == 0)
        log_captions(g_actual);
    log(robot, game.current_tick, g_actual);
    log(ball, game.current_tick, g_actual);

    robot.setAction(action);

    const double tickTime = 1.0 / rules.TICKS_PER_SECOND;
    const double microtickTime = tickTime / rules.MICROTICKS_PER_TICK;

    if(game.current_tick == 0)
    {
        std::ofstream out = std::ofstream("..\\log_1st.txt", std::ios_base::trunc | std::ios_base::out);
        log_captions(out);

        std::vector<Entity<Robot>> robots;
        robots.emplace_back(robot);

        for(int tick = 1; tick < 10; ++tick)
        {
            for(int i = 0; i < rules.MICROTICKS_PER_TICK; ++i)
                m_simulator->update(robots, ball, microtickTime);

            log(robots.front(), tick, out);
        }
    }

    if(game.current_tick == 10)
    {
        action.jump_speed = rules.ROBOT_MAX_JUMP_SPEED;
        action.target_velocity_x = me.velocity_x;
        robot.setAction(action);

        std::vector<Entity<Robot>> robots;
        robots.emplace_back(robot);

        std::ofstream out = std::ofstream("..\\log_jump.txt", std::ios_base::trunc | std::ios_base::out);
        log_captions(out);

        auto start = std::chrono::high_resolution_clock::now();
        for(int tick = 10; tick < 300; ++tick)
        {
            for(int i = 0; i < rules.MICROTICKS_PER_TICK; ++i)
                m_simulator->update(robots, ball, microtickTime);

            robots.front().action().jump_speed = 0;    // reset jump flag

            log(robots.front(), tick + 1, out);
            log(ball, tick + 1, out);
        }

        auto finish = std::chrono::high_resolution_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(finish - start).count();

        std::cout << ms << "ms, " << ms / 300 << " per unit per tick" << std::endl;
    }

    if(game.current_tick == 300)
        exit(1);
}

void Simulator::Test_Collide()
{
    Entity<model::Robot> r = model::Robot{
        0, // int    id;
        0, // int    player_id;
        true, // bool   is_teammate;
        0.1, // double x;
        0.1, // double y;
        0.1, // double z;
        1, // double velocity_x;
        1, // double velocity_y;
        1, // double velocity_z;
        1, // double radius;
        0, // double nitro_amount;
        true, // bool   touch;
        0, // double touch_normal_x;
        1, // double touch_normal_y;
        0, // double touch_normal_z;
    };

    Entity<model::Ball> b = model::Ball{
        .5,  // double x;
        2,   // double y;
        2,   // double z;
        .1,  // double velocity_x;
        .1,  // double velocity_y;
        .1,  // double velocity_z;
        2,   // double radius;
    };

    auto sumMoment = getMass(r) * r.velocity() + getMass(b) * b.velocity();
    auto distance = linalg::length(r.position() - b.position());

    assert(distance < (r.radius + b.radius));
    collide_entities(r, b);

    auto sumMomentAfter = getMass(r) * r.velocity() + getMass(b) * b.velocity();
    auto distanceAfter = linalg::length(r.position() - b.position());
    assert(sumMoment == sumMomentAfter);
    assert(k_Epsilon > std::abs(distanceAfter - (r.radius + b.radius)));
}

