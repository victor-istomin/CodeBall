#include "MyStrategy.h"

#define LOG
#ifdef LOG
#include <fstream>
#include <iostream>
#include <chrono>

static std::ofstream g_actual = std::ofstream("..\\log_actual.txt", std::ios_base::trunc | std::ios_base::out);
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
#endif

using namespace model;

MyStrategy::MyStrategy() 
{
}

void MyStrategy::act(const Robot& me, const Rules& rules, const Game& game, Action& action)
{
    if(!m_simulator)
        m_simulator = std::make_unique<Simulator>(rules, 1);

	if(me.id != 1)
	{
		//action.target_velocity_z = -ROBOT_MAX_GROUND_SPEED;
		return;
	}

	//action.target_velocity_x = ROBOT_ACCELERATION;

	Entity<Robot> robot = me;
	Entity<Ball> ball = game.ball;

	if(game.current_tick == 0)
		log_captions(g_actual);
	log(robot, game.current_tick, g_actual);
	log(ball, game.current_tick, g_actual);
	
	robot.setAction(action);

	constexpr double tickTime      = 1 / TICKS_PER_SECOND;
	constexpr double microtickTime = tickTime / MICROTICKS_PER_TICK;

	if(game.current_tick == 0)
	{
		std::ofstream out = std::ofstream("..\\log_1st.txt", std::ios_base::trunc | std::ios_base::out);
		log_captions(out);

// 		std::vector<Entity<Robot>> robots;
// 		robots.emplace_back(robot);
// 
// 		for(int tick = 1; tick < 10; ++tick)
// 		{
// 			for(int i = 0; i < MICROTICKS_PER_TICK; ++i)
// 				m_simulator->update(robots, ball, microtickTime);
// 
// 			log(robots.front(), tick, out);
// 		}
	}

	if(game.current_tick == 10)
	{ 
		action.jump_speed = ROBOT_MAX_JUMP_SPEED;
		action.target_velocity_x = me.velocity_x;
		robot.setAction(action);

		std::vector<Entity<Robot>> robots;
		robots.emplace_back(robot);

		std::ofstream out = std::ofstream("..\\log_jump.txt", std::ios_base::trunc | std::ios_base::out);
		log_captions(out);

		auto start = std::chrono::high_resolution_clock::now();
		for(int tick = 10; tick < 300; ++tick)
		{
			for(int i = 0; i < MICROTICKS_PER_TICK; ++i)
				m_simulator->update(robots, ball, microtickTime);
			
			robots.front().action().jump_speed = 0;    // reset jump flag

//			log(robots.front(), tick + 1, out);
//			log(ball, tick + 1, out);
		}

		auto finish = std::chrono::high_resolution_clock::now();
		auto ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(finish - start).count();

		std::cout << ms << "ms, " << ms / 300 << " per unit per tick" << std::endl;
	}

	if(game.current_tick == 300)
		exit(1);
}
