#pragma once
#include "model/Action.h"
#include "model/Robot.h"
#include "model/Ball.h"
#include "model/Rules.h"
#include "model/Player.h"
#include "model/Game.h"
#include <memory>

class Simulator;

void Test_Simulator(const model::Robot &me, const model::Game &game, model::Action &action, const model::Rules &rules, const std::unique_ptr<Simulator>& m_simulator);