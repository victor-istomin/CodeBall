#pragma once
#include "model/Robot.h"
#include "model/Rules.h"
#include "model/Game.h"
#include "model/Action.h"


class State
{
    const model::Robot* m_me     = nullptr;
    const model::Rules* m_rules  = nullptr;
    const model::Game*  m_game   = nullptr;
    model::Action*      m_action = nullptr;

    bool m_isMoveCommitted = false;

public:
    State();
    ~State();

    const model::Robot&  me() const     { return *m_me; }
    const model::Rules&  rules() const  { return *m_rules; }
    const model::Game&   game() const   { return *m_game; }
    const model::Action& action() const { return *m_action; }

    void updateState(const model::Robot& me, const model::Rules& rules, const model::Game& game, model::Action& action)
    {
        m_me     = &me;
        m_rules  = &rules;
        m_game   = &game;
        m_action = &action;
        m_isMoveCommitted = false;
    }

    bool isMoveCommitted() const { return m_isMoveCommitted; }
};

