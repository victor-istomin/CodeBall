#pragma once
#include "model/Robot.h"
#include "model/Rules.h"
#include "model/Game.h"
#include "model/Action.h"

#undef min
#undef max
#include "linalg.h"

#include <vector>
#include <optional>

class State
{
public:
    using Rational = double;
    using Vec3d = linalg::vec<Rational, 3>;

    struct PredictedPos
    {
        int   m_tick = -1;
        Vec3d m_pos;

        PredictedPos() = default;
        PredictedPos(int tick, Vec3d&& pos) : m_tick(tick), m_pos(pos) {}
    };

private:

    const model::Robot* m_me     = nullptr;
    const model::Rules* m_rules  = nullptr;
    const model::Game*  m_game   = nullptr;
    model::Action*      m_action = nullptr;

    bool m_isMoveCommitted = false;

    std::vector<PredictedPos> m_ballPrediction;

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

        m_ballPrediction.resize(0);    // resize will not reduce capacity
    }

    void saveBallPos(int tick, Vec3d&& pos);
    std::optional<Vec3d> predictedBallPos(int tick) const;

    void commitAction(const model::Action& a) 
    { 
        *m_action = a; 
        m_isMoveCommitted = true;
    }

    bool isMoveCommitted() const { return m_isMoveCommitted; }
};

