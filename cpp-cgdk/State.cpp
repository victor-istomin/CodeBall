#include "State.h"
#include <algorithm>
#include "noReleaseAssert.h"


State::State()
{
}


State::~State()
{
}

void State::updateState(const model::Robot& me, const model::Rules& rules, const model::Game& game, model::Action& action)
{
    m_me     = &me;
    m_rules  = &rules;
    m_game   = &game;
    m_action = &action;

    bool isBallAtStartPos = game.ball.x == 0 && game.ball.z == 0 && game.ball.velocity_x == 0 && game.ball.velocity_z == 0;

    Score score = { game.players[0].score, game.players[1].score };
    if(!game.players[0].me)
        std::swap(score.first, score.second);
    m_isNewRound = m_score != score && isBallAtStartPos;

    if(isBallAtStartPos)
        m_score = score;

    m_isMoveCommitted = false;
}

void State::saveBallPos(int tick, Vec3d&& pos, Vec3d&& velocity)
{
    assert((m_ballPrediction.empty() || tick > m_ballPrediction.back().m_tick) && "ticks should be sorted for this call");
    m_ballPrediction.emplace_back(tick, std::forward<Vec3d>(pos), std::forward<Vec3d>(velocity));
}

std::optional<State::PredictedPos> State::predictedBallPos(int tick) const
{
    PredictedPos key = { tick, {}, {} };

    auto found = std::lower_bound(m_ballPrediction.begin(), m_ballPrediction.end(), key, 
                                    [](const PredictedPos& a, const PredictedPos& b) { return a.m_tick < b.m_tick; });

    if(found != m_ballPrediction.end() && found->m_tick == tick)
        return *found;

    return std::nullopt;
}

void State::invalidateBallPredictions()
{
    m_ballPrediction.resize(0);    // resize will not reduce capacity
}

void State::saveJumpPrediction(int tick, double initialSpeed, Vec3d&& pos, Vec3d&& velocity)
{
    m_jumpPredictions.push_back(PredictedJumpHeight{ tick, initialSpeed, std::forward<Vec3d>(pos), std::forward<Vec3d>(velocity) });
}