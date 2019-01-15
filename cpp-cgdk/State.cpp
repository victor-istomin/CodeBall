#include "State.h"
#include <algorithm>
#include "noReleaseAssert.h"
#include "Entity.h"

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

    Score score = { game.players[0].score, game.players[1].score };
    if(!game.players[0].me)
        std::swap(score.first, score.second);

    bool isGoal = m_score != score;
    if(isGoal && game.current_tick > m_roundStartTick)
        m_roundStartTick = game.current_tick + rules.RESET_TICKS - 1;    // #todo : is it really -1?

    m_isNewRound = isGoal && isBallAtStartPos();
    if(isBallAtStartPos())
        m_score = score;

    m_teammates.clear();
    m_enemies.clear();
    for(const model::Robot& r : game.robots)
    {
        if(r.is_teammate)
            m_teammates.emplace_back(r);
        else
            m_enemies.emplace_back(r);
    }

    m_isMoveCommitted = false;
}

bool State::isBallAtStartPos() const
{
    return m_game->ball.x == 0 && m_game->ball.z == 0 && m_game->ball.velocity_x == 0 && m_game->ball.velocity_z == 0;
}

void State::saveBallPos(int tick, Vec3d&& pos, Vec3d&& velocity)
{
    assert((m_ballPrediction.empty() || tick > m_ballPrediction.back().m_tick) && "ticks should be sorted for this call");
    m_ballPrediction.emplace_back(tick, std::forward<Vec3d>(pos), std::forward<Vec3d>(velocity));

    const double theirGoalZ = m_rules->arena.depth / 2 + m_rules->BALL_RADIUS;
    const double mineGoalZ  = -theirGoalZ;

    if(pos.z >= theirGoalZ)
        m_goalPredictionTick.m_theirGates = m_goalPredictionTick.m_theirGates == INT_NONE ? tick : std::min(m_goalPredictionTick.m_theirGates, tick);
    if(pos.z <= mineGoalZ)
        m_goalPredictionTick.m_mineGates = m_goalPredictionTick.m_mineGates == INT_NONE ? tick : std::min(m_goalPredictionTick.m_mineGates, tick);
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
    m_goalPredictionTick = {};
}

void State::saveJumpPrediction(int tick, double initialSpeed, const Vec3d& pos, const Vec3d& velocity)
{
    m_jumpPredictions[initialSpeed].push_back(PredictedJumpHeight{ tick, initialSpeed, pos, velocity });
}
