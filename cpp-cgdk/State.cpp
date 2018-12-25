#include "State.h"
#include <algorithm>
#include "noReleaseAssert.h"


State::State()
{
}


State::~State()
{
}

void State::saveBallPos(int tick, Vec3d&& pos)
{
    assert((m_ballPrediction.empty() || tick > m_ballPrediction.back().m_tick) && "ticks should be sorted for this call");
    m_ballPrediction.emplace_back(tick, std::forward<Vec3d>(pos));
}

std::optional<State::Vec3d> State::predictedBallPos(int tick) const
{
    PredictedPos key = { tick, Vec3d{} };

    auto found = std::lower_bound(m_ballPrediction.begin(), m_ballPrediction.end(), key, 
                                    [](const PredictedPos& a, const PredictedPos& b) { return a.m_tick < b.m_tick; });

    if(found != m_ballPrediction.end() && found->m_tick == tick)
        return found->m_pos;

    return std::nullopt;
}
