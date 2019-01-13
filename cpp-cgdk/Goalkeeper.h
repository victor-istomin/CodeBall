#pragma once
#include "goal.h"
#include "goalUtils.h"
#include <unordered_map>

namespace goals
{

class Goalkeeper : public Goal
{
    bool isGoalkeeper() const;
    bool canMove() const;
    bool isDefendPhase();
    bool isGoalDone() const;

    StepStatus assignGoalkeeperId();
    StepStatus findDefendPos();
    StepStatus reachDefendPos();

    StepStatus repeat();

private:
    static constexpr const int NONE = -1;
    constexpr static const int NO_MEETING = std::numeric_limits<int>::max();

    struct DefenceInfo
    {
        Vec3d ballPos      = {};
        int   meetingTick  = NO_MEETING;
        int   ticksToReach = std::numeric_limits<int>::max();
    };

    using Id = int;
    using DefendMap = std::unordered_map<Id, DefenceInfo>;

    Id        m_keeperId       = NONE;
    int       m_lastDefendTick = NONE;
    DefendMap m_defendMap      = {};

    virtual bool isCompatibleWith(const Goal* interrupted) override;

    int ticksFromLastDefence() const;
    Vec3d getDefendPos(Id id) const;

public:
    Goalkeeper(State& state, GoalManager& goalManager);
    ~Goalkeeper();
};


}

