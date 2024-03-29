#include "goalManager.h"
#include "GoalTakeBallPair.h"
#include "GoalAttackSingle.h"
#include "Goalkeeper.h"
#include "State.h"

void GoalManager::fillCurrentGoals()
{
    // use sorted list because std::multimap looks like overengieneering

    if (m_state.isNewRound())
    {
        int priority = 0;

        m_forcedGoal = nullptr;
        m_currentGoals.clear();

        m_currentGoals.emplace_back(priority++, std::make_unique<goals::TakeBallPair>(m_state, *this));
        m_currentGoals.emplace_back(priority++, std::make_unique<goals::AttackSingle>(m_state, *this));
        m_currentGoals.emplace_back(priority++, std::make_unique<goals::Goalkeeper>(m_state, *this));


        m_currentGoals.sort();
    }

    if (!m_waitingInsetrion.empty())
    {
        // can't reorder goals if there is any goal which can't be paused right now
        auto isGoalBusy = [](const GoalHolder& holder) { return !holder.m_goal->canPause(); };
        bool isManagerBusy = std::find_if(m_currentGoals.begin(), m_currentGoals.end(), isGoalBusy) != m_currentGoals.end();

        if (!isManagerBusy)
        {
            m_currentGoals.splice(m_currentGoals.end(), m_waitingInsetrion);
        }

        m_currentGoals.sort();
    }

    assert(std::is_sorted(m_currentGoals.begin(), m_currentGoals.end()) && "please keep goals sorted by-priority");
}

GoalManager::GoalManager(State& state) 
    : m_state(state)
    , m_forcedGoal(nullptr)
{

}

void GoalManager::tick()
{
    fillCurrentGoals();

    if (m_forcedGoal)
    {
        m_forcedGoal->performStep(*this, true);

        if (m_forcedGoal->isFinished())
        {
            auto forcedIt = std::find_if(m_currentGoals.begin(), m_currentGoals.end(), 
                [this](const GoalHolder& holder) { return holder.m_goal.get() == m_forcedGoal; });
            assert(forcedIt != m_currentGoals.end());

            m_forcedGoal = nullptr;           // done, pause and remove
            m_currentGoals.erase(forcedIt);
        }

        if (m_forcedGoal && m_forcedGoal->canPause())
            m_forcedGoal = nullptr;           // just pause
    }

    if (!m_state.isMoveCommitted())
    {
        if (!m_currentGoals.empty())
        {
            const GoalPtr& mostPriority = m_currentGoals.front().m_goal;
            mostPriority->performStep(*this, false);
            if (mostPriority->isFinished())
            {
                if (m_forcedGoal == m_currentGoals.front().m_goal.get())
                    m_forcedGoal = nullptr;

                m_currentGoals.pop_front();
            }
        }
    }
}

void GoalManager::doMultitasking(const Goal* interruptedGoal)
{
    Goal* executedGoal = nullptr;

    for (const GoalHolder& goalHolder : m_currentGoals)
    {
        const GoalPtr& goal = goalHolder.m_goal;
        if (!goal->isEligibleForBackgroundMode(interruptedGoal))
            continue;

        goal->performStep(*this, true);
        if (m_state.isMoveCommitted())
        {
            executedGoal = goal.get();
            break;   // one tick - one move
        }
    }

    if (executedGoal && !executedGoal->isFinished() && !executedGoal->canPause())
        m_forcedGoal = executedGoal;

    // purge finished goals
    m_currentGoals.remove_if([](const GoalHolder& holder) { return holder.m_goal->isFinished(); });
}

