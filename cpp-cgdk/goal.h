#pragma once
#include <memory>
#include <functional>
#include <list>
#include <algorithm>

// forward declarations
class State;
class GoalManager;


class Goal
{
public:
    enum class StepStatus { Ok, Done, AbortGoal };

    enum class StepType
    {
        INVALID_VALUE = 0,
        ATOMIC,
        ALLOW_MULTITASK
    };

    typedef std::function<bool()>       CallbackBool;
    typedef std::function<StepStatus()> CallbackStatus;

    struct Step
    {

        const char* m_debugName;

        CallbackBool    m_shouldAbort;
        CallbackBool    m_shouldProceed;
        CallbackStatus  m_proceed;
        bool            m_isMultitaskPoint;

        Step(CallbackBool shouldAbort, CallbackBool shouldProceed, CallbackStatus proceed, const char* debugName = nullptr, StepType type = StepType::ATOMIC)
            : m_debugName(debugName)
            , m_shouldAbort(shouldAbort), m_shouldProceed(shouldProceed), m_proceed(proceed)
            , m_isMultitaskPoint(type == StepType::ALLOW_MULTITASK)
        {}
    };

private:
    typedef std::unique_ptr<Step> StepPtr;

    std::list<StepPtr> m_steps;
    GoalManager&       m_goalManager;
    State&             m_state;
    bool               m_isStarted;


    void abortGoal() { m_steps.clear(); }

    void doMultitasking(GoalManager &goalManager);
    bool isNoMoveComitted();

    // check if this goal could be performed in multitasking mode when 'interrupted' has nothing to do right now
    virtual bool isCompatibleWith(const Goal* interrupted) { return false; }

protected:

    template <typename... Args>
    void pushBackStep(Args&&... args)
    {
        m_steps.emplace_back(std::make_unique<Step>(std::forward<Args>(args)...));
    }

    template <typename... Args>
    void pushNextStep(Args&&... args)
    {
        if (m_steps.empty())
            pushBackStep(std::forward<Args>(args)...);
        else
            m_steps.emplace(++m_steps.begin(), std::make_unique<Step>(std::forward<Args>(args)...));
    }

    template <typename... Args>
    void pushFirstStep(Args&&... args)
    {
        if(m_steps.empty())
            pushBackStep(std::forward<Args>(args)...);
        else
            m_steps.emplace_front(std::make_unique<Step>(std::forward<Args>(args)...));
    }


    State& state()                                { return m_state; }
    const State& state() const                    { return m_state; }
    GoalManager& goalManager()                    { return m_goalManager; }
    const GoalManager& goalManager() const        { return m_goalManager; }

    bool isAboutToAbort() const                   { return m_steps.size() == 1 && m_steps.front()->m_shouldAbort(); }

public:

    Goal(State& state, GoalManager& goalManager) : m_goalManager(goalManager), m_state(state), m_isStarted(false) {}
    virtual ~Goal()                                         {}

    bool isFinished() const              { return m_steps.empty(); }
    bool isStarted() const               { return m_isStarted; }
    bool canPause() const                { return isFinished() || !isStarted() || m_steps.front()->m_isMultitaskPoint; }

    bool isEligibleForBackgroundMode(const Goal* interrupted) 
    { 
        // ensure it will return execution
        bool hasMultitaskPoint = std::find_if(m_steps.begin(), m_steps.end(), [](const StepPtr& step) { return step->m_isMultitaskPoint; }) != m_steps.end();

        return this != interrupted && hasMultitaskPoint && isCompatibleWith(interrupted); 
    }

    void performStep(GoalManager& goalManager, bool isBackgroundMode);
};

