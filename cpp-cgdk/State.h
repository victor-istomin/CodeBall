#pragma once
#include "model/Robot.h"
#include "model/Rules.h"
#include "model/Game.h"
#include "model/Action.h"
#include "algebra.h"
#include "forwards.h"

#include <vector>
#include <optional>
#include <map>

struct PredictedJumpHeight
{
    double m_height = 0;
    double m_initialSpeed = 0;
    double m_timeToReach = 0;
    double m_velocity_y = 0;

    PredictedJumpHeight(int tick, double initialSpeed, const Vec3d& pos, const Vec3d& velocity)
        : m_height(pos.y)
        , m_initialSpeed(initialSpeed)
        , m_timeToReach(tick)
        , m_velocity_y(velocity.y)
    {
    }
};

class State
{
public:
    static constexpr int INT_NONE = -1;

    struct PredictedPos
    {
        int   m_tick = INT_NONE;
        Vec3d m_pos;
        Vec3d m_velocity;

        PredictedPos() = default;
        PredictedPos(int tick, Vec3d&& pos, Vec3d&& velocity) : m_tick(tick), m_pos(pos), m_velocity(velocity) {}
    };

    struct GoalPredictionTick
    {
        int m_mineGates  = INT_NONE;
        int m_theirGates = INT_NONE;
    };

    using JumpPredictionMap = std::map<double/*initial speed*/, std::vector<PredictedJumpHeight>> ;
    using EntityRobot       = Entity<model::Robot>;

private:
    using Score = std::pair<int/*mine*/, int/*their*/>;

    const model::Robot* m_me     = nullptr;
    const model::Rules* m_rules  = nullptr;
    const model::Game*  m_game   = nullptr;
    model::Action*      m_action = nullptr;
    Score               m_score  = { -1, -1 };

    bool m_isMoveCommitted = false;
    bool m_isNewRound      = false;
    int  m_roundStartTick  = 0;

    std::vector<PredictedPos> m_ballPrediction;
    JumpPredictionMap         m_jumpPredictions;
    GoalPredictionTick        m_goalPredictionTick;
    std::vector<EntityRobot>  m_teammates;
    std::vector<EntityRobot>  m_enemies;

public:
    State();
    ~State();

    const model::Robot&  me() const     { return *m_me; }
    const model::Rules&  rules() const  { return *m_rules; }
    const model::Game&   game() const   { return *m_game; }
    const model::Action& action() const { return *m_action; }

    void updateState(const model::Robot& me, const model::Rules& rules, const model::Game& game, model::Action& action);

    void saveBallPos(int tick, Vec3d&& pos, Vec3d&& velocity);
    std::optional<PredictedPos> predictedBallPos(int tick) const;
    const std::vector<PredictedPos>& ballPredictions() const        { return m_ballPrediction; }
    void invalidateBallPredictions();

    void saveJumpPrediction(int tick, double initialSpeed, const Vec3d& pos, const Vec3d& velocity);
    const JumpPredictionMap& jumpPredictions() const { return m_jumpPredictions; }

    const GoalPredictionTick& goalPredictionTick() const { return m_goalPredictionTick; }

    const std::vector<EntityRobot>& teammates() const { return m_teammates; }
    const std::vector<EntityRobot>& enemies() const { return m_enemies; }    

    void commitAction(const model::Action& a) 
    { 
        *m_action = a; 
        m_isMoveCommitted = true;
    }

    bool isBallAtStartPos() const;
    bool isMoveCommitted() const         { return m_isMoveCommitted; }
    bool isNewRound() const              { return m_isNewRound; }
    bool isChilloutPhase() const         { return roundLocalTick() < 0; }
    int  roundLocalTick() const          { return m_game->current_tick - m_roundStartTick; }
};

