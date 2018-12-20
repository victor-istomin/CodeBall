/*
 * Quick start strategy, copy-paste from SDK as is
*/

#include "QuickStart_MyStrategy.h"

using namespace model;

QuickStart_MyStrategy::QuickStart_MyStrategy() { }

void QuickStart_MyStrategy::act(const Robot& me, const Rules& rules, const Game& game, Action& action) {

    ball.set(game.ball.x, game.ball.z, game.ball.y);
    ball_v.set(game.ball.velocity_x, game.ball.velocity_z, game.ball.velocity_y);

    // Ќаша стратеги€ умеет играть только на земле
    // ѕоэтому, если мы не касаемс€ земли, будет использовать нитро
    // чтобы как можно быстрее попасть обратно на землю
    if(!me.touch) {
        action.target_velocity_x = 0.0;
        action.target_velocity_z = 0.0;
        action.target_velocity_y = -MAX_ENTITY_SPEED;
        action.jump_speed = 0.0;
        action.use_nitro = true;
        return;
    }

    // ≈сли при прыжке произойдет столкновение с м€чом, и мы находимс€
    // с той же стороны от м€ча, что и наши ворота, прыгнем, тем самым
    // ударив по м€чу сильнее в сторону противника
    bool jump = (ball.distTo(me.x, me.z, me.y) < (BALL_RADIUS + ROBOT_MAX_RADIUS)
        && me.y < ball.y);

    // “ак как роботов несколько, определим нашу роль - защитник, или нападающий
    // Ќападающим будем в том случае, если есть дружественный робот,
    // наход€щийс€ ближе к нашим воротам
    bool is_attacker = false; // = (game.robots.size() == 2);
    for(const Robot &robot : game.robots) {
        if(robot.is_teammate
            && robot.id != me.id) {
            if(robot.z < me.z) {
                is_attacker = true;
            }
        }
    }

    if(is_attacker) {
        // —тратеги€ нападающего:
        // ѕросимулирем примерное положение м€ча в следующие 10 секунд, с точностью 0.1 секунда
        for(int i = 0; i < 100; ++i) {
            double t = i * 0.1;
            Point3D ball_pos = ball + ball_v * t;
            // ≈сли м€ч не вылетит за пределы арены
            // (произойдет столкновение со стеной, которое мы не рассматриваем),
            // и при этом м€ч будет находитс€ ближе к вражеским воротам, чем робот,
            if(ball_pos.z > me.z
                && abs(ball.x) < (rules.arena.width / 2.0)
                && abs(ball.z) < (rules.arena.depth / 2.0)) {
                // ѕосчитаем, с какой скоростью робот должен бежать,
                // „тобы прийти туда же, где будет м€ч, в то же самое врем€
                Point2D delta_pos(ball_pos.x - me.x, ball_pos.z - me.z);
                double delta_pos_dist = delta_pos.dist();
                double need_speed = delta_pos_dist / t;
                // ≈сли эта скорость лежит в допустимом отрезке
                if(0.5 * ROBOT_MAX_GROUND_SPEED < need_speed
                    && need_speed < ROBOT_MAX_GROUND_SPEED) {
                    // “о это и будет наше текущее действие
                    Point2D target_velocity(delta_pos.normalize(delta_pos_dist)*need_speed);
                    action.target_velocity_x = target_velocity.x;
                    action.target_velocity_z = target_velocity.z;
                    action.target_velocity_y = 0.0;
                    action.jump_speed = jump ? ROBOT_MAX_JUMP_SPEED : 0.0;
                    action.use_nitro = false;
                    return;
                }
            }
        }
    }
    // —тратеги€ защитника (или атакующего, не нашедшего хорошего момента дл€ удара):
    // Ѕудем сто€ть посередине наших ворот
    Point2D target_pos(0.0, -(rules.arena.depth / 2.0) + rules.arena.bottom_radius);
    // ѕричем, если м€ч движетс€ в сторону наших ворот
    if(ball_v.z < -EPS) {
        // Ќайдем врем€ и место, в котором м€ч пересечет линию ворот
        double t = (target_pos.z - ball.z) / ball_v.z;
        double x = ball.x + ball_v.x * t;
        // ≈сли это место - внутри ворот
        if(abs(x) < (rules.arena.goal_width / 2.0)) {
            // “о пойдем защищать его
            target_pos.x = x;
        }
    }
    // ”становка нужных полей дл€ желаемого действи€
    Point2D target_velocity(target_pos.x - me.x, target_pos.z - me.z);
    target_velocity *= ROBOT_MAX_GROUND_SPEED;

    action.target_velocity_x = target_velocity.x;
    action.target_velocity_z = target_velocity.z;
    action.target_velocity_y = 0.0;
    action.jump_speed = jump ? ROBOT_MAX_JUMP_SPEED : 0.0;
    action.use_nitro = false;
}
