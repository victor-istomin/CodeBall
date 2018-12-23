#pragma once
#undef min
#undef max
#include "Entity.h"
#include "model/Rules.h"

#include <algorithm>
#include <optional>
#include <random>

// #todo - move somewhere
template <typename Rational, int Dim>
linalg::vec<Rational, Dim> clamp(linalg::vec<Rational, Dim> vec, Rational max)
{
    auto length = linalg::length(vec);
    if(length != 0)
    {
        Rational k = max / length;
        if(k < 1)
            vec *= k;
    }

    return vec;
};


class Simulator
{
public:
    using Rational = double;
    constexpr static Rational Epsilon = 1e-6;

    using Vec3d = linalg::vec<Rational, 3>;
    using Vec2d = linalg::vec<Rational, 2>;

private:
    const model::Rules m_rules;
    std::mt19937       m_rng;

    template <typename Unit> auto getMass(const Unit&)   -> std::enable_if_t<std::is_base_of_v<model::Robot, Unit>, Rational> { return m_rules.ROBOT_MASS; }
    template <typename Unit> auto getMass(const Unit&)   -> std::enable_if_t<std::is_base_of_v<model::Ball,  Unit>, Rational> { return m_rules.BALL_MASS; }
    template <typename Unit> auto getArenaE(const Unit&) -> std::enable_if_t<!std::is_base_of_v<model::Ball, Unit>, Rational> { return 0; }
    template <typename Unit> auto getArenaE(const Unit&) -> std::enable_if_t< std::is_base_of_v<model::Ball, Unit>, Rational> { return m_rules.BALL_ARENA_E; }

    struct Dan     // just following SDK naming, Distance And Normal
    {
        Rational distance = std::numeric_limits<Rational>::max();
        Vec3d    normal;

        bool operator<(const Dan& right) const { return distance < right.distance; }
    };

    struct Plane
    {
        Vec3d point;
        Vec3d normal;
    };

    struct Sphere
    {
        Vec3d    center;
        Rational radius;
    };

    static Dan dan_to_plane(const Vec3d& point, const Plane& plane);
    static Dan dan_to_sphere_inner(const Vec3d& point, const Sphere& sphere);
    static Dan dan_to_sphere_outer(const Vec3d& point, const Sphere& sphere);

    Dan dan_to_arena_quarter(const Vec3d& point);
    Dan dan_to_arena(Vec3d point);

    // returns normal
    template <typename EntityType>
    std::optional<Vec3d> collide_with_arena(EntityType& e)
    {
        Dan dan = dan_to_arena(e.position());
        Rational penetration = e.radius - dan.distance;
        if(penetration > 0)
        {
            e.setPosition(e.position() + penetration * dan.normal);
            Rational velocity = linalg::dot(e.velocity(), dan.normal) - e.radiusChangeSpeed();
            if(velocity < 0)
            {
                e.setVelocity(e.velocity() - ((1 + getArenaE(e)) * velocity * dan.normal));
                return dan.normal;
            }
        }

        return std::nullopt;
    }

    template <typename EntityType>
    void move(EntityType& e, Rational delta_time)
    {
        e.setVelocity(clamp(e.velocity(), m_rules.MAX_ENTITY_SPEED));
        e.setPosition(e.position() + e.velocity() * delta_time);

        // separate step for gravity
        e.y          -= m_rules.GRAVITY * delta_time * delta_time / 2;
        e.velocity_y -= m_rules.GRAVITY * delta_time;
    }

    template <typename LeftEntity, typename RightEntity>
    void collide_entities(LeftEntity& a, RightEntity& b)
    {
        Vec3d    delta_pos = b.position() - a.position();
        Rational distance = linalg::length(delta_pos);
        Rational penetration = a.radius + b.radius - distance;

        if(penetration > 0)
        {
            Rational k_a = (1 / getMass(a)) / ((1 / getMass(a)) + (1 / getMass(b)));
            Rational k_b = (1 / getMass(b)) / ((1 / getMass(a)) + (1 / getMass(b)));
            Vec3d    normal = normalize(delta_pos);

            Vec3d d_a = normal * penetration * k_a;
            Vec3d d_b = normal * penetration * k_b;

            a.setPosition(a.position() - d_a);
            b.setPosition(b.position() + d_b);

            Rational delta_v = linalg::dot(b.velocity() - a.velocity(), normal) + b.radiusChangeSpeed() - a.radiusChangeSpeed();

            if(delta_v < 0)   // #todo - why < 0?
            {
                Vec3d impulse = (1 + random(m_rules.MIN_HIT_E, m_rules.MAX_HIT_E)) * delta_v * normal;
                a.setVelocity(a.velocity() + impulse * k_a);
                b.setVelocity(b.velocity() - impulse * k_b);
            }
        }
    }

public:

    Simulator(const model::Rules& rules) 
        : m_rules(rules)
        , m_rng(static_cast<int>(rules.seed))
    {

    }

    void update(std::vector<Entity<model::Robot>>& robots, Entity<model::Ball>& ball, Rational delta_time)
    {
        // #todo - avoid 'new'
        std::vector<Entity<model::Robot>*> robotPointers;
        robotPointers.reserve(robots.size());

        std::transform(robots.begin(), robots.end(), std::back_inserter(robotPointers), [](Entity<model::Robot>& r) { return &r; });
        std::shuffle(robotPointers.begin(), robotPointers.end(), m_rng);

        for(Entity<model::Robot>& robot : robots)
        {
            if(robot.touch)   // #todo - check whether it's set in Sim
            {
                Vec3d target_velocity = clamp(robot.actionTargetVelocity(), m_rules.ROBOT_MAX_GROUND_SPEED);
                Rational ground_projection = linalg::dot(robot.touchNormal(), target_velocity);
                target_velocity = target_velocity - robot.touchNormal() * ground_projection;
                Vec3d target_velocity_change = target_velocity - robot.velocity();

                if(linalg::length(target_velocity_change) > 0)
                {
                    Rational acceleration = m_rules.ROBOT_ACCELERATION * std::max(0.0, robot.touch_normal_y);
                    robot.setVelocity(robot.velocity()
                        + clamp(
                            linalg::normalize(target_velocity_change) * acceleration * delta_time,
                            length(target_velocity_change)));
                }
            }

            if(robot.action().use_nitro)
            {
                Vec3d target_velocity_change = clamp(robot.actionTargetVelocity() - robot.velocity(),
                    robot.nitro_amount * m_rules.NITRO_POINT_VELOCITY_CHANGE);

                if(linalg::length(target_velocity_change) > 0)
                {
                    Vec3d acceleration = linalg::normalize(target_velocity_change) * m_rules.ROBOT_NITRO_ACCELERATION;
                    Vec3d velocity_change = clamp(acceleration * delta_time, linalg::length(target_velocity_change));

                    robot.setVelocity(robot.velocity() + velocity_change);
                    robot.nitro_amount -= linalg::length(velocity_change) / m_rules.NITRO_POINT_VELOCITY_CHANGE;
                }
            }

            move(robot, delta_time);
            robot.radius = m_rules.ROBOT_MIN_RADIUS + (m_rules.ROBOT_MAX_RADIUS - m_rules.ROBOT_MIN_RADIUS) * robot.action().jump_speed / m_rules.ROBOT_MAX_JUMP_SPEED;
            robot.setRadiusChangeSpeed(robot.action().jump_speed);
        }

        move(ball, delta_time);

        for(int i = 0; i < (int)robots.size(); ++i)
            for(int j = 0; j < i - 1; ++j)
                collide_entities(robots[i], robots[j]);

        for(Entity<model::Robot>& robot : robots)
        {
            collide_entities(robot, ball);

            std::optional<Vec3d> collisionNormal = collide_with_arena(robot);
            if(collisionNormal.has_value())
            {
                robot.touch = true;
                robot.setTouchNormal(*collisionNormal);
            }
            else
            {
                robot.touch = false;
            }
        }

        collide_with_arena(ball);

        // #todo - goal callback
        // if (abs(ball.position.z) > arena.depth / 2 + ball.radius)
        //     goal_scored();

        // #todo - nitro packs
    }


    Rational random(double min, double max) { return (min + max) / 2; }  // #todo - random

    void Test_Collide();   // a kind of unit test
};

