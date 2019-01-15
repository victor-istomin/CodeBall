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

#include <emmintrin.h>
#include <smmintrin.h>

struct SimdVec
{
    __m128d xy;
    __m128d zw;

    SimdVec() = default;
    SimdVec(__m128d xy_, __m128d zw_) : xy(xy_), zw(zw_) {}

    SimdVec(const linalg::vec<double, 3>& v)
        : xy(_mm_loadu_pd(&v[0]))
        , zw(_mm_set_sd(v.z))
    {
    }

    linalg::vec<double, 3> toVec3d() const
    {
        linalg::vec<double, 3> result;
        _mm_storeu_pd(&result[0], xy);
        result.z = _mm_cvtsd_f64(zw);
        return result;
    }
};

struct DanTmp
{
    __m128d distance;
    SimdVec draftNormal;
};



class Simulator
{
public:
    using Rational = double;
    constexpr static Rational k_Epsilon = 1e-6;

    using Vec3d = linalg::vec<Rational, 3>;
    using Vec2d = linalg::vec<Rational, 2>;

    struct CollisionFlags
    {
        bool ball : 1;
        bool robots : 1;

        CollisionFlags() : ball(false), robots(false) {}

        void apply(CollisionFlags&& other)            { ball = ball || other.ball; robots = robots || other.robots; }

        operator bool() const                         { return ball && robots; }
    };

private:
    const model::Rules m_rules;
    std::mt19937       m_rng;

    template <typename Robot> auto getMass(const Robot&)   -> std::enable_if_t<std::is_base_of_v<model::Robot, Robot>, Rational> { return m_rules.ROBOT_MASS; }
    template <typename Robot> auto getMass(const Robot&)   -> std::enable_if_t<std::is_base_of_v<model::Ball,  Robot>, Rational> { return m_rules.BALL_MASS; }
    template <typename Robot> auto getArenaE(const Robot&) -> std::enable_if_t<!std::is_base_of_v<model::Ball, Robot>, Rational> { return 0; }
    template <typename Robot> auto getArenaE(const Robot&) -> std::enable_if_t< std::is_base_of_v<model::Ball, Robot>, Rational> { return m_rules.BALL_ARENA_E; }

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

    static DanTmp dan_to_plane2(const SimdVec& point, const Plane& plane);
    Simulator::Dan dan_to_wall(const SimdVec& point);

    static Dan dan_to_sphere_inner(const Vec3d& point, const Sphere& sphere);
    static Dan dan_to_sphere_outer(const Vec3d& point, const Sphere& sphere);

    Dan dan_to_arena_quarter(const Vec3d& point);

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
    bool collide_entities(LeftEntity& a, RightEntity& b)
    {
        Vec3d    delta_pos   = b.position() - a.position();
        Rational distance    = linalg::length(delta_pos);
        Rational penetration = a.radius + b.radius - distance;
        bool     isCollided  = false;

        if(penetration > 0)
        {
            Rational k_a = (1 / getMass(a)) / ((1 / getMass(a)) + (1 / getMass(b)));
            Rational k_b = (1 / getMass(b)) / ((1 / getMass(a)) + (1 / getMass(b)));
            Vec3d    normal = normalize(delta_pos);

            Vec3d d_a = normal * penetration * k_a;
            Vec3d d_b = normal * penetration * k_b;

            a.setPosition(a.position() - d_a);
            b.setPosition(b.position() + d_b);

            Rational delta_v = linalg::dot(b.velocity() - a.velocity(), normal) - b.radiusChangeSpeed() - a.radiusChangeSpeed();

            if(delta_v < 0)
            {
                Vec3d impulse = (1 + random(m_rules.MIN_HIT_E, m_rules.MAX_HIT_E)) * delta_v * normal;
                a.setVelocity(a.velocity() + impulse * k_a);
                b.setVelocity(b.velocity() - impulse * k_b);
            }

            isCollided = true;
        }

        return isCollided;
    }

    std::vector<Entity<model::Robot>*> m_robotPointersBuffer;   // singe-threaded environment only, this avoid a lot of new's when calling update for many microtics


public:

    Simulator(const model::Rules& rules) 
        : m_rules(rules)
        , m_rng(static_cast<int>(rules.seed))
    {
        m_robotPointersBuffer.reserve(8);
    }

    CollisionFlags update(std::vector<Entity<model::Robot>>& robots, Entity<model::Ball>& ball, Rational delta_time);
    Dan dan_to_arena(Vec3d point);


    Rational random(double min, double max) { return (min + max) / 2; }  // #todo - random

    void Test_Collide();   // a kind of unit test
};

