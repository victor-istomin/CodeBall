#include "Simulator.h"

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

double simdDot(const SimdVec& a, const SimdVec& b)
{
    __m128d xy = _mm_dp_pd(a.xy, b.xy, 0xFF);
    __m128d zw = _mm_dp_pd(a.zw, b.zw, 0xFF);
    __m128d sum = _mm_add_pd(xy, zw);

    return _mm_cvtsd_f64(sum);
}

double simdLength(const SimdVec& v)
{
    __m128d xy = _mm_dp_pd(v.xy, v.xy, 0xFF);
    __m128d zw = _mm_dp_pd(v.zw, v.zw, 0xFF);
    __m128d root = _mm_sqrt_pd(_mm_add_pd(xy, zw));

    return _mm_cvtsd_f64(root);
}

SimdVec simdDiff(const SimdVec& a, const SimdVec& b)
{
    return SimdVec { _mm_sub_pd(a.xy, b.xy), _mm_sub_pd(a.zw, b.zw) };
}


Simulator::Dan Simulator::dan_to_plane(const Vec3d& point, const Plane& plane)
{
    Dan test{ simdDot(simdDiff(point, plane.point), plane.normal), plane.normal };
    return test;
}

Simulator::Dan Simulator::dan_to_sphere_inner(const Vec3d& point, const Sphere& sphere)
{
    double length = simdLength(simdDiff(point, sphere.center));
    Dan test1 { sphere.radius - length, simdDiff(sphere.center, point).toVec3d() / length };
    return test1;
}

Simulator::Dan Simulator::dan_to_sphere_outer(const Vec3d& point, const Sphere& sphere)
{
    return Dan{ linalg::length(point - sphere.center) - sphere.radius, linalg::normalize(point - sphere.center) };
}

Simulator::Dan Simulator::dan_to_arena_quarter(const Vec3d& point)
{
    const model::Arena& arena = m_rules.arena;

    // --- planes
    const Plane ground = { Vec3d{ 0, 0, 0 }, Vec3d{ 0, 1, 0 } };
    const Plane ceiling = { Vec3d{ 0, arena.height, 0 }, Vec3d{ 0, -1, 0 } };
    const Plane sideX = { Vec3d{arena.width / 2, 0, 0}, Vec3d{-1, 0 , 0} };
    const Plane goalZ = { Vec3d{0, 0, (arena.depth / 2) + arena.goal_depth}, Vec3d{0, 0, -1} };

    Dan dan = std::min({
            dan_to_plane(point, ground),
            dan_to_plane(point, ceiling),
            dan_to_plane(point, sideX),
            dan_to_plane(point, goalZ),
        });

    const Vec2d pointXY = point.xy();
    Vec2d arenaSqGate = { (arena.goal_width / 2) - arena.goal_top_radius, arena.goal_height - arena.goal_top_radius };

    // side Z (non-goal)
    Vec2d vz /*name in SDK*/ = pointXY - arenaSqGate;
    if((pointXY.x >= (arena.goal_width / 2) + arena.goal_side_radius)
        || (pointXY.y >= arena.goal_height + arena.goal_side_radius)
        || (vz.x > 0 && vz.y > 0 && linalg::length(vz) >= arena.goal_top_radius + arena.goal_side_radius))
    {
        const Plane sideZ = { Vec3d{0, 0, (arena.depth / 2)}, Vec3d{0, 0, -1} };
        dan = std::min(dan, dan_to_plane(point, sideZ));
    }

    // side X & ceiling (goal)
    if(point.z >= ((arena.depth / 2) + arena.goal_side_radius))
    {
        const Plane goalX = { Vec3d{arena.goal_width / 2, 0, 0}, Vec3d{-1, 0, 0} };
        const Plane goalY = { Vec3d{0, arena.goal_height, 0}, Vec3d{0, -1, 0} };

        dan = std::min({ dan,
            dan_to_plane(point, goalX),
            dan_to_plane(point, goalY),
            });
    }

    // --- corners

    // Goal back corners
    if(point.z > ((arena.depth / 2) + arena.goal_depth - arena.bottom_radius))
    {
        const Sphere bottomHorizontal = {
            Vec3d{ std::clamp(point.x,
                              arena.bottom_radius - (arena.goal_width / 2),      // actually, might use 0, because point.x is always positive
                              (arena.goal_width / 2) - arena.bottom_radius),
                   std::clamp(point.y,
                              arena.bottom_radius,
                              arena.goal_height - arena.goal_top_radius),
                   (arena.depth / 2) + arena.goal_depth - arena.bottom_radius },
            arena.bottom_radius
        };

        dan = std::min(dan, dan_to_sphere_inner(point, bottomHorizontal));
    }

    // Arena corner
    if(point.x > ((arena.width / 2) - arena.corner_radius) 
        && point.z > ((arena.depth / 2) - arena.corner_radius))
    {
        const Sphere arenaCorner = { 
            Vec3d{ (arena.width / 2) - arena.corner_radius, point.y, (arena.depth / 2) - arena.corner_radius }, 
            arena.corner_radius };
        dan = std::min(dan, dan_to_sphere_inner(point, arenaCorner));
    }

    // Goal outer corner
    if(point.z < ((arena.depth / 2) + arena.goal_side_radius))
    {
        // side X
        if(point.x < ((arena.goal_width / 2) + arena.goal_side_radius))
        {
            const Sphere cornerX = { 
                Vec3d{ (arena.goal_width / 2) + arena.goal_side_radius, 
                       point.y, 
                       (arena.depth / 2) + arena.goal_side_radius }, 
                arena.goal_side_radius };
            dan = std::min(dan, dan_to_sphere_outer(point, cornerX));
        }

        // ceiling
        if(point.y < (arena.goal_height + arena.goal_side_radius))
        {
            const Sphere cornerCeil = { 
                Vec3d{ point.x, 
                       arena.goal_height + arena.goal_side_radius, 
                       (arena.depth / 2) + arena.goal_side_radius }, 
                arena.goal_side_radius };
            dan = std::min(dan, dan_to_sphere_outer(point, cornerCeil));
        }

        // top goal gate corner
        Vec2d o /*SDK naming*/ = { (arena.goal_width / 2) - arena.goal_top_radius, 
                                    arena.goal_height - arena.goal_top_radius };
        Vec2d v /*SDK naming*/ = Vec2d{ point.x, point.y } - o;
        if(v.x > 0 && v.y > 0)
        {
            o = o + linalg::normalize(v) * (arena.goal_top_radius + arena.goal_side_radius);
            const Sphere topGateCorner = { 
                Vec3d {o.x, o.y, (arena.depth / 2) + arena.goal_side_radius}, 
                arena.goal_side_radius };
            dan = std::min(dan, dan_to_sphere_outer(point, topGateCorner));
        }
    }

    // Goal inside top corners
    if(point.z > ((arena.depth / 2) + arena.goal_side_radius) 
        && point.y > (arena.goal_height - arena.goal_top_radius))
    {
        // side X
        if(point.x > ((arena.goal_width / 2) - arena.goal_top_radius))
        {
            const Sphere topCorner = { 
                Vec3d { (arena.goal_width / 2) - arena.goal_top_radius, 
                        arena.goal_height - arena.goal_top_radius, 
                        point.z }, 
                arena.goal_top_radius };
            dan = std::min(dan, dan_to_sphere_inner(point, topCorner));
        }

        // side Z
        if(point.z > ((arena.depth / 2) + arena.goal_depth - arena.goal_top_radius))
        {
            const Sphere topCorner = { 
                Vec3d { point.x, 
                        arena.goal_height - arena.goal_top_radius, 
                        (arena.depth / 2) + arena.goal_depth - arena.goal_top_radius },
                arena.goal_top_radius };
            dan = std::min(dan, dan_to_sphere_inner(point, topCorner));
        }
    }

    // Bottom (floor) corners
    if(point.y < arena.bottom_radius)
    {
        // side X
        if(point.x > ((arena.width / 2) - arena.bottom_radius))
        {
            const Sphere& floorCornerX = { 
                Vec3d{ (arena.width / 2) - arena.bottom_radius, 
                       arena.bottom_radius, 
                       point.z }, 
                arena.bottom_radius };
            dan = std::min(dan, dan_to_sphere_inner(point, floorCornerX));
        }

        // side Z
        if(point.z > ((arena.depth / 2) - arena.bottom_radius) 
            && point.x >= ((arena.goal_width / 2) + arena.goal_side_radius))
        {
            const Sphere& floorCornerZ = { 
                Vec3d{ point.x, 
                       arena.bottom_radius, 
                       (arena.depth / 2) - arena.bottom_radius }, 
                arena.bottom_radius };
            dan = std::min(dan, dan_to_sphere_inner(point, floorCornerZ));
        }

        // side Z (goal)
        if(point.z > ((arena.depth / 2) + arena.goal_depth - arena.bottom_radius))
        {
            const Sphere& floorCornerZG = { 
                Vec3d{ point.x, 
                       arena.bottom_radius, 
                       (arena.depth / 2) + arena.goal_depth - arena.bottom_radius }, 
                arena.bottom_radius };
            dan = std::min(dan, dan_to_sphere_inner(point, floorCornerZG));
        }

        // Goal outer corner
        Vec2d o /*SDK naming*/ = { (arena.goal_width / 2) + arena.goal_side_radius, 
                                   (arena.depth / 2) + arena.goal_side_radius };
        Vec2d v /*SDK naming*/ = Vec2d{ point.x, point.z } - o;
        if(v.x < 0 && v.y < 0 
            && length(v) < (arena.goal_side_radius + arena.bottom_radius))
        {
            o = o + linalg::normalize(v) * (arena.goal_side_radius + arena.bottom_radius);
            const Sphere outerCorner = { Vec3d{ o.x, arena.bottom_radius, o.y }, arena.bottom_radius };
            dan = std::min(dan, dan_to_sphere_inner(point, outerCorner));
        }

        // Side x (goal)
        if(point.z >= ((arena.depth / 2) + arena.goal_side_radius) 
            && point.x > ((arena.goal_width / 2) - arena.bottom_radius))
        {
            const Sphere floorGoalX = { 
                Vec3d{ (arena.goal_width / 2) - arena.bottom_radius, arena.bottom_radius, point.z }, 
                arena.bottom_radius };
            dan = std::min(dan, dan_to_sphere_inner(point, floorGoalX));
        }

        // Corner
        if(point.x > ((arena.width / 2) - arena.corner_radius) 
            && point.z > ((arena.depth / 2) - arena.corner_radius))
        {
            Vec2d corner_o /*SDK naming*/ = { (arena.width / 2) - arena.corner_radius, 
                                              (arena.depth / 2) - arena.corner_radius };
            Vec2d n /*SDK naming*/ = Vec2d{ point.x, point.z } - corner_o;
            Rational dist = linalg::length(n);
            if(dist > (arena.corner_radius - arena.bottom_radius))
            {
                n /= dist;
                Vec2d o2 /*SDK naming*/ = corner_o + n * (arena.corner_radius - arena.bottom_radius);

                const Sphere corner = { Vec3d{ o2.x, arena.bottom_radius, o2.y }, arena.bottom_radius };
                dan = std::min(dan, dan_to_sphere_inner(point, corner));
            }
        }
    }

    // Ceiling corners
    if(point.y > (arena.height - arena.top_radius))
    {
        // side X
        if(point.x > ((arena.width / 2) - arena.top_radius))
        {
            const Sphere ceilX = { 
                Vec3d{ (arena.width / 2) - arena.top_radius, 
                        arena.height - arena.top_radius, 
                        point.z }, 
                arena.top_radius };
            dan = std::min(dan, dan_to_sphere_inner(point, ceilX));
        }

        // side Z
        if(point.z > ((arena.depth / 2) - arena.top_radius))
        {
            const Sphere ceilZ = { 
                Vec3d { point.x, 
                        arena.height - arena.top_radius, 
                        (arena.depth / 2) - arena.top_radius }, 
                arena.top_radius };
            dan = std::min(dan, dan_to_sphere_inner(point, ceilZ));
        }

        // corner
        if(point.x > ((arena.width / 2) - arena.corner_radius) 
            && point.z > ((arena.depth / 2) - arena.corner_radius))
        {
            Vec2d corner_o /*SDK naming*/ = { (arena.width / 2) - arena.corner_radius, 
                                              (arena.depth / 2) - arena.corner_radius };
            Vec2d dv /*SDK naming*/ = Vec2d{ point.x, point.z } - corner_o;
            if(linalg::length(dv) > (arena.corner_radius - arena.top_radius))
            {
                Vec2d n  /*SDK naming*/ = normalize(dv);
                Vec2d o2 /*SDK naming*/ = corner_o + n * (arena.corner_radius - arena.top_radius);

                const Sphere corner = { 
                    Vec3d{ o2.x, arena.height - arena.top_radius, o2.y }, 
                    arena.top_radius };
                dan = std::min(dan, dan_to_sphere_inner(point, corner));
            }
        }
    }

    return dan;
}

Simulator::Dan Simulator::dan_to_arena(Vec3d point)
{
    bool isNegateX = point.x < 0;
    bool isNegateZ = point.z < 0;

    if(isNegateX)
        point.x = -point.x;
    if(isNegateZ)
        point.z = -point.z;

    Dan results = dan_to_arena_quarter(point);
    if(isNegateX)
        results.normal.x = -results.normal.x;
    if(isNegateZ)
        results.normal.z = -results.normal.z;

    return results;
}

Simulator::CollisionFlags Simulator::update(std::vector<Entity<model::Robot>>& robots, Entity<model::Ball>& ball, Rational delta_time)
{
    m_robotPointersBuffer.resize(0);   // resize() ensures that capacity is never reduced
    CollisionFlags collisions; 

    std::transform(robots.begin(), robots.end(), std::back_inserter(m_robotPointersBuffer), [](Entity<model::Robot>& r) { return &r; });
    std::shuffle(m_robotPointersBuffer.begin(), m_robotPointersBuffer.end(), m_rng);

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
            if(collide_entities(robots[i], robots[j]))
                collisions.robots = true;

    for(Entity<model::Robot>& robot : robots)
    {
        if(collide_entities(robot, ball))
            collisions.ball = collisions.robots = true;

        std::optional<Vec3d> collisionNormal = collide_with_arena(robot);
        if(collisionNormal.has_value())
        {
            robot.touch = true;
            robot.setTouchNormal(*collisionNormal);
            collisions.robots = true;
        }
        else
        {
            robot.touch = false;
        }
    }

    if(collide_with_arena(ball).has_value())
        collisions.ball = true;

    // #todo st.3 - goal callback?
    if(abs(ball.z) > m_rules.arena.depth / 2 + ball.radius)
         ball.velocity_z = 0;    // there is no bounce from gates net

    // #todo st. 2- nitro packs

    return collisions;
}
