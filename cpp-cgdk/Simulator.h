#pragma once
#undef min
#undef max
#include "Entity.h"
#include "model/Rules.h"
#include <algorithm>

class Simulator
{
public:
	using Vec3d = linalg::vec<double, 3>;
	using Rational = double;

private:
	const Rational     m_minHitE;
	const Rational     m_maxHitE;
	const Rational     m_robotMass;
	const Rational     m_ballMass;
    const model::Rules m_rules;

	template <typename Unit> auto getMass(const Unit& u) -> std::enable_if_t<std::is_base_of_v<model::Robot, Unit>, Rational> { return m_robotMass; }
	template <typename Unit> auto getMass(const Unit& u) -> std::enable_if_t<std::is_base_of_v<model::Ball,  Unit>, Rational> { return m_ballMass; }

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
        Vec3d center;
        Rational radius;
    };

	static Dan dan_to_plane(const Vec3d& point, const Plane& plane)
	{
		return Dan{ linalg::dot(point - plane.point, plane.normal), plane.normal };
	}

    static Dan dan_to_sphere_inner(const Vec3d& point, const Sphere& sphere)
	{
		return Dan{ sphere.radius - linalg::length(point - sphere.center), linalg::normalize(sphere.center- point) };
	}

	static Dan dan_to_sphere_outer(const Vec3d& point, const Sphere& sphere)
	{
		return Dan{ linalg::length(point - sphere.center) - sphere.radius, linalg::normalize(point - sphere.center) };
	}

    Dan dan_to_arena_quarter(const Vec3d& point)
    {
        const model::Arena& arena = m_rules.arena;

        // --- planes
        const Plane ground = { Vec3d{ 0, 0, 0 }, Vec3d{ 0, 1, 0 } };
        const Plane ceiling = { Vec3d{ 0, arena.height, 0 }, Vec3d{ 0, -1, 0 } };
        const Plane sideX = { Vec3d{arena.width / 2, 0, 0}, Vec3d{-1, 0 , 0} };
        const Plane goalZ = { Vec3d{0, 0, (arena.depth / 2) + arena.goal_depth}, Vec3d{0, 0, -1} };
        const Plane sideZ = { Vec3d{0, 0, (arena.depth / 2)}, Vec3d{0, 0, -1} };
        const Plane goalX = { Vec3d{arena.goal_width / 2, 0, 0}, Vec3d{-1, 0, 0} };
        const Plane goalY = { Vec3d{0, arena.goal_height, 0}, Vec3d{0, -1, 0} };

        Dan dan = std::min({
                dan_to_plane(point, ground),
                dan_to_plane(point, ceiling),
                dan_to_plane(point, sideX),
                dan_to_plane(point, goalZ),
            });

        using Vec2d = linalg::vec<Rational, 2>;
        const Vec2d pointXY = point.xy();
        Vec2d arenaSqGate = { (arena.goal_width / 2) - arena.goal_top_radius, arena.goal_height - arena.goal_top_radius };

        // side Z (non-goal)
        Vec2d v /*name in SDK*/ = pointXY - arenaSqGate;
        if(    (pointXY.x >= (arena.goal_width / 2) + arena.goal_side_radius)
            || (pointXY.y >= arena.goal_height + arena.goal_side_radius)
            || (v.x > 0 && v.y > 0 && length(v) >= arena.goal_top_radius + arena.goal_side_radius))
        {
            dan = std::min(dan, dan_to_plane(point, sideZ));
        }

        // side X & ceiling (goal)
        if(point.z >= ((arena.depth / 2) + arena.goal_side_radius))
        {
            dan = std::min({ dan,
                dan_to_plane(point, goalX),
                dan_to_plane(point, goalY),
                });
        }

        // --- corners

        // Goal back corners
        if(point.z > (arena.depth / 2) + arena.goal_depth - arena.bottom_radius)
        {
            const Sphere bottomHorizontal = {
                Vec3d{ std::clamp(point.x,
                                  arena.bottom_radius - (arena.goal_width / 2),
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
		if(point.x > (arena.width / 2) - arena.corner_radius && point.z > (arena.depth / 2) - arena.corner_radius)
		{
			const Sphere arenaCorner = { Vec3d{ (arena.width / 2) - arena.corner_radius, point.y, (arena.depth / 2) - arena.corner_radius }, arena.corner_radius };
			dan = std::min(dan, dan_to_sphere_inner(point, arenaCorner));
		}

		// Goal outer corner
		if(point.z < (arena.depth / 2) + arena.goal_side_radius)
		{
			// side X
			if(point.x < (arena.goal_width / 2) + arena.goal_side_radius)
			{
				const Sphere cornerX = { Vec3d{ (arena.goal_width / 2) + arena.goal_side_radius, point.y, (arena.depth / 2) + arena.goal_side_radius }, arena.goal_side_radius };
				dan = std::min(dan, dan_to_sphere_outer(point, cornerX));
			}

			// ceiling
			if(point.y < arena.goal_height + arena.goal_side_radius)
			{
				const Sphere cornerCeil = { Vec3d{ point.x, arena.goal_height + arena.goal_side_radius, (arena.depth / 2) + arena.goal_side_radius }, arena.goal_side_radius };
				dan = std::min(dan, dan_to_sphere_outer(point, cornerCeil));
			}

			// top goal gate corner
			Vec2d o /*SDK naming*/ = { (arena.goal_width / 2) - arena.goal_top_radius, arena.goal_height - arena.goal_top_radius };
			Vec2d v /*SDK naming*/ = pointXY - o;
			if(v.x > 0 && v.y > 0)
			{
				o = o + linalg::normalize(v) * (arena.goal_top_radius + arena.goal_side_radius);
				const Sphere topGateCorner = { Vec3d {o.x, o.y, (arena.depth / 2) + arena.goal_side_radius}, arena.goal_side_radius };
				dan = std::min(dan, dan_to_sphere_outer(point, topGateCorner));
			}
		}

		// Goal inside top corners
		if(point.z > (arena.depth / 2) + arena.goal_side_radius && point.y > arena.goal_height - arena.goal_top_radius)
		{
			// side X
			if(point.x > (arena.goal_width / 2) - arena.goal_top_radius)
			{
				const Sphere topCorner = { Vec3d { (arena.goal_width / 2) - arena.goal_top_radius, arena.goal_height - arena.goal_top_radius, point.z }, arena.goal_top_radius };
				dan = std::min(dan, dan_to_sphere_inner(point, topCorner));
			}

			// side Z
			if(point.z > (arena.depth / 2) + arena.goal_depth - arena.goal_top_radius)
			{
				const Sphere topCorner = { Vec3d { point.x, arena.goal_height - arena.goal_top_radius, (arena.depth / 2) + arena.goal_depth - arena.goal_top_radius }, arena.goal_top_radius };
				dan = std::min(dan, dan_to_sphere_inner(point, topCorner));
			}
		}

		// Bottom (floor) corners
		if(point.y < arena.bottom_radius)
		{
			// side X
			if(point.x > (arena.width / 2) - arena.bottom_radius)
			{
				const Sphere& floorCornerX = { Vec3d{ (arena.width / 2) - arena.bottom_radius, arena.bottom_radius, point.z }, arena.bottom_radius };
				dan = std::min(dan, dan_to_sphere_inner(point, floorCornerX));
			}

			// side Z
			if(point.z > (arena.depth / 2) - arena.bottom_radius && point.x >= (arena.goal_width / 2) + arena.goal_side_radius)
			{
				const Sphere& floorCornerZ = { Vec3d{ point.x, arena.bottom_radius, (arena.depth / 2) - arena.bottom_radius }, arena.bottom_radius };
				dan = std::min(dan, dan_to_sphere_inner(point, floorCornerZ));
			}

			// side Z (goal)
			if(point.z > (arena.depth / 2) + arena.goal_depth - arena.bottom_radius)
			{
				const Sphere& floorCornerZG = { Vec3d{ point.x, arena.bottom_radius, (arena.depth / 2) + arena.goal_depth - arena.bottom_radius }, arena.bottom_radius };
				dan = std::min(dan, dan_to_sphere_inner(point, floorCornerZG));
			}

			// Goal outer corner
			Vec2d o /*SDK naming*/ = { (arena.goal_width / 2) + arena.goal_side_radius, (arena.depth / 2) + arena.goal_side_radius };
			Vec2d v /*SDK naming*/ = Vec2d{ point.x, point.z } - o;
			if(v.x < 0 && v.y < 0 && length(v) < arena.goal_side_radius + arena.bottom_radius)
			{
				o = o + linalg::normalize(v) * (arena.goal_side_radius + arena.bottom_radius);
				const Sphere outerCorner = { Vec3d{ o.x, arena.bottom_radius, o.y }, arena.bottom_radius };
				dan = std::min(dan, dan_to_sphere_inner(point, outerCorner));
			}

			// Side x (goal)
			if(point.z >= (arena.depth / 2) + arena.goal_side_radius && point.x > (arena.goal_width / 2) - arena.bottom_radius)
			{
				const Sphere floorGoalX = { Vec3d{ (arena.goal_width / 2) - arena.bottom_radius, arena.bottom_radius, point.z }, arena.bottom_radius };
				dan = std::min(dan, dan_to_sphere_inner(point, floorGoalX));
			}

			// Corner
			if(point.x > (arena.width / 2) - arena.corner_radius && point.z > (arena.depth / 2) - arena.corner_radius)
			{
				Vec2d corner_o /*SDK naming*/ = { (arena.width / 2) - arena.corner_radius, (arena.depth / 2) - arena.corner_radius };
				Vec2d n /*SDK naming*/ = Vec2d{ point.x, point.z } - corner_o;
				Rational dist = linalg::length(n);
				if(dist > arena.corner_radius - arena.bottom_radius)
				{
					n /= dist;
					Vec2d o2 /*SDK naming*/ = corner_o + n * (arena.corner_radius - arena.bottom_radius);

					const Sphere corner = { Vec3d{ o2.x, arena.bottom_radius, o2.y }, arena.bottom_radius };
					dan = std::min(dan, dan_to_sphere_inner(point, corner));
				}
			}
		}

		// Ceiling corners
		if(point.y > arena.height - arena.top_radius)
		{
			// side X
			if(point.x > (arena.width / 2) - arena.top_radius)
			{
				const Sphere ceilX = { Vec3d{ (arena.width / 2) - arena.top_radius, arena.height - arena.top_radius, point.z }, arena.top_radius };
				dan = std::min(dan, dan_to_sphere_inner(point, ceilX));
			}

			// side Z
			if(point.z > (arena.depth / 2) - arena.top_radius)
			{
				const Sphere ceilZ = { Vec3d { point.x, arena.height - arena.top_radius, (arena.depth / 2) - arena.top_radius }, arena.top_radius };
				dan = std::min(dan, dan_to_sphere_inner(point, ceilZ));
			}

			// corner
			if(point.x > (arena.width / 2) - arena.corner_radius && point.z > (arena.depth / 2) - arena.corner_radius)
			{
				Vec2d corner_o /*SDK naming*/ = { (arena.width / 2) - arena.corner_radius, (arena.depth / 2) - arena.corner_radius };
				Vec2d dv /*SDK naming*/ = Vec2d{ point.x, point.z } - corner_o;
				if(linalg::length(dv) > arena.corner_radius - arena.top_radius)
				{
					Vec2d n  /*SDK naming*/ = normalize(dv);
					Vec2d o2 /*SDK naming*/ = corner_o + n * (arena.corner_radius - arena.top_radius);

					const Sphere corner = { Vec3d{ o2.x, arena.height - arena.top_radius, o2.y }, arena.top_radius };
					dan = std::min(dan, dan_to_sphere_inner(point, corner));
				}
			}
		}

		return dan;
    }

	Dan dan_to_arena(Vec3d point)
	{
		// field is symmetrical
		bool negate_x = point.x < 0;
		bool negate_z = point.z < 0;
		
		if(negate_x)
			point.x = -point.x;
		if(negate_z)
			point.z = -point.z;

		Dan result = dan_to_arena_quarter(point);

		if(negate_x)
			result.normal.x = -result.normal.x;
		if(negate_z)
			result.normal.z = -result.normal.z;

		return result;
	}


public:

	Simulator(const model::Rules& rules, Rational minHitE, Rational maxHitE, Rational robotMass, Rational ballMass) 
		: m_minHitE(minHitE)
		, m_maxHitE(maxHitE)
		, m_robotMass(robotMass)
		, m_ballMass(ballMass)
        , m_rules(rules)
	{
		testCollide();
	}

	Rational random(double min, double max) { return (min + max) / 2; }  // #todo - random

	template <typename LeftEntity, typename RightEntity>
	void collide_entities(LeftEntity& a, RightEntity& b)
	{
		Vec3d    delta_pos   = b.position() - a.position();
		Rational distance    = linalg::length(delta_pos);
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
				Vec3d impulse = (1 + random(m_minHitE, m_maxHitE)) * delta_v * normal;
				a.setVelocity(a.velocity() + impulse * k_a);
				b.setVelocity(b.velocity() - impulse * k_b);
			}
		}
	}

	void testCollide()
	{
		constexpr double Epsilon = 0.0000001;

		Entity<model::Robot> r = model::Robot {
			0, // int    id;
			0, // int    player_id;
			true, // bool   is_teammate;
			0.1, // double x;
			0.1, // double y;
			0.1, // double z;
			1, // double velocity_x;
			1, // double velocity_y;
			1, // double velocity_z;
			1, // double radius;
			0, // double nitro_amount;
			true, // bool   touch;
			0, // double touch_normal_x;
			1, // double touch_normal_y;
			0, // double touch_normal_z;
		};

		Entity<model::Ball> b = model::Ball {
			.5,  // double x;
			2,   // double y;
			2,   // double z;
			.1,  // double velocity_x;
			.1,  // double velocity_y;
			.1,  // double velocity_z;
			2,   // double radius;
		};

		auto sumMoment = getMass(r) * r.velocity() + getMass(b) * b.velocity();
		auto distance = linalg::length(r.position() - b.position());

		assert(distance < (r.radius + b.radius));
		collide_entities(r, b);

		auto sumMomentAfter = getMass(r) * r.velocity() + getMass(b) * b.velocity();
		auto distanceAfter = linalg::length(r.position() - b.position());
		assert(sumMoment == sumMomentAfter);
		assert(Epsilon > std::abs(distanceAfter - (r.radius + b.radius)));
	}
};

