#pragma once
#include "linalg.h"
#include <functional>
#include "model/Ball.h"
#include "model/Robot.h"

template <typename Type>       // either model::Ball or model::Robot
class Entity : public Type
{
public:
	using Rational = double;
	using Vec3d    = linalg::vec<Rational, 3>;

private:
	Rational m_radiusChangeSpeed = 0;      // #todo - strategy is responsible of settings this manually!
    model::Action m_action;              // #todo - strategy is responsible of settings this manually!

public:
	Entity()              = default;
	Entity(const Entity&) = default;
	Entity(Entity&&)      = default;

	Entity(Type&& copy) : Entity()       { *this = std::forward<Type&&>(copy); }
	Entity<Type>& operator=(Type&& copy) { static_cast<Type&>(*this) = std::forward<Type&&>(copy); return *this; }

	Vec3d position()    const            { return Vec3d { x, y, z }; }
	Vec3d velocity()    const            { return Vec3d { velocity_x, velocity_y, velocity_z }; }
	Vec3d touchNormal() const            { return Vec3d { touch_normal_x, touch_normal_y, touch_normal_z }; }
    Rational radiusChangeSpeed() const   { return m_radiusChangeSpeed; }

	void setPosition(Vec3d&& v)          { x = v.x; y = v.y; z = v.z; }
	void setVelocity(Vec3d&& v)           { velocity_x = v.x; velocity_y = v.y; velocity_z = v.z; }
	void setTouchNormal(const Vec3d& v)   { touch_normal_x = v.x; touch_normal_y = v.y; touch_normal_z = v.z; }
    void setRadiusChangeSpeed(Rational s) { m_radiusChangeSpeed = s; }

    const model::Action& action() const  { return m_action; }
    Vec3d actionTargetVelocity() const { return Vec3d{ m_action.target_velocity_x, m_action.target_velocity_y, m_action.target_velocity_z }; }

	template <typename S = Type> auto radiusChangeSpeed()                  const -> std::enable_if_t< std::is_same_v<S, model::Robot>, Rational>{ return m_radiusChangeSpeed; }
	template <typename S = Type> auto radiusChangeSpeed()                  const -> std::enable_if_t<!std::is_same_v<S, model::Robot>, Rational>{ return 0; }
	template <typename S = Type> auto setRadiusChangeSpeed(Rational speed) const -> std::enable_if_t< std::is_same_v<S, model::Robot>, Rational>{ return m_radiusChangeSpeed = speed; }
	template <typename S = Type> auto setRadiusChangeSpeed(Rational speed) const -> std::enable_if_t<!std::is_same_v<S, model::Robot>, Rational>{ return 0; }


};

