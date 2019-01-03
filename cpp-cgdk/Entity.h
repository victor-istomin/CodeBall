#pragma once
#include "linalg.h"
#include <functional>
#include "model/Ball.h"
#include "model/Robot.h"
#include "model/Action.h"

template <typename Type>       // either model::Ball or model::Robot
class Entity : public Type
{
public:
    using Rational = double;
    using Vec3d    = linalg::vec<Rational, 3>;

    // standard conformance using's for dependent base lookup:
    using Type::velocity_x; using Type::velocity_y; using Type::velocity_z;
    using Type::x; using Type::y; using Type::z;

private:
    Rational m_radiusChangeSpeed = 0;      // #todo - strategy is responsible of settings this manually!
    model::Action m_action;                // #todo - strategy is responsible of settings this manually!

public:
    Entity()              = default;
    Entity(const Entity&) = default;
    Entity(Entity&&)      = default;

    Entity(const Type& copy) : Entity()       { *this = copy; }
    Entity(Type&& copy) : Entity()            { *this = std::forward<Type&&>(copy); }
    Entity<Type>& operator=(Type&& copy)      { static_cast<Type&>(*this) = std::forward<Type&&>(copy); return *this; }
    Entity<Type>& operator=(const Type& copy) { static_cast<Type&>(*this) = copy; return *this; }

    Vec3d position()    const                 { return Vec3d { x, y, z }; }
    Vec3d velocity()    const                 { return Vec3d { velocity_x, velocity_y, velocity_z }; }
    Rational radiusChangeSpeed() const        { return m_radiusChangeSpeed; }

    void setPosition(const Vec3d& v)          { x = v.x; y = v.y; z = v.z; }
    void setVelocity(const Vec3d& v)          { velocity_x = v.x; velocity_y = v.y; velocity_z = v.z; }
    void setRadiusChangeSpeed(Rational s)     { m_radiusChangeSpeed = s; }

    const model::Action& action() const       { return m_action; }
    model::Action& action()                   { return m_action; }
    Vec3d actionTargetVelocity() const        { return Vec3d{ m_action.target_velocity_x, m_action.target_velocity_y, m_action.target_velocity_z }; }
    void setAction(const model::Action& a)    { m_action = a; }

    template <typename S = Type> auto getId() const -> std::enable_if_t< std::is_same_v<S, model::Robot>, int> { return (int)this->id; }
    template <typename S = Type> auto getId() const -> std::enable_if_t<!std::is_same_v<S, model::Robot>, int> { return (int)-1; }

    template <typename S = Type> auto radiusChangeSpeed()                  const -> std::enable_if_t< std::is_same_v<S, model::Robot>, Rational>{ return m_radiusChangeSpeed; }
    template <typename S = Type> auto radiusChangeSpeed()                  const -> std::enable_if_t<!std::is_same_v<S, model::Robot>, Rational>{ return 0; }
    template <typename S = Type> auto setRadiusChangeSpeed(Rational speed) const -> std::enable_if_t< std::is_same_v<S, model::Robot>, Rational>{ return m_radiusChangeSpeed = speed; }
    template <typename S = Type> auto setRadiusChangeSpeed(Rational speed) const -> std::enable_if_t<!std::is_same_v<S, model::Robot>, Rational>{ return 0; }

    template <typename S = Type> auto touchNormal() const -> std::enable_if_t< std::is_same_v<S, model::Robot>, Vec3d> 
    {
        return Vec3d{ this->touch_normal_x, this->touch_normal_y, this->touch_normal_z };
    }

    template <typename S = Type> auto setTouchNormal(const Vec3d& v)  -> std::enable_if_t< std::is_same_v<S, model::Robot>, void>
    { 
        this->touch_normal_x = v.x; 
        this->touch_normal_y = v.y; 
        this->touch_normal_z = v.z; 
    }

};

