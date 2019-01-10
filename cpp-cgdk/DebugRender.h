#pragma once
#include <string>
#include <vector>
#include "linalg.h"

class DebugRender
{
    using Vec3d = linalg::vec<double, 3>;
    using Rgba = linalg::vec<double, 4>;

    struct Sphere
    {
        Vec3d  center;
        double radius;
        Rgba   color;
    };

    DebugRender() = default;

    std::vector<Sphere>      m_spheres;
    std::vector<std::string> m_strings;
    bool                     m_isEnabled = false;

    DebugRender& enableFrame() { m_isEnabled = true; return *this; }

public:
    ~DebugRender() = default;

    constexpr static const Rgba RGB_BALL_PREDICTED = { 0.0, 1.0, 1.0, 0.25 };

#ifdef DEBUG_RENDER
    DebugRender& text(std::string&& s)                              { enableFrame(); m_strings.emplace_back(std::forward<std::string>(s)); return *this; }
    DebugRender& shpere(const Vec3d& point, double r, Rgba color)   { enableFrame(); m_spheres.emplace_back(Sphere{point, r, color});      return *this; }
    std::string  commit()                                           { return commitImpl(); }
    bool         isEnabled() const                                  { return m_isEnabled; }

#else
    DebugRender& text(std::string&&)                           { return *this; }
    DebugRender& shpere(Vec3d&&, double, Rgba)                 { return *this; }
    std::string  commit()                                      { return {}; }
    bool         isEnabled() const                             { return false; }
#endif

    std::string commitImpl();

    static DebugRender& instance()
    {
        static DebugRender r;
        return r;
    }
};

