#pragma once
#include <string>
#include "linalg.h"

using Vec3d = linalg::vec<double, 3>;

namespace std
{
    inline std::string to_string(const Vec3d& v)
    {
        return "(" + std::to_string(v.x) + "; " + std::to_string(v.y) + "; " + std::to_string(v.z) + ")";
    }

    inline std::string to_string(const std::string& s)
    {
        return s;
    }
}

template <typename Value> std::string& format(std::string& s, const std::string& spec, const Value& value)
{
    auto pos = s.find(spec);
    auto val = std::to_string(value);

    while(pos != std::string::npos)
    {
        s.replace(pos, spec.size(), val);
        pos = s.find(spec);
    }

    return s;
};

class FormattedString
{
    std::string m_result;

public:
    FormattedString(std::string s)   : m_result(std::move(s)) {}

    template <typename Value> FormattedString& format(const std::string& spec, const Value& value)
    {
        ::format(m_result, spec, value);
        return *this;
    }

    const std::string& get() const { return m_result; }
    std::string&& move() { return std::move(m_result); }
};

inline FormattedString operator "" _fs(const char* begin, std::size_t size) { return FormattedString(std::string{ begin, size }); }

