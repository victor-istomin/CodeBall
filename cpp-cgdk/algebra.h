#pragma once

#undef min
#undef max
#include "linalg.h"

using Rational = double;
using Vec3d = linalg::vec<Rational, 3>;
using Vec2d = linalg::vec<Rational, 2>;

constexpr static Rational k_Epsilon = 1e-6;

// text
