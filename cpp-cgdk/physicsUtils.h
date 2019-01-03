#pragma once
#include <cmath>

template <typename Rational> 
Rational pow2(Rational x) 
{ 
    return x * x;
};

namespace uniform_accel     // uniform acceleration movement
{
    template <typename Rational>
    Rational timeToSpeed(Rational v0, Rational a, Rational t)
    {
        return v0 + a * t;
    }

    template <typename Rational>
    Rational speedToTime(Rational v0, Rational a, Rational v)
    {
        return (v - v0) / a;
    }

    template <typename Rational>
    Rational timeToDistance(Rational v0, Rational a, Rational t)
    {
        return v0 * t + a * pow2(t) / 2;
    }

//     template <typename Rational>
//     Rational timeToStop(Rational v0, Rational a)
//     {
//         // s = (pow2(v) - pow2(v0)) / 2a, in this case v = 0:
//         return pow2(v0) / (2 * a);
//     }

    // min. time to run 's' distance
    template <typename Rational>
    Rational distanceToTime(Rational s, Rational a, Rational v0)
    {
        if(a != 0)
        {
            double D = pow2(v0) + 2 * a * s;
            double rd = std::sqrt(D);
            double x1 = (-v0 + rd) / a;
            double x2 = (-v0 - rd) / a;

            double min = std::min(x1, x2);
            double max = std::max(x1, x2);

            return min > 0 ? min : max;
        }

        return v0 != 0 ? (s / v0) : std::numeric_limits<Rational>::max();
    };

    // min. time to run 's' distance
    template <typename Rational>
    Rational distanceToTime(Rational s, Rational a, Rational v0, Rational vLimit)
    {
        double timeWithoultVLimit = distanceToTime(s, a, v0);
        double maxV = timeToSpeed(v0, a, timeWithoultVLimit);

        if(maxV > vLimit)
        {
            double timeToLimit     = speedToTime(v0, a, vLimit);
            double distanceToLimit = timeToDistance(v0, a, timeToLimit);
            double distanceAtLimit = s - distanceToLimit;
            double timeAfterLimit  = distanceToTime(distanceAtLimit, 0.0, vLimit);

            return timeToLimit + timeAfterLimit;
        }
        else
        {
            return timeWithoultVLimit;
        }
    };


}


