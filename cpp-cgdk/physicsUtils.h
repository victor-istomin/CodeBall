#pragma once
#include <cmath>

template <typename Rational> 
Rational pow2(Rational x) 
{ 
    return x * x;
};

// uniform acceleration movement, min. time to run 's' distance
template <typename Rational> 
Rational timeToApproach(Rational s, Rational v0, Rational a)
{ 
    return std::sqrt(pow2(v0) + 2 * a * s) - v0; 
};

// uniform acceleration movement, min. time to run 's' distance
template <typename Rational>
Rational jumpHeight(Rational v0, Rational a)
{
    // s = (pow2(v) - pow2(v0)) / 2a, in this case v = 0:
    return pow2(v0) / (2 * a);
}
