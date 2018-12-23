#pragma once
#include "linalg.h"
#include <smmintrin.h>
#include <immintrin.h>

namespace simd
{
    // split into two Vec2d:
    inline __m128d get_low(__m256d ymm) { return _mm256_castpd256_pd128(ymm); }
    inline __m128d get_high(__m256d ymm) { return _mm256_extractf128_pd(ymm, 1); }

    // Horizontal add: Calculates the sum of all vector elements. @todo - better comment
    inline __m128d horizontal_add_pair(__m256d a)
    {
        __m256d t1 = _mm256_hadd_pd(a, a);
        //__m128d t3 = _mm_add_sd(get_high(t1), get_low(t1));
        __m128d high = get_high(t1);
        __m128d low = get_low(t1);
        __m128d t3 = _mm_add_sd(high, low);
        return t3;
    }

    inline __m128d dot_simd(const linalg::vec<double, 3> & a, const linalg::vec<double, 3> & b)
    {
        __m128d aXY = _mm_load_pd(&a[0]);
        __m128d bXY = _mm_load_pd(&b[0]);
        __m128d aZW = _mm_set_sd(a[2]);
        __m128d bZW = _mm_set_sd(b[2]);

        __m128d abXY = _mm_dp_pd(aXY, bXY, 0b11111111);
        __m128d abZW = _mm_dp_pd(aZW, bZW, 0b11111111);

        __m128d res = _mm_add_pd(abXY, abZW);

        return res;
// tmp0 := (mask4 == 1) ? (a0 * b0) : +0.0
// tmp1 := (mask5 == 1) ? (a1 * b1) : +0.0
// tmp2 := tmp0 + tmp1
// r0 := (mask0 == 1) ? tmp2 : +0.0
// r1 := (mask1 == 1) ? tmp2 : +0.0        
    }

    inline __m128d length2_simd(const linalg::vec<double, 3> & a)
    {
        __m128d aXY = _mm_load_pd(&a[0]);
        __m128d aZW = _mm_set_sd(a[2]);

        __m128d abXY = _mm_dp_pd(aXY, aXY, 0b11111111);
        __m128d abZW = _mm_dp_pd(aZW, aZW, 0b11111111);

        __m128d res = _mm_add_pd(abXY, abZW);
        return res;
    }

    inline __m128d length_simd(const linalg::vec<double, 3> & a)
    {
        return _mm_sqrt_pd(length2_simd(a));
    }

    inline linalg::vec<double, 3> normalize(const linalg::vec<double, 3> & a)
    {
        __m128d l = length_simd(a);
        __m128d xy = _mm_load_pd(&a[0]);
        __m128d zw = _mm_set_sd(a[2]);
        xy = _mm_div_pd(xy, l);
        zw = _mm_div_pd(zw, l);

        std::array<double, 3> xyz;
        _mm_store_pd(xyz.data(), xy);
        _mm_store_sd(xyz.data() + 1, zw);
        return linalg::vec<double, 3> {xyz.data()};
    }

    inline double length(const linalg::vec<double, 3>& a)
    {
        return _mm_cvtsd_f64(length_simd(a));
    }
}
