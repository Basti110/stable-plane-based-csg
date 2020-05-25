#pragma once

#ifdef _MSC_VER
#include <intrin.h>
#else
#include <x86intrin.h>
#endif

#include <cstring>
#include <string>
//#include <stdfwd.hh> // streams
#include <typed-geometry/tg.hh>

#include <typed-geometry/detail/macros.hh>
#include <hash.hh>

// macros for conditional 2 complement

#define OB_NEG_CONDITIONAL_64(value, flag) \
    do                                     \
    {                                      \
        value = (value ^ flag) - flag;     \
    } while (false)

#define OB_NEG_CONDITIONAL_128(v0, v1, flag) \
    do                                       \
    {                                        \
        v0 = (u64(flag) ^ v0) - u64(flag);   \
        auto const carry = (v0 == 0) & flag; \
        v1 = (u64(flag) ^ v1) + u64(carry);  \
    } while (false)

#define OB_NEG_CONDITIONAL_192(v0, v1, v2, flag)       \
    do                                                 \
    {                                                  \
        v0 = (u64(flag) ^ v0) - u64(flag);             \
        auto const carry0 = (v0 == 0) & flag;          \
        v1 = (u64(flag) ^ v1) + u64(carry0);           \
        auto const carry1 = (v1 == 0) & flag & carry0; \
        v2 = (u64(flag) ^ v2) + u64(carry1);           \
    } while (false)


#define OB_NEG_CONDITIONAL_256(v0, v1, v2, v3, flag)   \
    do                                                 \
    {                                                  \
        v0 = (u64(flag) ^ v0) - u64(flag);             \
        auto const carry0 = (v0 == 0) & flag;          \
        v1 = (u64(flag) ^ v1) + u64(carry0);           \
        auto const carry1 = (v1 == 0) & flag & carry0; \
        v2 = (u64(flag) ^ v2) + u64(carry1);           \
        auto const carry2 = (v2 == 0) & flag & carry1; \
        v3 = (u64(flag) ^ v3) + u64(carry2);           \
    } while (false)

namespace ob
{
using i32 = int;
using i64 = long long;
using u64 = unsigned long long;

struct i128
{
    u64 v0 = 0; // low
    u64 v1 = 0; // high

    constexpr i128() noexcept = default;
    constexpr i128(i128 const& rhs) noexcept = default;
    constexpr i128(i128&& rhs) noexcept = default;
    constexpr i128& operator=(i128 const& rhs) noexcept = default;
    constexpr i128& operator=(i128&& rhs) noexcept = default;

    constexpr i128(u64 low, u64 high) : v0(low), v1(high) {}
    explicit constexpr i128(i64 v) : v0(static_cast<u64>(v)), v1(v < 0 ? tg::max<u64>() : 0) {}
    explicit constexpr i128(long v) : v0(static_cast<u64>(v)), v1(v < 0 ? tg::max<u64>() : 0) {}
    explicit constexpr i128(int v) : v0(static_cast<u64>(v)), v1(v < 0 ? tg::max<u64>() : 0) {}

#ifndef _MSC_VER
    explicit i128(__int128 v) { memcpy(this, &v, sizeof(v)); }

    constexpr void operator=(i128 const& rhs) volatile // for test
    {
        v0 = rhs.v0;
        v1 = rhs.v1;
    }
    constexpr i128(i128 volatile const& rhs) : v0(rhs.v0), v1(rhs.v1) {} // for test
#endif

    static inline constexpr i128 max() { return i128{u64(0x7FFFFFFFFFFFFFFF), u64(0xFFFFFFFFFFFFFFFF)}; }
    static inline constexpr i128 min() { return i128{u64(0x8000000000000000), u64(0x0000000000000000)}; }

    constexpr size_t hash() const
    {
        size_t seed = 0;
        util::hash_combine(seed, v0);
        util::hash_combine(seed, v1);
        return seed;
    }

    explicit operator double() const;

    bool operator==(i128 const& rhs) const noexcept { return v0 == rhs.v0 && v1 == rhs.v1; }
    bool operator!=(i128 const& rhs) const noexcept { return v0 != rhs.v0 || v1 != rhs.v1; }

    bool operator==(int rhs) const noexcept { return (v1 == 0 && i64(v0) == rhs) || (i64(v1) == -1 && i64(v0) == rhs); }
    bool operator!=(int rhs) const noexcept { return !operator==(rhs); }
};

struct i192
{
    u64 v0 = 0; // low
    u64 v1 = 0;
    u64 v2 = 0; // high

    constexpr i192() noexcept = default;
    constexpr i192(i192 const& rhs) noexcept = default;
    constexpr i192(i192&& rhs) noexcept = default;
    constexpr i192& operator=(i192 const& rhs) noexcept = default;
    constexpr i192& operator=(i192&& rhs) noexcept = default;

    explicit constexpr i192(i64 i) : v0(u64(i)), v1(i < 0 ? u64(-1) : 0), v2(i < 0 ? u64(-1) : 0) {}
    constexpr i192(u64 v0, u64 v1, u64 v2) : v0(v0), v1(v1), v2(v2) {}

    static inline constexpr i192 max() { return i192{u64(0x7FFFFFFFFFFFFFFF), u64(0xFFFFFFFFFFFFFFFF), u64(0xFFFFFFFFFFFFFFFF)}; }
    static inline constexpr i192 min() { return i192{u64(0x8000000000000000), u64(0x0000000000000000), u64(0x0000000000000000)}; }

#ifndef _MSC_VER
    constexpr void operator=(i192 const& rhs) volatile // for test
    {
        v0 = rhs.v0;
        v1 = rhs.v1;
        v2 = rhs.v2;
    }

    constexpr i192(i192 volatile const& rhs) : v0(rhs.v0), v1(rhs.v1), v2(rhs.v2) {} // for test
#endif

    constexpr size_t hash() const
    {
        size_t seed = 0;
        util::hash_combine(seed, v0);
        util::hash_combine(seed, v1);
        util::hash_combine(seed, v2);
        return seed;
    }

    explicit operator double() const;

    bool operator==(i192 const& rhs) const noexcept { return v0 == rhs.v0 && v1 == rhs.v1 && v2 == rhs.v2; }
    bool operator!=(i192 const& rhs) const noexcept { return v0 != rhs.v0 || v1 != rhs.v1 || v2 != rhs.v2; }
};

struct i256
{
    u64 v0 = 0; // low
    u64 v1 = 0;
    u64 v2 = 0;
    u64 v3 = 0; // high

    constexpr i256() noexcept = default;
    constexpr i256(i256 const& rhs) noexcept = default;
    constexpr i256(i256&& rhs) noexcept = default;
    constexpr i256& operator=(i256 const& rhs) noexcept = default;
    constexpr i256& operator=(i256&& rhs) noexcept = default;

    explicit constexpr i256(i64 i) : v0(u64(i)), v1(i < 0 ? u64(-1) : 0), v2(i < 0 ? u64(-1) : 0), v3(i < 0 ? u64(-1) : 0) {}
    constexpr i256(u64 v0, u64 v1, u64 v2, u64 v3) : v0(v0), v1(v1), v2(v2), v3(v3) {}

#ifndef _MSC_VER
    constexpr void operator=(i256 const& rhs) volatile // for test
    {
        v0 = rhs.v0;
        v1 = rhs.v1;
        v2 = rhs.v2;
        v3 = rhs.v3;
    }
    constexpr i256(i256 volatile const& rhs) : v0(rhs.v0), v1(rhs.v1), v2(rhs.v2), v3(rhs.v3) {} // for test
#endif

    static inline constexpr i256 max()
    {
        return i256{u64(0x7FFFFFFFFFFFFFFF), u64(0xFFFFFFFFFFFFFFFF), u64(0xFFFFFFFFFFFFFFFF), u64(0xFFFFFFFFFFFFFFFF)};
    }

    static inline constexpr i256 min()
    {
        return i256{u64(0x8000000000000000), u64(0x0000000000000000), u64(0x0000000000000000), u64(0x0000000000000000)};
    }

    constexpr size_t hash() const
    {
        size_t seed = 0;
        util::hash_combine(seed, v0);
        util::hash_combine(seed, v1);
        util::hash_combine(seed, v2);
        util::hash_combine(seed, v3);
        return seed;
    }

    explicit operator double() const;

    bool operator==(i256 const& rhs) const noexcept { return v0 == rhs.v0 && v1 == rhs.v1 && v2 == rhs.v2 && v3 == rhs.v3; }
    bool operator!=(i256 const& rhs) const noexcept { return v0 != rhs.v0 || v1 != rhs.v1 || v2 != rhs.v2 || v3 != rhs.v3; }
};


template <int bits_a, int bits_b, int bits_out>
struct impl_math;

template <int bits>
struct fixed_int_t
{
    static_assert(bits % 64 != 0, "not supported");
    using type = typename fixed_int_t<bits <= 32 ? 32 : (bits + 63) / 64 * 64>::type;
};

template <>
struct fixed_int_t<32>
{
    using type = i32;
};

template <>
struct fixed_int_t<64>
{
    using type = i64;
};

template <>
struct fixed_int_t<128>
{
    using type = i128;
};

template <>
struct fixed_int_t<192>
{
    using type = i192;
};

template <>
struct fixed_int_t<256>
{
    using type = i256;
};

template <int bits>
using fixed_int = typename fixed_int_t<bits>::type;

template <int bits_out, class A, class B>
fixed_int<bits_out> mul(A a, B b)
{
    // round to next 64bit
    auto const bits_out_rounded = bits_out <= 32 ? 32 : (bits_out + 63) / 64 * 64;
    auto const bits_a_rounded = 8 * sizeof(A);
    auto const bits_b_rounded = 8 * sizeof(B);

    if constexpr (bits_a_rounded < bits_b_rounded)
        return impl_math<bits_out_rounded, bits_b_rounded, bits_a_rounded>::mul(b, a);
    else
        return impl_math<bits_out_rounded, bits_a_rounded, bits_b_rounded>::mul(a, b);
}

/// implementation

/// helper for overflow assertions
inline bool high_is_zero(u64 a, u64 b)
{
    u64 h;
    _mulx_u64(a, b, &h);
    return h == 0;
}

/// helper for overflow detection
inline bool overflows(u64 a, u64 b, u64 c = 0, u64 d = 0)
{
    auto const s1 = a + b;
    if (s1 < a || s1 < b)
        return true;

    auto const s2 = s1 + c;
    if (s2 < s1 || s2 < c)
        return true;

    auto const s3 = s2 + d;
    if (s3 < s2 || s3 < d)
        return true;

    return false;
}

inline bool is_zero(i64 const& v) { return v == 0; }
inline bool is_zero(i128 const& v) { return v.v0 == 0 && v.v1 == 0; }
inline bool is_zero(i192 const& v) { return v.v0 == 0 && v.v1 == 0 && v.v2 == 0; }
inline bool is_zero(i256 const& v) { return v.v0 == 0 && v.v1 == 0 && v.v2 == 0 && v.v3 == 0; }

inline tg::i8 sign_of(i64 const& v) { return v < 0 ? -1 : v > 0 ? 1 : 0; }

inline tg::i8 sign_of(i128 const& v)
{
    tg::i8 neg = i64(v.v1) >> 63;
    tg::i8 zero = v.v0 == 0 & v.v1 == 0;
    return 1 - zero + 2 * neg;
}

inline tg::i8 sign_of(i192 const& v)
{
    tg::i8 neg = i64(v.v2) >> 63;
    tg::i8 zero = v.v0 == 0 & v.v1 == 0 & v.v2 == 0;
    return 1 - zero + 2 * neg;
}

inline tg::i8 sign_of(i256 const& v)
{
    tg::i8 neg = i64(v.v3) >> 63;
    tg::i8 zero = v.v0 == 0 & v.v1 == 0 & v.v2 == 0 & v.v3 == 0;
    return 1 - zero + 2 * neg;
}

template <>
struct impl_math<32, 32, 32>
{
    static i32 mul(i32 a, i32 b)
    {
        TG_ASSERT(tg::abs(i64(a) * i64(b)) <= (i64(1) << 31) && "overflow");
        return a * b;
    }
};

template <>
struct impl_math<64, 32, 32>
{
    static i64 mul(i32 a, i32 b)
    {
        // no overflow check needed
        return i64(a) * b;
    }
};

template <>
struct impl_math<64, 64, 32>
{
    static i64 mul(i64 a, i32 b)
    {
        TG_ASSERT(a == 0 || (a * b) / a == b && "overflow");
        return a * b;
    }
};

template <>
struct impl_math<64, 64, 64>
{
    static i64 mul(i64 a, i64 b)
    {
        TG_ASSERT(a == 0 || (a * b) / a == b && "overflow");
        return a * b;
    }
};

template <>
struct impl_math<128, 128, 64>
{
    static i128 mul(i128 a, i64 b)
    {
#ifndef _MSC_VER
        static_assert(sizeof(i128) == sizeof(__int128), "Must have the same size!");


        // linux:
        __int128 ha;
        __int128 hb(b);
        memcpy(&ha, &a, sizeof(__int128));
        auto const hres = ha * hb;
        i128 res;
        memcpy(&res, &hres, sizeof(i128));

        TG_ASSERT(!(ha != 0 && hres / ha != hb) && "overflow!");

        return res;
#else
        i128 b128(b);
        i128 res;
        u64 l;
        u64 h;
        l = _mulx_u64(a.v0, b128.v0, &h);
        res.v0 = l;
        res.v1 = h + i64(a.v1) * i64(b128.v0) + i64(a.v0) * i64(b128.v1);
        return res;

        // windows
//        i128 res;
//        u64 s_l = u64(i64(a.v1) >> 63); // 0 iff > 0, -1 otherwise
//        u64 s_r = u64(i64(b) >> 63);    // 0 iff > 0, -1 otherwise
//        u64 s_res = s_l ^ s_r;
//        { // conditional inversion
//            a.v0 = ((u64(a.v0) ^ s_l) - s_l);
//            u64 c0 = (a.v0 == 0) & s_l;
//            a.v1 = (u64(a.v1) ^ s_l) + c0;
//        }
//        { // conditional inversion
//            b = i64((u64(b) ^ s_r) - s_r);
//        }
//        u64 l00 = 0;
//        u64 l10 = 0;
//        u64 h00 = 0;
//        l00 = _mulx_u64(u64(a.v0), u64(b), &h00);
//        l10 = u64(a.v1) * u64(b);
//        unsigned char c = 0;
//        c += _addcarry_u64(0, res.v0, l00, &res.v0);
//        res.v1 = c + h00 + l10;
//        { // conditional inversion
//            res.v0 = ((u64(res.v0) ^ s_res) - s_res);
//            u64 c0 = (res.v0 == 0) & s_res;
//            res.v1 = (u64(res.v1) ^ s_res) + c0;
//        }
//        return res;
#endif
    }
};

template <>
struct impl_math<256, 128, 128>
{
    static i256 mul(i128 a, i128 b)
    {
        auto const ma = i64(a.v1) >> 63; // 0 iff > 0, -1 otherwise
        auto const mb = i64(b.v1) >> 63; // 0 iff > 0, -1 otherwise
        auto const mout = ma ^ mb;

        OB_NEG_CONDITIONAL_128(a.v0, a.v1, ma);
        OB_NEG_CONDITIONAL_128(b.v0, b.v1, mb);

        u64 l00, h00, l01, h01, l10, h10, l11, h11;
        l00 = _mulx_u64(a.v0, b.v0, &h00);
        l01 = _mulx_u64(a.v0, b.v1, &h01);
        l10 = _mulx_u64(a.v1, b.v0, &h10);
        l11 = _mulx_u64(a.v1, b.v1, &h11);

        // summation
        u64 v0, v1, v2, v3;
        // v0
        v0 = l00;
        // v1
        auto c1 = _addcarry_u64(0, h00, l10, &v1);
        c1 += _addcarry_u64(0, v1, l01, &v1);
        // v2
        auto c2 = _addcarry_u64(0, c1, h10, &v2);
        c2 += _addcarry_u64(0, v2, h01, &v2);
        c2 += _addcarry_u64(0, v2, l11, &v2);
        // v3
        v3 = c2 + h11;

        // note: this can not overflow!

        OB_NEG_CONDITIONAL_256(v0, v1, v2, v3, mout);

        return {v0, v1, v2, v3};
    }
};

template <>
struct impl_math<128, 128, 128>
{
    static i128 mul(i128 a, i128 b)
    {
#ifndef _MSC_VER
        // linux:
        __int128 ha;
        __int128 hb;
        memcpy(&ha, &a, sizeof(__int128));
        memcpy(&hb, &b, sizeof(__int128));
        auto const hres = ha * hb;
        i128 res;
        memcpy(&res, &hres, sizeof(i128));

        TG_ASSERT(!(ha != 0 && hres / ha != hb) && "overflow!");

        return res;

#else

        i128 res;
        u64 l;
        u64 h;
        l = _mulx_u64(a.v0, b.v0, &h);
        res.v0 = l;
        res.v1 = h + i64(a.v1) * i64(b.v0) + i64(a.v0) * i64(b.v1);
        return res;
//        i128 res;
//        i128 l = a;
//        i128 r = b;
//        u64 s_l = u64(i64(l.v1) >> 63); // 0 iff > 0, -1 otherwise
//        u64 s_r = u64(i64(r.v1) >> 63); // 0 iff > 0, -1 otherwise
//        u64 s_res = s_l ^ s_r;
//        { // conditional inversion
//            l.v0 = ((u64(l.v0) ^ s_l) - s_l);
//            u64 c0 = (l.v0 == 0) & s_l;
//            l.v1 = (u64(l.v1) ^ s_l) + c0;
//        }
//        { // conditional inversion
//            r.v0 = ((u64(r.v0) ^ s_r) - s_r);
//            u64 c0 = (r.v0 == 0) & s_r;
//            r.v1 = (u64(r.v1) ^ s_r) + c0;
//        }
//        u64 l00 = 0;
//        u64 l01 = 0;
//        u64 l10 = 0;
//        u64 h00 = 0;
//        l00 = _mulx_u64(u64(l.v0), u64(r.v0), &h00);
//        l01 = u64(l.v0) * u64(r.v1);
//        l10 = u64(l.v1) * u64(r.v0);
//        unsigned char c = 0;
//        c += _addcarry_u64(0, res.v0, l00, &res.v0);
//        res.v1 = c + h00 + l01 + l10;
//        { // conditional inversion
//            res.v0 = ((u64(res.v0) ^ s_res) - s_res);
//            u64 c0 = (res.v0 == 0) & s_res;
//            res.v1 = (u64(res.v1) ^ s_res) + c0;
//        }
//        return res;
#endif
    }
};

template <>
struct impl_math<128, 64, 64>
{
    static i128 mul(i64 a, i64 b)
    {
#ifndef _MSC_VER
        // linux:
        __int128 hpa(a);
        __int128 hpb(b);
        auto r = hpa * hpb;
        i128 r2;
        memcpy(&r2, &r, sizeof(r));

        // no overflow check needed

        return r2;
#else
        i128 res;
        i64 l = a;
        i64 r = b;
        u64 s_l = u64(i64(l) >> 63); // 0 iff > 0, -1 otherwise
        u64 s_r = u64(i64(r) >> 63); // 0 iff > 0, -1 otherwise
        u64 s_res = s_l ^ s_r;
        { // conditional inversion
            l = i64((u64(l) ^ s_l) - s_l);
        }
        { // conditional inversion
            r = i64((u64(r) ^ s_r) - s_r);
        }
        u64 l00 = 0;
        u64 h00 = 0;
        l00 = _mulx_u64(u64(l), u64(r), &h00);
        unsigned char c = 0;
        c += _addcarry_u64(0, res.v0, l00, &res.v0);
        res.v1 = c + h00;
        { // conditional inversion
            res.v0 = ((u64(res.v0) ^ s_res) - s_res);
            u64 c0 = (res.v0 == 0) & s_res;
            res.v1 = (u64(res.v1) ^ s_res) + c0;
        }
        return res;
#endif
    }
};

template <>
struct impl_math<128, 64, 32>
{
    // todo: maybe better impl
    static i128 mul(i64 a, i32 b) { return impl_math<128, 64, 64>::mul(a, b); }
};

template <>
struct impl_math<128, 128, 32>
{
    // todo: maybe better impl
    static i128 mul(i128 a, i32 b) { return impl_math<128, 128, 64>::mul(a, b); }
};

template <>
struct impl_math<192, 128, 128>
{
    static i192 mul(i128 a, i128 b)
    {
        auto const ma = i64(a.v1) >> 63; // 0 iff > 0, -1 otherwise
        auto const mb = i64(b.v1) >> 63; // 0 iff > 0, -1 otherwise
        auto const mout = ma ^ mb;

        OB_NEG_CONDITIONAL_128(a.v0, a.v1, ma);
        OB_NEG_CONDITIONAL_128(b.v0, b.v1, mb);

        // multiplications
        u64 l00, h00, l01, h01, l10, h10, l11;
        l00 = _mulx_u64(a.v0, b.v0, &h00);
        l01 = _mulx_u64(a.v0, b.v1, &h01);
        l10 = _mulx_u64(a.v1, b.v0, &h10);
        l11 = a.v1 * b.v1;

        // summations
        u64 v0, v1, v2;
        // v0
        v0 = l00;

        // v1
        auto c = _addcarry_u64(0, h00, l10, &v1);
        c += _addcarry_u64(0, v1, l01, &v1);

        // v2
        v2 = u64(c) + l11 + h10 + h01;

        TG_ASSERT(high_is_zero(a.v1, b.v1) && "overflow");
        TG_ASSERT(!overflows(c, l11, h10, h01) && "overflow");

        OB_NEG_CONDITIONAL_192(v0, v1, v2, mout);

        return i192{v0, v1, v2};
    }
};

template <>
struct impl_math<192, 128, 64>
{
    static i192 mul(i128 a, i64 b)
    {
        auto const ma = i64(a.v1) >> 63; // 0 iff > 0, -1 otherwise
        auto const mb = i64(b) >> 63;    // 0 iff > 0, -1 otherwise
        auto const mout = ma ^ mb;

        OB_NEG_CONDITIONAL_128(a.v0, a.v1, ma);
        OB_NEG_CONDITIONAL_64(b, mb);

        // unsigned multiplication
        u64 v0, v1, v2, carry2;
        v0 = _mulx_u64(a.v0, u64(b), &carry2);
        v1 = carry2 + _mulx_u64(a.v1, u64(b), &v2);
        v2 += v1 < carry2;

        // no overflow possible

        OB_NEG_CONDITIONAL_192(v0, v1, v2, mout);

        return i192{v0, v1, v2};
    }
};

template <>
struct impl_math<256, 192, 64>
{
    static i256 mul(i192 a, i64 b)
    {
        auto const ma = i64(a.v2) >> 63; // 0 iff > 0, -1 otherwise
        auto const mb = i64(b) >> 63;    // 0 iff > 0, -1 otherwise
        auto const mout = ma ^ mb;

        OB_NEG_CONDITIONAL_192(a.v0, a.v1, a.v2, ma);
        OB_NEG_CONDITIONAL_64(b, mb);

        // multiplication
        u64 l0, h0, l1, h1, l2, h2;
        l0 = _mulx_u64(a.v0, u64(b), &h0);
        l1 = _mulx_u64(a.v1, u64(b), &h1);
        l2 = _mulx_u64(a.v2, u64(b), &h2);

        // summation
        u64 v0, v1, v2, v3;
        v0 = l0;
        auto c = _addcarry_u64(0, h0, l1, &v1);
        c = _addcarry_u64(c, h1, l2, &v2);
        v3 = h2 + c;

        // no overflow possible

        OB_NEG_CONDITIONAL_256(v0, v1, v2, v3, mout);

        return {v0, v1, v2, v3};
    }
};

template <>
struct impl_math<256, 192, 128>
{
    static i256 mul(i192 a, i128 b)
    {
        // invert negatives
        auto const ma = i64(a.v2) >> 63; // 0 iff > 0, -1 otherwise
        auto const mb = i64(b.v1) >> 63; // 0 iff > 0, -1 otherwise
        auto const mout = ma ^ mb;

        OB_NEG_CONDITIONAL_192(a.v0, a.v1, a.v2, ma);
        OB_NEG_CONDITIONAL_128(b.v0, b.v1, mb);

        // multiply
        u64 h00, l00, h10, l10, h01, l01, h20, l20, h11, l11, l21;
        l00 = _mulx_u64(a.v0, b.v0, &h00);
        l10 = _mulx_u64(a.v1, b.v0, &h10);
        l01 = _mulx_u64(a.v0, b.v1, &h01);
        l20 = _mulx_u64(a.v2, b.v0, &h20);
        l11 = _mulx_u64(a.v1, b.v1, &h11);
        l21 = a.v2 * b.v1;

        // summation
        u64 v0, v1, v2, v3;
        // v0
        v0 = l00;
        // v1
        auto c1 = _addcarry_u64(0, h00, l10, &v1);
        c1 += _addcarry_u64(0, v1, l01, &v1);
        // v2
        auto c2 = _addcarry_u64(0, h10, l20, &v2);
        c2 += _addcarry_u64(0, v2, h01, &v2);
        c2 += _addcarry_u64(0, v2, l11, &v2);
        c2 += _addcarry_u64(0, v2, c1, &v2);
        // v3
        v3 = h20 + h11 + l21 + c2;

        TG_ASSERT(!overflows(h20, h11, l21, c2) && "overflow");
        TG_ASSERT(high_is_zero(a.v2, b.v1) && "overflow");

        OB_NEG_CONDITIONAL_256(v0, v1, v2, v3, mout);

        return {v0, v1, v2, v3};
    }
};

template <>
struct impl_math<256, 256, 64>
{
    static i256 mul(i256 a, i64 b)
    {
        // invert negatives
        auto const ma = i64(a.v3) >> 63; // 0 iff > 0, -1 otherwise
        auto const mb = i64(b) >> 63;    // 0 iff > 0, -1 otherwise
        auto const mout = ma ^ mb;

        OB_NEG_CONDITIONAL_256(a.v0, a.v1, a.v2, a.v3, ma);
        OB_NEG_CONDITIONAL_64(b, mb);

        // multiply
        u64 h00, l00, h10, l10, h20, l20, l30;

        l00 = _mulx_u64(a.v0, u64(b), &h00);
        l10 = _mulx_u64(a.v1, u64(b), &h10);
        l20 = _mulx_u64(a.v2, u64(b), &h20);
        l30 = a.v3 * u64(b);

        // summation
        u64 v0, v1, v2, v3;
        // v0
        v0 = l00;

        // v1
        auto c = _addcarry_u64(0, h00, l10, &v1);

        // v2
        c = _addcarry_u64(c, h10, l20, &v2);

        // v3
        _addcarry_u64(c, h20, l30, &v3);

        TG_ASSERT(!overflows(c, h20, l30) && "overflow");
        TG_ASSERT(high_is_zero(a.v3, u64(b)) && "overflow");

        OB_NEG_CONDITIONAL_256(v0, v1, v2, v3, mout);

        return {v0, v1, v2, v3};
    }
};

inline i128 add_i128_i128_i128(i128 const& a, i128 const& b)
{
    u64 h, l;
    auto c = _addcarry_u64(0, a.v0, b.v0, &l);
    c = _addcarry_u64(c, a.v1, b.v1, &h);

    TG_ASSERT((sign_of(a) != sign_of(b)) || sign_of(i128{l, h}) == sign_of(a) && "overflow");

    return i128{l, h};
}

// either of the two
inline i128 sub_i128_i128_i128(i128 const& a, i128 const& b)
{
    u64 h, l;
    auto c = _subborrow_u64(0, a.v0, b.v0, &l);
    c = _subborrow_u64(c, a.v1, b.v1, &h);

    TG_ASSERT(!is_zero(a) || b != i128::min() && "overflow");
    TG_ASSERT(is_zero(a) || sign_of(a) == sign_of(b) || sign_of(i128{l, h}) == sign_of(a) && "overflow");

    return i128{l, h};
}

inline i128 negate_i128_i128(i128 const& v)
{
    TG_ASSERT(v.v1 != (u64(1) << 63) || v.v0 != 0 && "overflow");

    u64 v0, v1;
    v0 = ~v.v0 + 1;
    v1 = ~v.v1;
    v1 += v0 == 0;

    return i128{v0, v1};
}

inline i192 add_i192_i192_i192(i192 const& a, i192 const& b)
{
    u64 v0, v1, v2;
    auto c = _addcarry_u64(0, a.v0, b.v0, &v0);
    c = _addcarry_u64(c, a.v1, b.v1, &v1);
    c = _addcarry_u64(c, a.v2, b.v2, &v2);

    TG_ASSERT((sign_of(a) != sign_of(b)) || sign_of(i192{v0, v1, v2}) == sign_of(a) && "overflow");

    return i192{v0, v1, v2};
}

// either of the two
inline i192 sub_i192_i192_i192(i192 const& a, i192 const& b)
{
    u64 v0, v1, v2;
    auto c = _subborrow_u64(0, a.v0, b.v0, &v0);
    c = _subborrow_u64(c, a.v1, b.v1, &v1);
    c = _subborrow_u64(c, a.v2, b.v2, &v2);

    TG_ASSERT(!is_zero(a) || b != i192::min() && "overflow");
    TG_ASSERT(is_zero(a) || sign_of(a) == sign_of(b) || sign_of(i192{v0, v1, v2}) == sign_of(a) && "overflow");

    return i192{v0, v1, v2};
}

inline i192 negate_i192_i192(i192 const& v)
{
    TG_ASSERT(v.v2 != (u64(1) << 63) || v.v1 != 0 || v.v0 != 0 && "overflow");

    u64 v0, v1, v2;
    v0 = ~v.v0 + 1;
    v1 = ~v.v1;
    v2 = ~v.v2;
    u64 c0 = v0 == 0;
    v1 += c0;
    u64 c1 = (v1 == 0) & c0;
    v2 += c1;

    return i192{v0, v1, v2};
}

inline i256 add_i256_i256_i256(i256 const& a, i256 const& b)
{
    u64 v0, v1, v2, v3;
    auto c = _addcarry_u64(0, a.v0, b.v0, &v0);
    c = _addcarry_u64(c, a.v1, b.v1, &v1);
    c = _addcarry_u64(c, a.v2, b.v2, &v2);
    c = _addcarry_u64(c, a.v3, b.v3, &v3);

    TG_ASSERT((sign_of(a) != sign_of(b)) || sign_of(i256{v0, v1, v2, v3}) == sign_of(a) && "overflow");

    return {v0, v1, v2, v3};
}

inline i256 sub_i256_i256_i256(i256 const& a, i256 const& b)
{
    u64 v0, v1, v2, v3;
    auto c = _subborrow_u64(0, a.v0, b.v0, &v0);
    c = _subborrow_u64(c, a.v1, b.v1, &v1);
    c = _subborrow_u64(c, a.v2, b.v2, &v2);
    c = _subborrow_u64(c, a.v3, b.v3, &v3);

    TG_ASSERT(!is_zero(a) || b != i256::min() && "overflow");
    TG_ASSERT(is_zero(a) || sign_of(a) == sign_of(b) || sign_of(i256{v0, v1, v2, v3}) == sign_of(a) && "overflow");

    return {v0, v1, v2, v3};
}

inline i256 negate_i256_i256(i256 const& v)
{
    TG_ASSERT(v.v3 != (u64(1) << 63) || v.v2 != 0 || v.v1 != 0 || v.v0 != 0 && "overflow");

    u64 v0, v1, v2, v3;
    v0 = ~v.v0 + 1;
    v1 = ~v.v1;
    v2 = ~v.v2;
    v3 = ~v.v3;
    u64 c0 = v0 == 0;
    v1 += c0;
    u64 c1 = (v1 == 0) & c0;
    v2 += c1;
    u64 c2 = (v2 == 0) & c1;
    v3 += c2;
    return {v0, v1, v2, v3};
}

inline i128 operator+(i128 const& lhs, i128 const& rhs) { return add_i128_i128_i128(lhs, rhs); }

inline i128 operator-(i128 const& lhs, i128 const& rhs) { return sub_i128_i128_i128(lhs, rhs); }

inline i128 operator-(i128 const& v) { return negate_i128_i128(v); }

inline i128& operator+=(i128& lhs, i128 const& rhs)
{
    lhs = lhs + rhs;
    return lhs;
}

inline i128& operator-=(i128& lhs, i128 const& rhs)
{
    lhs = lhs - rhs;
    return lhs;
}

inline i192 operator+(i192 const& lhs, i192 const& rhs) { return add_i192_i192_i192(lhs, rhs); }

inline i192 operator-(i192 const& lhs, i192 const& rhs) { return sub_i192_i192_i192(lhs, rhs); }

inline i192 operator-(i192 const& v) { return negate_i192_i192(v); }

inline i192& operator+=(i192& lhs, i192 const& rhs)
{
    lhs = lhs + rhs;
    return lhs;
}

inline i192& operator-=(i192& lhs, i192 const& rhs)
{
    lhs = lhs - rhs;
    return lhs;
}

inline i256 operator+(i256 const& lhs, i256 const& rhs) { return add_i256_i256_i256(lhs, rhs); }

inline i256 operator-(i256 const& lhs, i256 const& rhs) { return sub_i256_i256_i256(lhs, rhs); }

inline i256 operator-(i256 const& v) { return negate_i256_i256(v); }

inline i256& operator+=(i256& lhs, i256 const& rhs)
{
    lhs = lhs + rhs;
    return lhs;
}

inline i256& operator-=(i256& lhs, i256 const& rhs)
{
    lhs = lhs - rhs;
    return lhs;
}

inline ob::i128::operator double() const
{
    //    __int128 tmp;
    //    memcpy(&tmp, this, sizeof(tmp));
    //    return double(tmp);

    auto c = *this;
    auto const neg = i64(v1) < 0;
    if (neg)
        c = negate_i128_i128(*this);

    return (1 - 2 * neg) * (double(c.v0) + double(c.v1) * 0x1p64);
}

inline ob::i192::operator double() const
{
    auto c = *this;
    auto const neg = i64(v2) < 0;
    if (neg)
        c = negate_i192_i192(*this);

    return (1 - 2 * neg)                 // sign
           * (double(c.v0)               // least significant bits
              + double(c.v1) * 0x1p64    // center bits
              + double(c.v2) * 0x1p128); // most significant bits
}

inline ob::i256::operator double() const
{
    auto c = *this;
    auto const neg = i64(v3) < 0;
    if (neg)
        c = negate_i256_i256(*this);

    return (1 - 2 * neg)                 // sign
           * (double(c.v0)               // least significant bits
              + double(c.v1) * 0x1p64    // center bits
              + double(c.v2) * 0x1p128   // center bits
              + double(c.v3) * 0x1p192); // most significant bits
}

inline ob::i128 operator<<(ob::i128 const& lhs, int rhs)
{
    TG_ASSERT(rhs >= 0);
    TG_ASSERT(rhs <= 64); // for now

    u64 v0, v1;
    // mask for all bits that need to move from one word to the other
    u64 mask = u64(-1) << (64 - rhs);
    // get the bits that are moved from the less significant word to the more significant one
    u64 copy_bits = lhs.v0 & mask;
    // shift them to the correct position
    copy_bits >>= (64 - rhs);
    // shift of the least significant word
    v0 = lhs.v0 << rhs;
    // shift of the most significant word
    v1 = lhs.v1 << rhs;
    // copy the bits accordingly
    v1 |= copy_bits;
    return ob::i128{v0, v1};
}
inline bool greater_than_zero(i64 const& v) { return v > 0; }

inline bool greater_than_zero(i128 const& v) { return i64(v.v1) > 0 || (i64(v.v1) == 0 && i64(v.v0) != 0); }

inline bool less_than_zero(i64 const& v) { return v < 0; }

inline bool less_than_zero(i128 const& v) { return i64(v.v1) < 0; }

inline bool less_than_zero(i192 const& v) { return i64(v.v2) < 0; }

inline bool less_than_zero(i256 const& v) { return i64(v.v3) < 0; }

/// zero literal for optimized zero test
template <class value_t>
struct non_zero
{
    value_t value;

    non_zero(value_t v) : value(v) {}
};

/// called with rhs == 0
inline bool operator<=(i128 const& lhs, void*) { return less_than_zero(lhs) || is_zero(lhs); }
/// called with rhs != 0
inline bool operator<=(i128 const& lhs, non_zero<i128> const& rhs) { return (lhs - rhs.value) <= 0; }

/// called with rhs == 0
inline bool operator>=(i128 const& lhs, void*) { return !less_than_zero(lhs); }
/// called with rhs != 0
inline bool operator>=(i128 const& lhs, non_zero<i128> const& rhs) { return (lhs - rhs.value) >= 0; }

inline bool operator<=(i128 const& lhs, i128 const& rhs)
{
    auto const diff = lhs - rhs;
    return less_than_zero(diff) || is_zero(diff);
}

inline bool operator>=(i128 const& lhs, i128 const& rhs)
{
    auto const diff = lhs - rhs;
    return greater_than_zero(diff) || is_zero(diff);
}

inline bool operator<(i128 const& lhs, void*) { return less_than_zero(lhs); }

inline bool operator>(i128 const& lhs, void*) { return greater_than_zero(lhs); }

inline bool operator<(i128 const& lhs, i128 const& rhs)
{
    auto const diff = lhs - rhs;
    return less_than_zero(diff);
}

inline bool operator>(i128 const& lhs, i128 const& rhs)
{
    auto const diff = rhs - lhs;
    return less_than_zero(diff);
}

inline i128 abs(i128 const& v) { return less_than_zero(v) ? -v : v; }

inline i192 abs(i192 const& v) { return less_than_zero(v) ? -v : v; }

inline i256 abs(i256 const& v) { return less_than_zero(v) ? -v : v; }

inline i64 abs(i64 v) { return std::abs(v); }

inline i64 min(i64 a, i64 b) { return a < b ? a : b; }
inline i64 max(i64 a, i64 b) { return a < b ? b : a; }

inline i128 min(i128 const& a, i128 const& b)
{
    if (less_than_zero(a - b))
        return a;
    else
        return b;
}

inline i128 max(i128 const& a, i128 const& b)
{
    if (less_than_zero(a - b))
        return b;
    else
        return a;
}

inline i192 min(i192 const& a, i192 const& b)
{
    if (less_than_zero(a - b))
        return a;
    else
        return b;
}

inline i192 max(i192 const& a, i192 const& b)
{
    if (less_than_zero(a - b))
        return b;
    else
        return a;
}


inline i256 min(i256 const& a, i256 const& b)
{
    if (less_than_zero(a - b))
        return a;
    else
        return b;
}

inline i256 max(i256 const& a, i256 const& b)
{
    if (less_than_zero(a - b))
        return b;
    else
        return a;
}

std::string to_string(i64 val);
std::string to_string(i128 val);
std::string to_string(i192 val);
std::string to_string(i256 val);

std::ostream& operator<<(std::ostream& os, i128 const& v);
std::ostream& operator<<(std::ostream& os, i192 const& v);
std::ostream& operator<<(std::ostream& os, i256 const& v);

} // namespace ob

// make types tg scalars
namespace tg
{
template <>
struct is_scalar_t<ob::i128>
{
    static constexpr bool value = true;
};

template <>
struct is_scalar_t<ob::i192>
{
    static constexpr bool value = true;
};

template <>
struct is_scalar_t<ob::i256>
{
    static constexpr bool value = true;
};
} // namespace tg

// std::hash
namespace std
{
template <>
struct hash<ob::i128>
{
    size_t operator()(ob::i128 const& v) const noexcept { return v.hash(); }
};

template <>
struct hash<ob::i192>
{
    size_t operator()(ob::i192 const& v) const noexcept { return v.hash(); }
};

template <>
struct hash<ob::i256>
{
    size_t operator()(ob::i256 const& v) const noexcept { return v.hash(); }
};

} // namespace std
