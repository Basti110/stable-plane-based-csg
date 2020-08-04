#pragma once
#include <optional>
#include <functional>
#include <integer_math.hh>
#include <typed-geometry/tg.hh>

namespace ob
{
template <class geometry_t>
struct plane;

template <class geometry_t>
struct subdeterminants;

template <int bits_pos_t, int bits_normal_t>
struct geometry
{
    static inline constexpr int bits_position = bits_pos_t;
    static inline constexpr int bits_normal = bits_normal_t;
    static inline constexpr int bits_edge = (bits_normal - 1) / 2;
    static inline constexpr int bits_plane_d = bits_position + bits_normal + 2;
    static inline constexpr int bits_determinant_abc = 3 * bits_normal + 3;
    static inline constexpr int bits_determinant_xxd = 2 * bits_normal + bits_plane_d + 3;

    using pos_scalar_t = fixed_int<bits_position>;
    using pos_t = tg::pos<3, pos_scalar_t>;
    using vec_t = tg::vec<3, pos_scalar_t>;
    using normal_scalar_t = fixed_int<bits_normal>;
    using plane_d_t = fixed_int<bits_plane_d>;
    using subdeterminants_t = subdeterminants<geometry>;
    using determinant_abc_t = fixed_int<bits_determinant_abc>;
    using determinant_xxd_t = fixed_int<bits_determinant_xxd>;
    using plane_t = plane<geometry>;
    using determinants_t = subdeterminants<geometry>;
    using aabb_t = tg::aabb<3, pos_scalar_t>;

    static tg::dpos3 to_dpos3(pos_scalar_t const& a, pos_scalar_t const& b, pos_scalar_t const& c) { return {double(a), double(b), double(c)}; }
};

using geometry256_x64_n45 = geometry<64, 45>;
using geometry128_x32_n21 = geometry<32, 21>;
using geometry256_x48_n49 = geometry<48, 49>;
using geometry256_x27_n55 = geometry<27, 55>;

template <class GeometryT>
struct plane
{
    using geometry_t = GeometryT;
    using normal_scalar_t = typename geometry_t::normal_scalar_t;
    using distance_t = typename geometry_t::plane_d_t;
    using pos_t = typename geometry_t::pos_t;
    using vec_t = tg::vec<3, typename geometry_t::pos_scalar_t>;

    normal_scalar_t a, b, c;
    distance_t d; // is plane equation dnormal_scalar_t

    void translate(vec_t v)
    {
        d = d - mul<8 * sizeof(distance_t)>(a, v.x);
        d = d - mul<8 * sizeof(distance_t)>(b, v.y);
        d = d - mul<8 * sizeof(distance_t)>(c, v.z);
    }

    static plane from_pos_normal(pos_t p, vec_t n)
    {
        TG_ASSERT(tg::abs(n.x) <= (i64(1) << geometry_t::bits_normal));
        TG_ASSERT(tg::abs(n.y) <= (i64(1) << geometry_t::bits_normal));
        TG_ASSERT(tg::abs(n.z) <= (i64(1) << geometry_t::bits_normal));

        auto const d = mul<8 * sizeof(distance_t)>(-n.x, p.x) + //
                       mul<8 * sizeof(distance_t)>(-n.y, p.y) + //
                       mul<8 * sizeof(distance_t)>(-n.z, p.z);

        return {n.x, n.y, n.z, d};
    }

    static plane from_points(pos_t p0, pos_t p1, pos_t p2)
    {
        // no high precision needed, as the max edge length limits what can come out here
        auto n = cross(p1 - p0, p2 - p0);

        auto const f = tg::gcd(tg::gcd(tg::abs(n.x), tg::abs(n.y)), tg::abs(n.z));

        if (f > 1)
            n /= f;

        // these assertions only work as long as the normal is less than 64 bit (for now)
        TG_ASSERT(tg::abs(n.x) <= (i64(1) << geometry_t::bits_normal));
        TG_ASSERT(tg::abs(n.y) <= (i64(1) << geometry_t::bits_normal));
        TG_ASSERT(tg::abs(n.z) <= (i64(1) << geometry_t::bits_normal));

        //        auto const d = -dot(n, p0);
        auto const d = mul<8 * sizeof(distance_t)>(-n.x, p0.x) + //
                       mul<8 * sizeof(distance_t)>(-n.y, p0.y) + //
                       mul<8 * sizeof(distance_t)>(-n.z, p0.z);

        //        TG_ASSERT(tg::abs(d) <= (1 << (geometry128::bits_normal + geometry128::bits_pos)));

        return {n.x, n.y, n.z, d};
    }

    static plane from_points_no_gcd(pos_t p0, pos_t p1, pos_t p2)
    {
        // no high precision needed, as the max edge length limits what can come out here
        auto n = cross(p1 - p0, p2 - p0);

        // these assertions only work as long as the normal is less than 64 bit (for now)
        TG_ASSERT(tg::abs(n.x) <= (i64(1) << geometry_t::bits_normal));
        TG_ASSERT(tg::abs(n.y) <= (i64(1) << geometry_t::bits_normal));
        TG_ASSERT(tg::abs(n.z) <= (i64(1) << geometry_t::bits_normal));

        //        auto const d = -dot(n, p0);
        auto const d = mul<8 * sizeof(distance_t)>(-n.x, p0.x) + //
                       mul<8 * sizeof(distance_t)>(-n.y, p0.y) + //
                       mul<8 * sizeof(distance_t)>(-n.z, p0.z);

        //        TG_ASSERT(tg::abs(d) <= (1 << (geometry128::bits_normal + geometry128::bits_pos)));

        return {n.x, n.y, n.z, d};
    }

    tg::dplane3 to_dplane() const
    {
        auto const il = 1. / length(tg::dvec3(a, b, c));
        return {{a * il, b * il, c * il}, -double(d) * il};
    }

    size_t hash() const noexcept
    {
        // see https://www.boost.org/doc/libs/1_35_0/doc/html/hash/combine.html
        auto const combine = [](size_t& seed, size_t value) {
            seed ^= value + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            return seed;
        };

        size_t seed = 0;
        size_t const hx(std::hash<normal_scalar_t>{}(a));
        size_t const hy(std::hash<normal_scalar_t>{}(b));
        size_t const hz(std::hash<normal_scalar_t>{}(c));
        size_t const hl(std::hash<distance_t>{}(d));

        combine(seed, hx);
        combine(seed, hy);
        combine(seed, hz);
        combine(seed, hl);

        return seed;
    }

    bool is_valid() const { return !is_zero(a) || !is_zero(b) || !is_zero(c); }

    bool constexpr operator==(plane const& rhs) const noexcept { return a == rhs.a && b == rhs.b && c == rhs.c && d == rhs.d; }
};

template <class geometry_t>
bool are_parallel(plane<geometry_t> const& p0, plane<geometry_t> const& p1)
{
    static constexpr int out_bits = 1 + 2 * geometry_t::bits_normal;
    // cross product
    auto const crossa = mul<out_bits>(p0.b, p1.c) - mul<out_bits>(p0.c, p1.b);
    auto const crossb = mul<out_bits>(p0.c, p1.a) - mul<out_bits>(p0.a, p1.c);
    auto const crossc = mul<out_bits>(p0.a, p1.b) - mul<out_bits>(p0.b, p1.a);

    // all zero
    return is_zero(crossa) && is_zero(crossb) && is_zero(crossc);
}

template <class geometry_t>
auto signed_distance(plane<geometry_t> const& plane, typename geometry_t::pos_t const& point)
{
    // dot of normal and point plus d
    return ob::mul<geometry_t::bits_plane_d>(plane.a, point.x) + //
           ob::mul<geometry_t::bits_plane_d>(plane.b, point.y) + //
           ob::mul<geometry_t::bits_plane_d>(plane.c, point.z) + // bits_determinant_xxd
           plane.d;
}

template <class geometry_t> // TODO
std::optional<typename double> intersection_distance(plane<geometry_t> const& plane, typename geometry_t::pos_t const& ray_origin, tg::vec3 const& ray_dir)
{
    tg::dvec3 plane_norm = -tg::dvec3(plane.a, plane.b, plane.c);
    plane_norm = tg::normalize(plane_norm);
    //std::cout << "i " << -plane.d << std::endl;
    //std::cout << "d " << plane.to_dplane().dis << std::endl;


    //plane.to_dplane().dis
    double denom = tg::dot(plane_norm, tg::dvec3(ray_dir));
    if (denom > 0) {
        //TODO double to scalar_t
        auto const il = 1. / length(tg::dvec3(plane.a, plane.b, plane.c));
        auto len = double(plane.d) * il;
        tg::dvec3 p0l0 = (plane_norm * len) - tg::dvec3(ray_origin);
        double t = (tg::dot(p0l0, plane_norm) / denom);
        if (t > 0)
            return t;
        else
            return -1;
    }

    return std::nullopt;
}

template <class GeometryT>
struct subdeterminants
{
    using geometry_t = GeometryT;
    using det_abc_t = typename geometry_t::determinant_abc_t;
    using det_xxd_t = typename geometry_t::determinant_xxd_t;

    det_abc_t det_abc;
    det_xxd_t det_abd;
    det_xxd_t det_acd;
    det_xxd_t det_bcd;

    bool is_valid() const { return !is_zero(det_abc); }
};

template <class geometry_t>
inline tg::dpos3 to_position(subdeterminants<geometry_t> const& subs)
{
    auto const x = -double(subs.det_bcd);
    auto const y = double(subs.det_acd);
    auto const z = -double(subs.det_abd);
    auto const iw = 1 / double(subs.det_abc);
    TG_ASSERT(!is_zero(subs.det_abc));
    TG_ASSERT(tg::is_finite(float(x * iw)));
    TG_ASSERT(tg::is_finite(float(y * iw)));
    TG_ASSERT(tg::is_finite(float(z * iw)));
    return {x * iw, y * iw, z * iw};
}

template <class geometry_t>
void compute_subdeterminants(plane<geometry_t> const& p, //
                             plane<geometry_t> const& q,
                             plane<geometry_t> const& r,
                             subdeterminants<geometry_t>& subs)
{
    using normal_scalar_t = typename geometry_t::normal_scalar_t;
    using plane_d_t = typename geometry_t::plane_d_t;

    auto constexpr old_computation = false;

    if constexpr (old_computation)
    {
        auto const det_normal = [](normal_scalar_t a, normal_scalar_t b, normal_scalar_t c, //
                                   normal_scalar_t d, normal_scalar_t e, normal_scalar_t f, //
                                   normal_scalar_t g, normal_scalar_t h, normal_scalar_t i) -> fixed_int<geometry_t::bits_determinant_abc> {
            auto const s0 = mul<geometry_t::bits_determinant_abc>(mul<2 * geometry_t::bits_normal>(a, e), i);
            auto const s1 = mul<geometry_t::bits_determinant_abc>(mul<2 * geometry_t::bits_normal>(b, f), g);
            auto const s2 = mul<geometry_t::bits_determinant_abc>(mul<2 * geometry_t::bits_normal>(c, d), h);
            auto const s3 = mul<geometry_t::bits_determinant_abc>(mul<2 * geometry_t::bits_normal>(c, e), g);
            auto const s4 = mul<geometry_t::bits_determinant_abc>(mul<2 * geometry_t::bits_normal>(b, d), i);
            auto const s5 = mul<geometry_t::bits_determinant_abc>(mul<2 * geometry_t::bits_normal>(a, f), h);

            return s0 + s1 + s2 - s3 - s4 - s5;
        };

        auto const det_mixed = [](normal_scalar_t a, normal_scalar_t b, plane_d_t c, //
                                  normal_scalar_t d, normal_scalar_t e, plane_d_t f, //
                                  normal_scalar_t g, normal_scalar_t h, plane_d_t i) -> fixed_int<geometry_t::bits_determinant_xxd> {
            auto const s0 = mul<geometry_t::bits_determinant_xxd>(mul<2 * geometry_t::bits_normal>(a, e), i);
            auto const s1 = mul<geometry_t::bits_determinant_xxd>(mul<2 * geometry_t::bits_normal>(b, g), f);
            auto const s2 = mul<geometry_t::bits_determinant_xxd>(mul<2 * geometry_t::bits_normal>(h, d), c);
            auto const s3 = mul<geometry_t::bits_determinant_xxd>(mul<2 * geometry_t::bits_normal>(g, e), c);
            auto const s4 = mul<geometry_t::bits_determinant_xxd>(mul<2 * geometry_t::bits_normal>(b, d), i);
            auto const s5 = mul<geometry_t::bits_determinant_xxd>(mul<2 * geometry_t::bits_normal>(a, h), f);

            return s0 + s1 + s2 - s3 - s4 - s5;
        };

        subs.det_abc = det_normal(p.a, p.b, p.c, //
                                  q.a, q.b, q.c, //
                                  r.a, r.b, r.c);

        subs.det_abd = det_mixed(p.a, p.b, p.d, //
                                 q.a, q.b, q.d, //
                                 r.a, r.b, r.d);

        subs.det_acd = det_mixed(p.a, p.c, p.d, //
                                 q.a, q.c, q.d, //
                                 r.a, r.c, r.d);

        subs.det_bcd = det_mixed(p.b, p.c, p.d, //
                                 q.b, q.c, q.d, //
                                 r.b, r.c, r.d);
    }

    // new version
    {
        auto constexpr bits_det_2x2_xx = geometry_t::bits_normal * 2;
        auto constexpr bits_det_2x2_xd = geometry_t::bits_normal + geometry_t::bits_plane_d;
        auto constexpr bits_det_abc = geometry_t::bits_determinant_abc;
        auto constexpr bits_det_xxd = geometry_t::bits_determinant_xxd;

        auto const det_2x2_ab = mul<bits_det_2x2_xx>(p.a, q.b) - mul<bits_det_2x2_xx>(p.b, q.a);
        auto const det_2x2_ac = mul<bits_det_2x2_xx>(p.a, q.c) - mul<bits_det_2x2_xx>(p.c, q.a);
        auto const det_2x2_ad = mul<bits_det_2x2_xd>(p.a, q.d) - mul<bits_det_2x2_xd>(p.d, q.a);
        auto const det_2x2_bc = mul<bits_det_2x2_xx>(p.b, q.c) - mul<bits_det_2x2_xx>(p.c, q.b);
        auto const det_2x2_bd = mul<bits_det_2x2_xd>(p.b, q.d) - mul<bits_det_2x2_xd>(p.d, q.b);
        auto const det_2x2_cd = mul<bits_det_2x2_xd>(p.c, q.d) - mul<bits_det_2x2_xd>(p.d, q.c);

        auto const det_abc = mul<bits_det_abc>(det_2x2_ab, r.c) - //
                             mul<bits_det_abc>(det_2x2_ac, r.b) + //
                             mul<bits_det_abc>(det_2x2_bc, r.a);

        auto const det_abd = mul<bits_det_xxd>(det_2x2_ab, r.d) - //
                             mul<bits_det_xxd>(det_2x2_ad, r.b) + //
                             mul<bits_det_xxd>(det_2x2_bd, r.a);

        auto const det_acd = mul<bits_det_xxd>(det_2x2_ac, r.d) - //
                             mul<bits_det_xxd>(det_2x2_ad, r.c) + //
                             mul<bits_det_xxd>(det_2x2_cd, r.a);

        auto const det_bcd = mul<bits_det_xxd>(det_2x2_cd, r.b) - //
                             mul<bits_det_xxd>(det_2x2_bd, r.c) + //
                             mul<bits_det_xxd>(det_2x2_bc, r.d);

        if constexpr (old_computation)
        {
            TG_ASSERT(det_abc == subs.det_abc);
            TG_ASSERT(det_abd == subs.det_abd);
            TG_ASSERT(det_acd == subs.det_acd);
            TG_ASSERT(det_bcd == subs.det_bcd);
        }
        else
        {
            subs.det_abc = det_abc;
            subs.det_abd = det_abd;
            subs.det_acd = det_acd;
            subs.det_bcd = det_bcd;
        }
    }
}

template <class geometry_t>
tg::i8 classify_vertex(typename geometry_t::determinant_abc_t const& det_abc, //
                       typename geometry_t::determinant_xxd_t const& det_abd,
                       typename geometry_t::determinant_xxd_t const& det_acd,
                       typename geometry_t::determinant_xxd_t const& det_bcd,
                       plane<geometry_t> const& s)
{
    // ld(3) = 2 summations plus maximal bits of multiplication.
    static constexpr int max_bits = 2 + geometry_t::bits_determinant_xxd + geometry_t::bits_normal;

    auto const d = mul<max_bits>(det_acd, s.b)   //
                   + mul<max_bits>(det_abc, s.d) //
                   - mul<max_bits>(det_bcd, s.a) //
                   - mul<max_bits>(det_abd, s.c);

    return sign_of(d) * sign_of(det_abc);
}

template <class geometry_t>
tg::i8 classify_vertex(subdeterminants<geometry_t> const& dets, plane<geometry_t> const& p)
{
    return classify_vertex(dets.det_abc, dets.det_abd, dets.det_acd, dets.det_bcd, p);
}

template <class geometry_t>
tg::dpos3 compute_intersection(plane<geometry_t> const& p, //
                               plane<geometry_t> const& q,
                               plane<geometry_t> const& r)
{
    return intersection(p.to_dplane(), q.to_dplane(), r.to_dplane());
}
} // namespace ob

namespace std
{
template <>
struct hash<ob::plane<ob::geometry256_x64_n45>>
{
    size_t operator()(ob::plane<ob::geometry256_x64_n45> const& plane) const noexcept { return plane.hash(); }
};
} // namespace std
