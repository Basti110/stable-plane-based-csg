#pragma once

#include <integer_math.hh>
#include <typed-geometry/tg.hh>
#include <polymesh/pm.hh>
#include <geometry.hh>
#include <plane_polygon.hh>
#include <intersection.hh>
#include <glow-extras/viewer/view.hh>

namespace ob {
    enum class intersection_result
    {
        non_intersecting,
        touching,
        proper_intersection,
        proper_contains,
        co_planar,
    };

    inline std::string to_string(intersection_result res)
    {
        switch (res)
        {
        case intersection_result::non_intersecting:
            return "non-intersecting";
        case intersection_result::touching:
            return "touching";
        case intersection_result::proper_intersection:
            return "proper-intersection";
        case intersection_result::proper_contains:
            return "proper-contains";
        default:
            return "<invalid value>";
        }
    }

    // returns an enum describing how the triangle and the aabb intersect
    // see https://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/pubs/tribox.pdf
    template <class geometry_t>
    intersection_result intersection_type(PlaneMesh const& planeMesh, pm::face_handle face, typename geometry_t::aabb_t const& aabb_in)
    {
        // NOTE: all coordinates are multiplied by 2 so we can center properly
        static constexpr int bits_pos = 1 + geometry_t::bits_position;
        static constexpr int bits_cross = 1 + 2 * bits_pos;
        static constexpr int bits_dot = 2 + bits_cross + bits_pos; // 1 + ...?

        using pos_scalar_t = fixed_int<bits_pos>;
        using pos_t = tg::pos<3, pos_scalar_t>;
        using vec_t = tg::vec<3, pos_scalar_t>;

        using cross_scalar_t = fixed_int<bits_cross>;
        using cross_vec_t = tg::vec<3, cross_scalar_t>;

        using aabb_t = tg::aabb<3, pos_scalar_t>;

        auto const center = vec_t(aabb_in.max) + vec_t(aabb_in.min);
        auto const amin = pos_t(pos_scalar_t(aabb_in.min.x) << 1, //
            pos_scalar_t(aabb_in.min.y) << 1, //
            pos_scalar_t(aabb_in.min.z) << 1)
            - center;

        auto const amax = pos_t(pos_scalar_t(aabb_in.max.x) << 1, //
            pos_scalar_t(aabb_in.max.y) << 1, //
            pos_scalar_t(aabb_in.max.z) << 1)
            - center;

        auto const aabb = aabb_t(amin, amax);


        //auto const& t = tris.triangles[tri_idx];
        auto h = face.any_halfedge();
        /*auto p0 = pos[h.vertex_from()];
        auto p1 = pos[h.vertex_to()];
        auto p2 = pos[h.next().vertex_to()];*/

        auto const p0 = pos_t(pos_scalar_t(planeMesh.posInt(h.vertex_from()).x) << 1, //
            pos_scalar_t(planeMesh.posInt(h.vertex_from()).y) << 1, //
            pos_scalar_t(planeMesh.posInt(h.vertex_from()).z) << 1)
            - center;

        auto const p1 = pos_t(pos_scalar_t(planeMesh.posInt(h.vertex_to()).x) << 1, //
            pos_scalar_t(planeMesh.posInt(h.vertex_to()).y) << 1, //
            pos_scalar_t(planeMesh.posInt(h.vertex_to()).z) << 1)
            - center;

        auto const p2 = pos_t(pos_scalar_t(planeMesh.posInt(h.next().vertex_to()).x) << 1, //
            pos_scalar_t(planeMesh.posInt(h.next().vertex_to()).y) << 1, //
            pos_scalar_t(planeMesh.posInt(h.next().vertex_to()).z) << 1)
            - center;

        //    TG_ASSERT(!is_zero_vector(cross(p1 - p0, p2 - p0)) && "degenerate triangles not supported");
        TG_ASSERT(amin == -amax && "centering didn't work?");

        // early out: AABB vs tri AABB
        auto tri_aabb = aabb_of(p0, p1, p2);
        if (tri_aabb.max.x < amin.x || tri_aabb.max.y < amin.y || tri_aabb.max.z < amin.z || //
            tri_aabb.min.x > amax.x || tri_aabb.min.y > amax.y || tri_aabb.min.z > amax.z)
            return intersection_result::non_intersecting;

        auto const proper_contains = [](aabb_t const& b, pos_t const& p) {
            return b.min.x < p.x && p.x < b.max.x && //
                b.min.y < p.y && p.y < b.max.y && //
                b.min.z < p.z && p.z < b.max.z;
        };

        auto const contains_p0 = proper_contains(aabb, p0);
        auto const contains_p1 = proper_contains(aabb, p1);
        auto const contains_p2 = proper_contains(aabb, p2);

        // early in: tri points vs AABB
        if (contains_p0 && contains_p1 && contains_p2)
            return intersection_result::proper_contains;

        if (contains_p0 || contains_p1 || contains_p2)
            return intersection_result::proper_intersection;

        // get adjusted tri base plane
        auto plane = planeMesh.face(face);
        auto dis = double(signed_distance(plane, planeMesh.posInt(h.vertex_from())));
        TG_ASSERT(dis == 0 && "invalid plane?");

        plane.d = -(ob::mul<geometry_t::bits_plane_d>(plane.a, p0.x) + //
            ob::mul<geometry_t::bits_plane_d>(plane.b, p0.y) + //
            ob::mul<geometry_t::bits_plane_d>(plane.c, p0.z)); // account for scale and shift

        auto any_zero = false;

        // fast plane / AABB test
        {
            auto bn = abs(ob::mul<2 + geometry_t::bits_normal + bits_pos>(plane.a, amax.x)) + //
                abs(ob::mul<2 + geometry_t::bits_normal + bits_pos>(plane.b, amax.y)) + //
                abs(ob::mul<2 + geometry_t::bits_normal + bits_pos>(plane.c, amax.z));

            // min dis: d - bn
            if (less_than_zero(bn - plane.d))
                return intersection_result::non_intersecting;

            // max dis: d + bn
            if (less_than_zero(plane.d + bn))
                return intersection_result::non_intersecting;

            any_zero |= plane.d - bn == 0;
            any_zero |= plane.d + bn == 0;
        }

        // 9 axis SAT test
        {
            auto const is_seperating = [&any_zero, amax](cross_vec_t const& n, pos_t const& tp0, pos_t const& tp1) -> bool {
                if (ob::is_zero(n.x) && ob::is_zero(n.y) && ob::is_zero(n.z))
                    return false; // not a real candidate axis


                // fast point / AABB separation test
                auto bn = abs(mul<bits_dot>(n.x, amax.x)) + //
                    abs(mul<bits_dot>(n.y, amax.y)) + //
                    abs(mul<bits_dot>(n.z, amax.z));

                auto tn0 = mul<bits_dot>(n.x, tp0.x) + mul<bits_dot>(n.y, tp0.y) + mul<bits_dot>(n.z, tp0.z);
                auto tn1 = mul<bits_dot>(n.x, tp1.x) + mul<bits_dot>(n.y, tp1.y) + mul<bits_dot>(n.z, tp1.z);

                auto tmin = min(tn0, tn1);
                auto tmax = max(tn0, tn1);

                auto bmin = -bn;
                auto bmax = bn;

                if (less_than_zero(tmax - bmin))
                    return true;
                if (less_than_zero(bmax - tmin))
                    return true;

                any_zero |= bmin == tmax;
                any_zero |= bmax == tmin;

                return false;
            };

            auto const cross = [](auto const& a, auto const& b) {
                return cross_vec_t(mul<bits_cross>(a.y, b.z) - mul<bits_cross>(a.z, b.y), //
                    mul<bits_cross>(a.z, b.x) - mul<bits_cross>(a.x, b.z), //
                    mul<bits_cross>(a.x, b.y) - mul<bits_cross>(a.y, b.x));
            };

            if (is_seperating(cross(p1 - p0, vec_t::unit_x), p0, p2))
                return intersection_result::non_intersecting;
            if (is_seperating(cross(p1 - p0, vec_t::unit_y), p0, p2))
                return intersection_result::non_intersecting;
            if (is_seperating(cross(p1 - p0, vec_t::unit_z), p0, p2))
                return intersection_result::non_intersecting;

            if (is_seperating(cross(p2 - p0, vec_t::unit_x), p0, p1))
                return intersection_result::non_intersecting;
            if (is_seperating(cross(p2 - p0, vec_t::unit_y), p0, p1))
                return intersection_result::non_intersecting;
            if (is_seperating(cross(p2 - p0, vec_t::unit_z), p0, p1))
                return intersection_result::non_intersecting;

            if (is_seperating(cross(p1 - p2, vec_t::unit_x), p0, p2))
                return intersection_result::non_intersecting;
            if (is_seperating(cross(p1 - p2, vec_t::unit_y), p0, p2))
                return intersection_result::non_intersecting;
            if (is_seperating(cross(p1 - p2, vec_t::unit_z), p0, p2))
                return intersection_result::non_intersecting;
        }

        // any_zero for tri_aabb vs aabb
        any_zero |= tri_aabb.max.x <= amin.x;
        any_zero |= tri_aabb.max.y <= amin.y;
        any_zero |= tri_aabb.max.z <= amin.z;
        any_zero |= tri_aabb.min.x >= amax.x;
        any_zero |= tri_aabb.min.y >= amax.y;
        any_zero |= tri_aabb.min.z >= amax.z;

        // found no separating axis? -> intersection
        return any_zero ? intersection_result::touching : intersection_result::proper_intersection;
    }

    template <class GeometryT>
    struct Fraction {
        using scalar_1 = typename GeometryT::determinant_xxd_t;
        using scalar_2 = typename GeometryT::determinant_abc_t;
        scalar_1 numerator;
        scalar_2 denominator;
    };

    //bool greaterPositiveFraction(int a, int b, int c, int d);
    static bool greaterPositiveFraction(int a, int b, int c, int d)
    {
        if (d == 0) return false;
        if (b == 0) return true;
        if (a / b > c / d) return true;
        if (a / b < c / d) return false;
        return !greaterPositiveFraction(b, a % b, d, c % d);
    }

    /*static bool greaterFraction(int a, int b, int c, int d)
    {
        if (b < 0) { b = -b; a = -a; }
        if (d < 0) { d = -d; c = -c; }
        if (a < 0 && c < 0) return greaterPositiveFraction(-c, d, -a, b);
        if (a < 0) return false;
        if (c < 0) return true;
        return greaterPositiveFraction(a, b, c, d);
    }*/

    /*template <class GeometryT> 
    static bool isGreaterFraction(Fraction<GeometryT> small, Fraction< GeometryT> great)
    {
        using geometry_t = GeometryT;
        using scalar_t = fixed_int<geometry_t::bits_position * 2>;
        int Y = small.numerator * great.denominator - small.denominator * great.numerator;
        return Y < 0;
    }*/

    template <class GeometryT>
    struct Interval {
        subdeterminants<GeometryT> from;
        subdeterminants<GeometryT> to;
    };
    

    static std::vector<i64> addMulCarryI256(i256 v0, i256 v1, i256 v2, i256 v3) {
        //i64 l00 = v0.v0;
        //i64 l01 = v0.v1 + v1.v0;
        //i64 l02 = v0.v2 + v1.v1 + v2.v0;
        //i64 l03 = v0.v3 + v1.v2 + v2.v1 + v3.v0;
        //i64 l04 = v1.v3 + v2.v2 + v3.v1;
        //i64 l05 = v2.v3 + v3.v2;
        //i64 l06 = v3.v3;

        std::vector<i64> result(7);
        u64 value = 0;

        result[0] = v0.v0;

        auto c = _addcarry_u64(0, v0.v1, v1.v0, &value);
        result[1] = value;

        c = _addcarry_u64(c, v0.v2, v1.v1, &value);
        c += _addcarry_u64(0, value, v2.v0, &value);
        result[2] = value;

        c = _addcarry_u64(0, v0.v3, c, &value);
        c += _addcarry_u64(0, value, v1.v2, &value);
        c += _addcarry_u64(0, value, v2.v1, &value);
        c += _addcarry_u64(0, value, v3.v0, &value);
        result[3] = value;

        c = _addcarry_u64(0, v1.v3, c, &value);
        c += _addcarry_u64(0, value, v2.v2, &value);
        c += _addcarry_u64(0, value, v3.v1, &value);
        result[4] = value;

        c = _addcarry_u64(0, v2.v3, c, &value);
        c += _addcarry_u64(0, value, v3.v2, &value);
        result[5] = value;

        c = _addcarry_u64(0, v3.v3, c, &value);
        result[6] = value;
        TG_ASSERT(c == 0);
        return result;
    }

    //a*b > c*d
    static int8_t isAbGreaterCd(i192 a, i256 b, i192 c, i256 d) {
        i256 v00 = ob::mul<256>(a, b.v0);
        i256 v01 = ob::mul<256>(a, b.v1);
        i256 v02 = ob::mul<256>(a, b.v2);
        i256 v03 = ob::mul<256>(a, b.v3);

        
        i256 v10 = ob::mul<256>(c, d.v0);
        i256 v11 = ob::mul<256>(c, d.v1);
        i256 v12 = ob::mul<256>(c, d.v2);
        i256 v13 = ob::mul<256>(c, d.v3);

        auto addAB = addMulCarryI256(v00, v01, v02, v03);
        auto addCD = addMulCarryI256(v10, v11, v12, v13);

        for (int i = 6; i >= 0; --i) {
            if (addAB[i] > addCD[i])
                return 1;
            else if (addAB[i] < addCD[i])
                return -1;
        }
        return 0;
    }

    template <class GeometryT>
    static bool overlapTest(Fraction<GeometryT>& a1, Fraction<GeometryT>& a2, Fraction<GeometryT>& b1, Fraction<GeometryT>& b2) {
        int8_t sign = isAbGreaterCd(a1.denominator, b1.numerator, b1.denominator, a1.numerator);
        if (sign == 0)
            return true;

        if (sign != isAbGreaterCd(a1.denominator, b2.numerator, b2.denominator, a1.numerator))
            return true;

        if (sign != isAbGreaterCd(a2.denominator, b1.numerator, b1.denominator, a2.numerator))
            return true;

        if (sign != isAbGreaterCd(a2.denominator, b2.numerator, b2.denominator, a2.numerator))
            return true;

        return false;
    }

    template <class GeometryT>
    bool overlapIntervalStrongAxis(Interval<GeometryT> first, Interval<GeometryT> second, uint8_t axis) {
        Fraction<GeometryT> first_1;
        Fraction<GeometryT> first_2;
        Fraction<GeometryT> second_1;
        Fraction<GeometryT> second_2;
        first_1.denominator = first.from.det_abc;
        first_2.denominator = first.to.det_abc;
        second_1.denominator = second.from.det_abc;
        second_2.denominator = second.to.det_abc;
        if (axis == 0) {
            first_1.numerator = -first.from.det_bcd;
            first_2.numerator = -first.to.det_bcd;
            second_1.numerator = -second.from.det_bcd;
            second_2.numerator = -second.to.det_bcd;
        }
        else if (axis == 1) {
            first_1.numerator = first.from.det_acd;
            first_2.numerator = first.to.det_acd;
            second_1.numerator = second.from.det_acd;
            second_2.numerator = second.to.det_acd;
        }
        else {
            first_1.numerator = -first.from.det_abd;
            first_2.numerator = -first.to.det_abd;
            second_1.numerator = -second.from.det_abd;
            second_2.numerator = -second.to.det_abd;
        }
        return overlapTest(first_1, first_2, second_1, second_2);
    }

}

/*template <class geometry_t>
intersection_result intersection_type(triangle_handle<geometry_t> triangle, typename geometry_t::aabb_t const& aabb_in)
{
    return intersection_type(*triangle.soup, triangle.index, aabb_in);
}*/