#pragma once

#include <integer_math.hh>
#include <typed-geometry/tg.hh>
#include <polymesh/pm.hh>
#include <geometry.hh>
#include <plane_polygon.hh>
#include <intersection.hh>

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
    intersection_result intersection_type(PlanePolygon const& polygon, typename geometry_t::aabb_t const& aabb_in)
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
        auto h = polygon.face.any_halfedge();
        VertexAttribute& pos = polygon.positions;
        /*auto p0 = pos[h.vertex_from()];
        auto p1 = pos[h.vertex_to()];
        auto p2 = pos[h.next().vertex_to()];*/

        auto const p0 = pos_t(pos_scalar_t(pos[h.vertex_from()].x) << 1, //
            pos_scalar_t(pos[h.vertex_from()].y) << 1, //
            pos_scalar_t(pos[h.vertex_from()].z) << 1)
            - center;

        auto const p1 = pos_t(pos_scalar_t(pos[h.vertex_to()].x) << 1, //
            pos_scalar_t(pos[h.vertex_to()].y) << 1, //
            pos_scalar_t(pos[h.vertex_to()].z) << 1)
            - center;

        auto const p2 = pos_t(pos_scalar_t(pos[h.next().vertex_to()].x) << 1, //
            pos_scalar_t(pos[h.next().vertex_to()].y) << 1, //
            pos_scalar_t(pos[h.next().vertex_to()].z) << 1)
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
        auto plane = polygon.facePlanes[polygon.face];
        TG_ASSERT(signed_distance(plane, pos[h.vertex_from()]) == 0 && "invalid plane?");

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

    struct IntersectionHandle {
        //using VertexTuple = std::tuple<pm::vertex_handle, pm::vertex_handle>;
        intersection_result intersection;
        pm::halfedge_handle intersectionEdge1;
        pm::halfedge_handle intersectionEdge2;
    };

    template <class GeometryT>
    struct Interval {
        subdeterminants<GeometryT> from;
        subdeterminants<GeometryT> to;
    };

    //Coplanar
    static IntersectionHandle planeBaseIntersection(const PlaneMesh& mesh1, pm::face_handle const& planeBase, const PlaneMesh& mesh2, pm::face_handle const& polygon)
    {
        int sign = 0;
        int index = 0;
        
        auto halfedges = polygon.halfedges().to_vector([](pm::halfedge_handle i) { return i; });
        pm::halfedge_handle halfedgeHandles[2];
        
        sign = ob::classify_vertex(mesh2.pos(halfedges[0].vertex_from()), mesh1.face(planeBase));
        //sign = mesh1.signDistanceToBasePlane(planeBase, halfedges[0].vertex_from());
        int signAcc = 0;
        bool startWithSign0 = (sign == 0);
        for (int i = 0; i < halfedges.size(); ++i) {
            int signTmp = ob::classify_vertex(mesh2.pos(halfedges[i].vertex_to()), mesh1.face(planeBase));
            signAcc += std::abs(signTmp);
            if (signTmp == sign)
                continue;

            if (signTmp == 0 && (index == 0 || !startWithSign0))
                continue;

            if (sign == 0) {
                sign = signTmp;
                continue;
            }

            sign = signTmp;
            halfedgeHandles[index] = halfedges[i];
            index ++;
        }

        IntersectionHandle intersection;
        if (index == 0) {
            if(signAcc == 0)
                intersection.intersection = intersection_result::co_planar;
            else
                intersection.intersection = intersection_result::non_intersecting;
            return intersection;
        } 
        TG_ASSERT(index == 2 && "On a Convex Polygon are now exactly 2 intersections");
        intersection.intersection = intersection_result::proper_intersection;
        intersection.intersectionEdge1 = halfedgeHandles[0];
        intersection.intersectionEdge2 = halfedgeHandles[1];
        return intersection;
    }

    static std::tuple<bool, bool> isInnerPointisEdgePoint(const std::vector<int8_t>& signDistanceToPlane) {
        bool isInnerPoint = true;
        bool isEdgePoint = false;
        for (int i = 0; i < signDistanceToPlane.size(); ++i) {
            if (signDistanceToPlane[i] == 1)
                isInnerPoint = false;

            if (signDistanceToPlane[i] == 0) {
                isEdgePoint = true;
                isInnerPoint = false;
                break;
            }
        }
        return std::make_tuple(isInnerPoint, isEdgePoint);
    }

    static bool isInnerPoint(SubDet& pos, const std::vector<TrianlgeIntersectionPlanar::EdgeData>& edges) {
        return true;
    }

    static pm::halfedge_handle computeIntersectionEdgeHelper1(const PlaneMesh& edgeMesh,
        TrianlgeIntersectionPlanar::EdgeData& edgeData,
        const PlaneMesh& intersectionMesh,
        const TrianlgeIntersectionPlanar::EdgeData& intersectionData) 
    {
        auto potentialIntersectionEdge = intersectionData.edge;
        const Plane& planeIntersectionEdge = edgeMesh.edge(edgeData.edge);
        const Plane& planePrev = edgeMesh.edge(potentialIntersectionEdge.prev());
        int signPrev = edgeMesh.halfedge(potentialIntersectionEdge.prev());
        while (true) {
            //TG_ASSERT(intersectionEdge != potentialIntersectionEdge);
            const Plane& planePotIntersectionEdge = edgeMesh.edge(potentialIntersectionEdge);
            if (!ob::are_parallel(planePotIntersectionEdge, planeIntersectionEdge)) {
                SubDet pos;
                ob::compute_subdeterminants(edgeMesh.face(edgeData.edge.face()), planeIntersectionEdge, planePotIntersectionEdge, pos);
                auto sign = ob::classify_vertex(pos, planePrev) * signPrev;
                if (sign < 0) {
                    auto pos1 = edgeMesh.pos(potentialIntersectionEdge.vertex_from());
                    auto pos2 = edgeMesh.pos(potentialIntersectionEdge.vertex_to());
                    auto sign1 = ob::classify_vertex(pos1, planeIntersectionEdge);
                    auto sign2 = ob::classify_vertex(pos2, planeIntersectionEdge);
                    if (sign1 != sign2) {
                        if (sign1 = 0)
                            potentialIntersectionEdge = potentialIntersectionEdge.prev();
                        break;
                    }
                    else
                        potentialIntersectionEdge = potentialIntersectionEdge.next();
                }
            }
        }
        return potentialIntersectionEdge;
    }

    static pm::halfedge_handle computeIntersectionEdgeHelper2(const PlaneMesh& edgeMesh,
        TrianlgeIntersectionPlanar::EdgeData& edgeData,
        const PlaneMesh& intersectionMesh)
    {
        auto intersectionEdge = edgeData.intersectionEdges.intersectionEdge1;
        auto potentialIntersectionEdge = intersectionEdge.next();
        const Plane& planeIntersectionEdge = edgeMesh.edge(edgeData.edge);
        while (true) {
            TG_ASSERT(intersectionEdge != potentialIntersectionEdge);
            const Plane& planePotIntersectionEdge = edgeMesh.edge(potentialIntersectionEdge);
            if (!ob::are_parallel(planePotIntersectionEdge, planeIntersectionEdge)) {
                auto pos1 = edgeMesh.pos(potentialIntersectionEdge.vertex_from());
                auto pos2 = edgeMesh.pos(potentialIntersectionEdge.vertex_from());
                auto sign1 = ob::classify_vertex(pos1, planeIntersectionEdge);
                auto sign2 = ob::classify_vertex(pos2, planeIntersectionEdge);
                if (sign1 != sign2)
                    break;
                else
                    potentialIntersectionEdge = potentialIntersectionEdge.next();
            }
        }
        return potentialIntersectionEdge;
    }


    static void classifyIntersectionEdges(const PlaneMesh& edgeMesh, 
        std::vector<TrianlgeIntersectionPlanar::EdgeData>& edgesData,
        const PlaneMesh& intersectionMesh,       
        const std::vector<TrianlgeIntersectionPlanar::EdgeData>& intersectionData)
    {
        typedef TrianlgeIntersectionPlanar::EdgeData EdgeData;
        typedef TrianlgeIntersectionPlanar::PlanarState PlanarState;

        bool innerPoint = isInnerPoint(edgeMesh.pos(edgesData[0].edge.vertex_from()), intersectionData);
        for (EdgeData& edgeData : edgesData) {
            if (edgeData.state == PlanarState::UNKNOWN) {
                if (!innerPoint) {
                    auto newIntersectionEdge = computeIntersectionEdgeHelper1(edgeMesh, edgeData, intersectionMesh, intersectionData[0]);
                    edgeData.intersectionEdges.intersectionEdge1 = newIntersectionEdge;
                    edgeData.state = PlanarState::NON_INTERSECTING_IN;
                }
                else {
                    edgeData.state = PlanarState::NON_INTERSECTING_OUT;
                }             
            }             
            else if (edgeData.state == PlanarState::ONE_EDGE) {
                if (!innerPoint) {
                    auto newIntersectionEdge = computeIntersectionEdgeHelper2(edgeMesh, edgeData, intersectionMesh);
                    edgeData.intersectionEdges.intersectionEdge2 = newIntersectionEdge;
                }
                innerPoint = !innerPoint;
            }            
        }
    }

    static void classifyNotIntersectionEdges(const PlaneMesh& mesh1, const PlaneMesh& mesh2, TrianlgeIntersectionPlanar& intersectionPlanar) {
        classifyIntersectionEdges(mesh1, intersectionPlanar.getEdgeDataT1(), mesh2, intersectionPlanar.getEdgeDataT2());
        classifyIntersectionEdges(mesh2, intersectionPlanar.getEdgeDataT2(), mesh1, intersectionPlanar.getEdgeDataT1());
    }

    static TrianlgeIntersectionPlanar handleCoplanar_FirstPointNotOnEdge(const PlaneMesh& mesh1,
        const PlaneMesh& mesh2, 
        std::vector<int8_t>& signsFirstPoint,
        const std::vector<pm::halfedge_handle>& edges1,
        const std::vector<pm::halfedge_handle>& edges2) 
    {
        TrianlgeIntersectionPlanar intersectionPlanar(edges1, edges2);

        for (int i = 1; i < edges1.size(); ++i) {
            pm::vertex_handle qOld = edges1[i - 1].vertex_from();
            pm::vertex_handle q = edges1[i].vertex_from();
            TrianlgeIntersectionPlanar::EdgeData& edge1Data = intersectionPlanar.getEdgeDataT1(i);

            for (int j = 0; j < edges2.size(); ++j) {
                int8_t signStorageTmp = ob::classify_vertex(mesh1.pos(q), mesh2.edge(edges2[j].edge())) * mesh2.halfedge(edges2[j]);
                if (signStorageTmp != signsFirstPoint[j] && signStorageTmp != 0) {
                    auto pos1 = edges2[j].vertex_from();
                    auto pos2 = edges2[j].vertex_to();
                    auto edge = mesh1.findEdge(q, qOld).edge();
                    int8_t sign = ob::classify_vertex(mesh2.pos(pos1), mesh1.edge(edge));
                    int8_t sign2 = ob::classify_vertex(mesh2.pos(pos2), mesh1.edge(edge));
                    if (sign != sign2 && sign != 0 && sign2 != 0) {
                        if (edge1Data.state == TrianlgeIntersectionPlanar::PlanarState::UNKNOWN) {
                            edge1Data.intersectionEdges.intersectionEdge1 = edges2[j];
                            edge1Data.state = TrianlgeIntersectionPlanar::PlanarState::ONE_EDGE;
                        }
                        else if (edge1Data.state == TrianlgeIntersectionPlanar::PlanarState::ONE_EDGE) {
                            edge1Data.intersectionEdges.intersectionEdge2 = edges2[j];
                            edge1Data.state = TrianlgeIntersectionPlanar::PlanarState::TWO_EDGES;
                        }
                        else
                            TG_ASSERT(false && "A line can not intersect more than 2 edges in a convex polygon");


                        TrianlgeIntersectionPlanar::EdgeData& edge2Data = intersectionPlanar.getEdgeDataT2(j);;

                        if (edge2Data.state == TrianlgeIntersectionPlanar::PlanarState::UNKNOWN) {
                            edge2Data.intersectionEdges.intersectionEdge1 = edges1[j];
                            edge2Data.state = TrianlgeIntersectionPlanar::PlanarState::ONE_EDGE;
                        }
                        else if (edge2Data.state == TrianlgeIntersectionPlanar::PlanarState::ONE_EDGE) {
                            edge2Data.intersectionEdges.intersectionEdge2 = edges1[j];
                            edge2Data.state = TrianlgeIntersectionPlanar::PlanarState::TWO_EDGES;
                        }
                        else
                            TG_ASSERT(false && "A line can not intersect more than 2 edges in a convex polygon");

                    }
                    signsFirstPoint[j] = signStorageTmp;
                }
            }
        }
        classifyNotIntersectionEdges();
        return intersectionPlanar;
    }

    static bool handleCoplanar_FirstPointIsEdgePoint() {
        return true;
    }

    static TrianlgeIntersectionPlanar handleCoplanar(const PlaneMesh& mesh1, const pm::face_handle& polygon1, const PlaneMesh& mesh2, const pm::face_handle& polygon2) {
        std::vector<pm::halfedge_handle> edges1 = polygon1.halfedges().to_vector([](pm::halfedge_handle i) { return i; });
        std::vector<pm::halfedge_handle> edges2 = polygon2.halfedges().to_vector([](pm::halfedge_handle i) { return i; });
        
        auto e1 = edges1[0];
        auto p1 = e1.vertex_from();
        std::vector<int8_t> signStorage(edges2.size());
        for (int i = 0; i < edges2.size(); ++i) {
            signStorage[i] = ob::classify_vertex(mesh1.pos(p1), mesh2.edge(edges2[i].edge())) * mesh2.halfedge(edges2[i]);
            if (signStorage[i] == 0) {
                signStorage[i] = 1;
            }
        }

        auto pointCase = isInnerPointisEdgePoint(signStorage);
        bool isInnerPoint = std::get<0>(pointCase);
        bool isEdgePoint = std::get<1>(pointCase);

        TG_ASSERT(!isEdgePoint);
        if (isInnerPoint)
            return true;

        //Check inner point from edge to. Avoids Poylygon in Polygon
        {
            auto p2 = edges2[0].vertex_from();
            std::vector<int8_t> signStorage2(edges1.size());
            for (int i = 0; i < edges1.size(); ++i) {
                signStorage2[i] = ob::classify_vertex(mesh2.pos(p2), mesh1.edge(edges1[i].edge())) * mesh1.halfedge(edges1[i]);
            }
            if (std::get<0>(isInnerPointisEdgePoint(signStorage2)))
                return true;
        }
        
        return handleCoplanar_FirstPointNotOnEdge(mesh1, mesh2, signStorage, edges1, edges2);
    }

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
    template <class GeometryT>
    static bool intersect(const PlaneMesh& mesh1, const pm::face_index& polygon1, const PlaneMesh& mesh2, const pm::face_index& polygon2)
    {
        return intersect<GeometryT>(mesh1, polygon1.of(mesh1.mesh()), mesh2, polygon2.of(mesh2.mesh()));
    }
    //TODO: PlanePolygon? 
    template <class GeometryT>
    static bool intersect(const PlaneMesh& mesh1, const pm::face_handle& polygon1, const PlaneMesh& mesh2, const pm::face_handle& polygon2)
    {
        using normalScalar = fixed_int<GeometryT::bits_normal * 2>;
        using normalVec = tg::vec<3, normalScalar>;
        static constexpr int NormalOutBits = GeometryT::bits_normal * 2;

        IntersectionHandle intersection1 = planeBaseIntersection(mesh1, polygon1, mesh2, polygon2);
        if (intersection1.intersection == intersection_result::non_intersecting)
            return false;

        if (intersection1.intersection == intersection_result::co_planar)
            return handleCoplanar(mesh1, polygon1, mesh2, polygon2); //TODO

        IntersectionHandle intersection2 = planeBaseIntersection(mesh2, polygon2, mesh1, polygon1);
        if (intersection2.intersection == intersection_result::non_intersecting)
            return false;

        const Plane& basePlane1 = mesh1.getPlane(polygon1);
        const Plane& basePlane2 = mesh2.getPlane(polygon2);

        if (false && "without plane direction") {
            auto const crossa = mul<NormalOutBits>(basePlane1.b, basePlane2.c) - mul<NormalOutBits>(basePlane1.c, basePlane2.b);
            auto const crossb = mul<NormalOutBits>(basePlane1.c, basePlane2.a) - mul<NormalOutBits>(basePlane1.a, basePlane2.c);
            auto const crossc = mul<NormalOutBits>(basePlane1.a, basePlane2.b) - mul<NormalOutBits>(basePlane1.b, basePlane2.a);
            normalVec normal = normalVec({ crossa , crossb , crossc });

            uint8_t strongAxis = 0;
            if (normal.x > normal.y)
                strongAxis = normal.x > normal.z ? 0 : 2;
            else
                strongAxis = normal.y > normal.z ? 1 : 2;
        }
        else {
            auto edge1_1_1 = mesh2.posInt(intersection1.intersectionEdge1.vertex_from());
            auto edge1_1_2 = mesh2.posInt(intersection1.intersectionEdge1.vertex_to());
            auto edge1_2_1 = mesh2.posInt(intersection1.intersectionEdge2.vertex_from());
            auto edge1_2_2 = mesh2.posInt(intersection1.intersectionEdge2.vertex_to());
            auto edge2_1_1 = mesh1.posInt(intersection2.intersectionEdge1.vertex_from());
            auto edge2_1_2 = mesh1.posInt(intersection2.intersectionEdge1.vertex_to());
            auto edge2_2_1 = mesh1.posInt(intersection2.intersectionEdge2.vertex_from());
            auto edge2_2_2 = mesh1.posInt(intersection2.intersectionEdge2.vertex_to());
            SubDet subdet1 = mesh1.pos(mesh2.edge(intersection1.intersectionEdge1.edge()), mesh1.face(polygon1), mesh2.face(polygon2));
            SubDet subdet2 = mesh1.pos(mesh2.edge(intersection1.intersectionEdge2.edge()), mesh1.face(polygon1), mesh2.face(polygon2));
            int8_t sign1 = ob::classify_vertex(subdet1, mesh1.edge(intersection2.intersectionEdge1.edge()));
            sign1 *= mesh1.halfedge(intersection2.intersectionEdge1);
            //Changed direction. Direction computation with plane edge not det edge ...
            int8_t sign2 = ob::classify_vertex(subdet2, mesh1.edge(intersection2.intersectionEdge1.edge()));
            sign2 *= mesh1.halfedge(intersection2.intersectionEdge1);
            TG_ASSERT(sign1 != 0 || sign2 != 0);
            bool sharedPoint = false;
            if (sign1 == 0 || sign2 == 0)
                sharedPoint = true;

            if (sign1 != sign2 && !sharedPoint)
                return true;

            int8_t sign = sign1 != 0 ? sign1 : sign2;
            sign1 = ob::classify_vertex(subdet1, mesh1.edge(intersection2.intersectionEdge2.edge()));
            sign1 *= mesh1.halfedge(intersection2.intersectionEdge2);

            if (sharedPoint && sign1 == 0)
                return true;

            if (sign == sign1 && sign1 != 0)
                return true;

            sign2 = ob::classify_vertex(subdet2, mesh1.edge(intersection2.intersectionEdge2.edge()));
            sign2 *= mesh1.halfedge(intersection2.intersectionEdge2);
            TG_ASSERT(sign1 != 0 || sign2 != 0);

            if (sharedPoint && sign2 == 0)
                return true;

            if (sign == sign2 && sign2 != 0)
                return true;
        }             
        return false; //overlapInterval(IntersectionHandle& intersection);
    }
}

/*template <class geometry_t>
intersection_result intersection_type(triangle_handle<geometry_t> triangle, typename geometry_t::aabb_t const& aabb_in)
{
    return intersection_type(*triangle.soup, triangle.index, aabb_in);
}*/