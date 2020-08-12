#pragma once
#include <map>
#include <polymesh/pm.hh>
#include <integer_math.hh>
#include <plane_polygon.hh>
//#include <intersection_utils.hh>

class TrianlgeIntersection {
public:
    struct IntersectionEdges {
        pm::halfedge_handle intersectionEdge1;
        pm::halfedge_handle intersectionEdge2;
        bool intersectVertex1;
        bool intersectVertex2;
    };

    enum class IntersectionState
    {
        NON_INTERSECTING,
        NON_PLANAR,
        PLANAR,
    };

    IntersectionState intersectionState = IntersectionState::NON_INTERSECTING;
};

class TrianlgeIntersectionNonPlanar : public TrianlgeIntersection {
public:

    //Touching: vertex of polygon ... lies on face ...
    enum class NonPlanarState
    {
        TOUCHING_1_ON_2,
        TOUCHING_2_ON_1,
        PROPER_INTERSECTION,
    };

    TrianlgeIntersectionNonPlanar() 
    {
        intersectionState = IntersectionState::NON_PLANAR;
    }

    NonPlanarState state = NonPlanarState::PROPER_INTERSECTION;
    IntersectionEdges triangle1;
    IntersectionEdges triangle2;
};

class TrianlgeIntersectionPlanar : public TrianlgeIntersection {
public:
    enum class PlanarState
    {
        UNKNOWN,
        NON_INTERSECTING_IN,
        NON_INTERSECTING_OUT,
        ONE_EDGE_TO_IN,
        ONE_EDGE_TO_OUT,
        ONE_EDGE,
        TWO_EDGES,
        MARKED,
    };

    struct EdgeData {
        pm::halfedge_handle edge;
        PlanarState state = PlanarState::UNKNOWN;
        IntersectionEdges intersectionEdges;
    };

    TrianlgeIntersectionPlanar() = delete;

    TrianlgeIntersectionPlanar(const std::vector<pm::halfedge_handle>& edges1, const std::vector<pm::halfedge_handle>& edges2) {
        mEdgeDataT1.resize(edges1.size());
        for (int i = 0; i < edges1.size(); ++i)
            mEdgeDataT1[i].edge = edges1[i];

        mEdgeDataT2.resize(edges2.size());
        for (int i = 0; i < edges2.size(); ++i)
            mEdgeDataT2[i].edge = edges2[i];

        intersectionState = IntersectionState::PLANAR;
    }

    EdgeData& getEdgeDataT1(size_t i) 
    {
        return mEdgeDataT1[i];
    }

    EdgeData& getEdgeDataT2(size_t i) 
    {
        return mEdgeDataT2[i];
    }

    const std::vector<EdgeData>& getEdgeDataT1() const
    {
        return mEdgeDataT1;
    }

    const std::vector<EdgeData>& getEdgeDataT2() const
    {
        return mEdgeDataT2;
    }

    std::vector<EdgeData>& getEdgeDataT1()
    {
        return mEdgeDataT1;
    }

    std::vector<EdgeData>& getEdgeDataT2()
    {
        return mEdgeDataT2;
    }

private:

    std::vector<EdgeData> mEdgeDataT1;
    //std::map<pm::vertex_handle, bool> vertexIsInnerPointT1;
    std::vector<EdgeData> mEdgeDataT2;
    //std::map<pm::vertex_handle, bool> vertexIsInnerPointT2;
};

/*struct IntersectionHandle {
    TrianlgeIntersection::IntersectionState intersection;
    pm::halfedge_handle intersectionEdge1;
    pm::halfedge_handle intersectionEdge2;
};*/

using SharedTriIntersect = std::shared_ptr<TrianlgeIntersection>;
using SharedTriIntersectNonPlanar = std::shared_ptr<TrianlgeIntersectionNonPlanar>;
using SharedTriIntersectPlanar = std::shared_ptr<TrianlgeIntersectionPlanar>;


//#############################################################################
//#                             Intersection Function                         #
//#############################################################################

class IntersectionObject {

public:
    //Todo replace
    enum class intersection_result
    {
        non_intersecting,
        touching,
        proper_intersection,
        proper_contains,
        co_planar,
    };

    struct IntersectionHandle {
        //using VertexTuple = std::tuple<pm::vertex_handle, pm::vertex_handle>;
        intersection_result intersection;
        pm::halfedge_handle intersectionEdge1;
        pm::halfedge_handle intersectionEdge2;
        bool intersectVertex1;
        bool intersectVertex2;
    };

    //Also checks if there is a vertex on the plane
    static IntersectionHandle planeBaseIntersection(const PlaneMesh& mesh1, pm::face_handle const& planeBase, const PlaneMesh& mesh2, pm::face_handle const& polygon)
    {
        int sign = 0;
        int index = 0;
        TG_ASSERT(!polygon.is_removed());
        TG_ASSERT(polygon.is_valid());
        auto halfedges = polygon.halfedges().to_vector([](pm::halfedge_handle i) { return i; });
        pm::halfedge_handle halfedgeHandles[2];
        bool vertexOnEdge[2];

        sign = ob::classify_vertex(mesh2.pos(halfedges[0].vertex_from()), mesh1.face(planeBase));
        //sign = mesh1.signDistanceToBasePlane(planeBase, halfedges[0].vertex_from());
        int signPosAcc = 0;
        int signNegAcc = 0;
        bool startWithSign0 = (sign == 0);
        int signTmp = sign;

        if (signTmp > 0)
            signPosAcc++;
        else if (signTmp < 0)
            signNegAcc++;

        for (int i = 0; i < halfedges.size(); ++i) {
            bool wasZeroBefore = (signTmp == 0);
            signTmp = ob::classify_vertex(mesh2.pos(halfedges[i].vertex_to()), mesh1.face(planeBase));

            if (signTmp > 0)
                signPosAcc++;
            else if (signTmp < 0)
                signNegAcc++;

            if (signTmp == sign)
                continue;

            /*if (signTmp == 0) {
                if ((index == 0 || !startWithSign0))
                    continue;

                index++;
                vertexOnEdge[1] = true;
                halfedgeHandles[1] = halfedges[i].next();
                break;
            }*/

            if (signTmp == 0 && wasZeroBefore) {
                continue;
            }

            if (signTmp == 0 && index == 2) {
                continue;
            }

            //Only happens if start with 0
            /*if (sign == 0) {
                sign = signTmp;
                continue;
            }*/

            if (wasZeroBefore && signPosAcc != 0 && signNegAcc != 0) {
                sign = signTmp;
                vertexOnEdge[index - 1] = true;
                halfedgeHandles[index - 1] = halfedges[i];
                continue;
            }

            sign = signTmp;
            vertexOnEdge[index] = wasZeroBefore;
            halfedgeHandles[index] = halfedges[i];
            index++;
            TG_ASSERT(index <= 2);
        }

        IntersectionHandle intersection;
        if (index == 0) {
            if (signPosAcc == 0 && signNegAcc == 0)
                intersection.intersection = intersection_result::co_planar;
            else
                intersection.intersection = intersection_result::non_intersecting;
            return intersection;
        }
        TG_ASSERT(index == 2 && "On a Convex Polygon are now exactly 2 intersections");
        if (signPosAcc > 0 && signNegAcc > 0)
            intersection.intersection = intersection_result::proper_intersection;
        else
            intersection.intersection = intersection_result::touching;

        intersection.intersectionEdge1 = halfedgeHandles[0];
        intersection.intersectionEdge2 = halfedgeHandles[1];
        intersection.intersectVertex1 = vertexOnEdge[0];
        intersection.intersectVertex2 = vertexOnEdge[1];
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

    static int8_t isInnerPoint(SubDet& pos, const std::vector<TrianlgeIntersectionPlanar::EdgeData>& edges, const PlaneMesh& edgesMesh) {
        int8_t sign = -1;
        for (const TrianlgeIntersectionPlanar::EdgeData& edge : edges) {
            int8_t direction = ob::classify_vertex(pos, edgesMesh.edge(edge.edge));
            direction *= edgesMesh.halfedge(edge.edge);
            if (direction > 0)
                return 1;
            else if (direction == 0)
                sign = 0;
        }
        return sign;
    }

    static void classifyIntersectionEdges(const PlaneMesh& edgeMesh,
        std::vector<TrianlgeIntersectionPlanar::EdgeData>& edgesData,
        const PlaneMesh& intersectionMesh,
        const std::vector<TrianlgeIntersectionPlanar::EdgeData>& intersectionData)
    {
        typedef TrianlgeIntersectionPlanar::EdgeData EdgeData;
        typedef TrianlgeIntersectionPlanar::PlanarState PlanarState;

        size_t edgesSize = edgesData.size();
        int8_t innerPoint = isInnerPoint(edgeMesh.pos(edgesData[0].edge.vertex_from()), intersectionData, intersectionMesh);
        for (int i = 0; i < edgesSize; ++i) {
            EdgeData& edgeData = edgesData[i];
            EdgeData& edgeDataNext = edgesData[(i + 1) % edgesSize];
            if (edgeData.state == PlanarState::UNKNOWN) {
                if (innerPoint == -1) {
                    //bool newIntersectionEdge = computeIntersectionEdgeHelper1(edgeMesh, edgeData, intersectionMesh, intersectionData[0]);                  
                    edgeData.state = PlanarState::NON_INTERSECTING_IN;
                    //fillNextIntersectionEdges(edgeData, edgeDataNext);
                    //edgeData.intersectionEdges.intersectionEdge1 = newIntersectionEdge;
                }
                else {
                    edgeData.state = PlanarState::NON_INTERSECTING_OUT;
                }
            }
            else if (edgeData.state == PlanarState::ONE_EDGE) {
                if (innerPoint >= 0) {
                    edgeData.state = PlanarState::ONE_EDGE_TO_IN;
                    //bool newIntersectionEdge = computeIntersectionEdgeHelper2(edgeMesh, edgeData, intersectionMesh);
                    //fillNextIntersectionEdges(edgeData, edgeDataNext);
                    //edgeData.intersectionEdges.intersectionEdge2 = newIntersectionEdge;
                    innerPoint = -1;
                }
                else {
                    edgeData.state = PlanarState::ONE_EDGE_TO_OUT;
                    innerPoint = 1;
                }

            }
        }
    }

    static void classifyNotIntersectionEdges(const PlaneMesh& mesh1, const PlaneMesh& mesh2, TrianlgeIntersectionPlanar& intersectionPlanar) {
        classifyIntersectionEdges(mesh1, intersectionPlanar.getEdgeDataT1(), mesh2, intersectionPlanar.getEdgeDataT2());
        classifyIntersectionEdges(mesh2, intersectionPlanar.getEdgeDataT2(), mesh1, intersectionPlanar.getEdgeDataT1());
    }

    void setStateOnIntersection(TrianlgeIntersectionPlanar::PlanarState& state) {
        if (state == TrianlgeIntersectionPlanar::PlanarState::UNKNOWN) {
            state = TrianlgeIntersectionPlanar::PlanarState::ONE_EDGE;
        }
        else if (state == TrianlgeIntersectionPlanar::PlanarState::ONE_EDGE) {
            state = TrianlgeIntersectionPlanar::PlanarState::TWO_EDGES;
        }
        else if (state == TrianlgeIntersectionPlanar::PlanarState::MARKED) {
            state = TrianlgeIntersectionPlanar::PlanarState::TWO_EDGES;
        }
        else
            TG_ASSERT(false && "A line can not intersect more than 2 edges in a convex polygon");
    }
   
    SharedTriIntersect handleCoplanar_FirstPointNotOnEdge(std::vector<int8_t>& signsFirstPoint, const std::vector<pm::halfedge_handle>& edges1, const std::vector<pm::halfedge_handle>& edges2)
    {
        bool isIntersecting = false;
        auto intersectionPlanar = std::make_shared<TrianlgeIntersectionPlanar>(edges1, edges2);

        for (int i = 0; i < edges1.size(); ++i) {
            pm::vertex_handle qOld = edges1[i].vertex_from();
            pm::vertex_handle q = edges1[(i + 1) % edges1.size()].vertex_from();
            TrianlgeIntersectionPlanar::EdgeData& edge1Data = intersectionPlanar->getEdgeDataT1(i);

            for (int j = 0; j < edges2.size(); ++j) {
                int8_t signStorageTmp = ob::classify_vertex(mPlaneMeshA.pos(q), mPlaneMeshB.edge(edges2[j].edge())) * mPlaneMeshB.halfedge(edges2[j]);
                int8_t signBefore = signsFirstPoint[j];
                if (signStorageTmp != signBefore && (signStorageTmp != 0 || signBefore == -1) && (signBefore != 0 || signStorageTmp == -1)) {
                    auto pos1 = edges2[j].vertex_from();
                    auto pos2 = edges2[j].vertex_to();
                    auto edge = mPlaneMeshA.findEdge(qOld, q);
                    int8_t t1 = mPlaneMeshA.halfedge(edge);
                    TG_ASSERT(t1 != 0);
                    int8_t sign = ob::classify_vertex(mPlaneMeshB.pos(pos1), mPlaneMeshA.edge(edge)) * mPlaneMeshA.halfedge(edge);
                    int8_t sign2 = ob::classify_vertex(mPlaneMeshB.pos(pos2), mPlaneMeshA.edge(edge)) * mPlaneMeshA.halfedge(edge);
                    if ((sign != sign2) && (sign2 != 0 || sign == -1) && (sign != 0 || sign2 == -1)) {
                        
                        TrianlgeIntersectionPlanar::EdgeData& edge2Data = intersectionPlanar->getEdgeDataT2(j);;

                        if (sign == 0 || sign2 == 0) {
                            int8_t innerPoint = -1;
                            if (sign == 0)
                                innerPoint = isInnerPoint(mPlaneMeshB.pos(pos2), intersectionPlanar->getEdgeDataT1(), mPlaneMeshA);
                            else
                                innerPoint = isInnerPoint(mPlaneMeshB.pos(pos1), intersectionPlanar->getEdgeDataT1(), mPlaneMeshA);

                            if (innerPoint != -1) {
                                signsFirstPoint[j] = signStorageTmp;
                                if (edge2Data.state == TrianlgeIntersectionPlanar::PlanarState::ONE_EDGE)
                                    edge2Data.state = TrianlgeIntersectionPlanar::PlanarState::TWO_EDGES;
                                else
                                    edge2Data.state = TrianlgeIntersectionPlanar::PlanarState::MARKED;
                                continue;
                            }
                        }

                        if (signBefore == 0 || signStorageTmp == 0) {
                            int8_t innerPoint = -1;
                            if(signBefore == 0)
                                innerPoint = isInnerPoint(mPlaneMeshA.pos(q), intersectionPlanar->getEdgeDataT2(), mPlaneMeshB);
                            else
                                innerPoint = isInnerPoint(mPlaneMeshA.pos(qOld), intersectionPlanar->getEdgeDataT2(), mPlaneMeshB);

                            if (innerPoint != -1) {
                                signsFirstPoint[j] = signStorageTmp;
                                if (edge1Data.state == TrianlgeIntersectionPlanar::PlanarState::ONE_EDGE)
                                    edge1Data.state = TrianlgeIntersectionPlanar::PlanarState::TWO_EDGES;
                                else
                                    edge1Data.state = TrianlgeIntersectionPlanar::PlanarState::MARKED;
                                continue;
                            }
                        }

                        isIntersecting = true;
                        setStateOnIntersection(edge1Data.state);
                        setStateOnIntersection(edge2Data.state);
                    }

                }
                signsFirstPoint[j] = signStorageTmp;
            }
        }

        classifyNotIntersectionEdges(mPlaneMeshA, mPlaneMeshB, *intersectionPlanar);
        if (isIntersecting)
            return std::dynamic_pointer_cast<TrianlgeIntersection>(intersectionPlanar);

        if (!isIntersecting) {
            for (auto& it : intersectionPlanar->getEdgeDataT1()) {
                if (it.state == TrianlgeIntersectionPlanar::PlanarState::NON_INTERSECTING_IN) {
                    return std::dynamic_pointer_cast<TrianlgeIntersection>(intersectionPlanar);
                }
            }
        }

        if (!isIntersecting) {
            for (auto& it : intersectionPlanar->getEdgeDataT2()) {
                if (it.state == TrianlgeIntersectionPlanar::PlanarState::NON_INTERSECTING_IN) {
                    return std::dynamic_pointer_cast<TrianlgeIntersection>(intersectionPlanar);
                }
            }
        }

        return std::make_shared<TrianlgeIntersection>();
    }

    static bool handleCoplanar_FirstPointIsEdgePoint() {
        return true;
    }

    SharedTriIntersect handleCoplanar(const pm::face_handle& polygon1, const pm::face_handle& polygon2) {
        std::vector<pm::halfedge_handle> edges1 = polygon1.halfedges().to_vector([](pm::halfedge_handle i) { return i; });
        std::vector<pm::halfedge_handle> edges2 = polygon2.halfedges().to_vector([](pm::halfedge_handle i) { return i; });

        auto e1 = edges1[0];
        auto p1 = e1.vertex_from();
        std::vector<int8_t> signStorage(edges2.size());
        for (int i = 0; i < edges2.size(); ++i) {
            signStorage[i] = ob::classify_vertex(mPlaneMeshA.pos(p1), mPlaneMeshB.edge(edges2[i].edge())) * mPlaneMeshB.halfedge(edges2[i]);
        }

        auto pointCase = isInnerPointisEdgePoint(signStorage);
        bool isInnerPoint = std::get<0>(pointCase);
        bool isEdgePoint = std::get<1>(pointCase);

        auto result = handleCoplanar_FirstPointNotOnEdge(signStorage, edges1, edges2);
        return result;
    }

    template <class GeometryT>
    bool intersect(const pm::face_index& polygon1, const pm::face_index& polygon2)
    {
        return intersect<GeometryT>(polygon1.of(mPlaneMeshA.mesh()), polygon2.of(mPlaneMeshB.mesh()))->intersectionState != TrianlgeIntersection::IntersectionState::NON_INTERSECTING;
    }

    static void showFaces(PlaneMesh& mesh1, const pm::face_handle& polygon1, PlaneMesh& mesh2, const pm::face_handle& polygon2) {
        mesh1.checkAndComputePositions();
        mesh2.checkAndComputePositions();

        /*auto face1Mask = mesh1.mesh().faces().make_attribute_with_default(false);
        auto face2Mask = mesh2.mesh().faces().make_attribute_with_default(false);
        face1Mask[polygon1] = true;
        face2Mask[polygon2] = true;

        tg::aabb3 viewBox;
        viewBox.min = tg::pos3(mesh1.posInt(polygon1.any_vertex()));
        viewBox.max = tg::pos3(mesh2.posInt(polygon1.any_vertex()));*/

        pm::Mesh mesh;
        pm::vertex_attribute<tg::pos3> pos(mesh);
        std::vector<pm::vertex_handle> vertex_handles;

        for (auto vertex : polygon1.vertices()) {
            auto newVertex = mesh.vertices().add();
            pos[newVertex] = tg::pos3(mesh1.posInt(vertex));
            vertex_handles.push_back(newVertex);
        }
        mesh.faces().add(vertex_handles);

        vertex_handles.clear();
        for (auto vertex : polygon2.vertices()) {
            auto newVertex = mesh.vertices().add();
            pos[newVertex] = tg::pos3(mesh2.posInt(vertex));
            vertex_handles.push_back(newVertex);
        }
        mesh.faces().add(vertex_handles);

        //auto view = gv::view(pos);
        //gv::view(mesh2.positions(), gv::masked(face2Mask), viewBox);
    }

    //TODO: PlanePolygon? 
    template <class GeometryT>
    SharedTriIntersect intersect(const pm::face_handle& polygon1, const pm::face_handle& polygon2)
    {
        using normalScalar = ob::fixed_int<GeometryT::bits_normal * 2>;
        using normalVec = tg::vec<3, normalScalar>;
        static constexpr int NormalOutBits = GeometryT::bits_normal * 2;

        IntersectionHandle intersection1 = planeBaseIntersection(mPlaneMeshA, polygon1, mPlaneMeshB, polygon2);
        if (intersection1.intersection == intersection_result::non_intersecting)
            return std::make_shared<TrianlgeIntersection>();

        if (intersection1.intersection == intersection_result::co_planar) {
            SharedTriIntersect testCoplanar = handleCoplanar(polygon1, polygon2);
            return testCoplanar;
        }

        IntersectionHandle intersection2 = planeBaseIntersection(mPlaneMeshB, polygon2, mPlaneMeshA, polygon1);
        if (intersection2.intersection == intersection_result::non_intersecting)
            return std::make_shared<TrianlgeIntersection>();

        if (intersection2.intersection == intersection_result::touching && intersection1.intersection == intersection_result::touching)
            return std::make_shared<TrianlgeIntersection>();

        const Plane& basePlane1 = mPlaneMeshA.getPlane(polygon1);
        const Plane& basePlane2 = mPlaneMeshB.getPlane(polygon2);

        if (false && "without plane direction") {
            /*auto const crossa = mul<NormalOutBits>(basePlane1.b, basePlane2.c) - mul<NormalOutBits>(basePlane1.c, basePlane2.b);
            auto const crossb = mul<NormalOutBits>(basePlane1.c, basePlane2.a) - mul<NormalOutBits>(basePlane1.a, basePlane2.c);
            auto const crossc = mul<NormalOutBits>(basePlane1.a, basePlane2.b) - mul<NormalOutBits>(basePlane1.b, basePlane2.a);
            normalVec normal = normalVec({ crossa , crossb , crossc });

            uint8_t strongAxis = 0;
            if (normal.x > normal.y)
                strongAxis = normal.x > normal.z ? 0 : 2;
            else
                strongAxis = normal.y > normal.z ? 1 : 2;*/
        }
        else {
            //Debug
            //mesh1.checkAndComputePositions();
            //mesh2.checkAndComputePositions();
            auto edge1_1_1 = mPlaneMeshB.posInt(intersection1.intersectionEdge1.vertex_from());
            auto edge1_1_2 = mPlaneMeshB.posInt(intersection1.intersectionEdge1.vertex_to());
            auto edge1_2_1 = mPlaneMeshB.posInt(intersection1.intersectionEdge2.vertex_from());
            auto edge1_2_2 = mPlaneMeshB.posInt(intersection1.intersectionEdge2.vertex_to());
            auto edge2_1_1 = mPlaneMeshA.posInt(intersection2.intersectionEdge1.vertex_from());
            auto edge2_1_2 = mPlaneMeshA.posInt(intersection2.intersectionEdge1.vertex_to());
            auto edge2_2_1 = mPlaneMeshA.posInt(intersection2.intersectionEdge2.vertex_from());
            auto edge2_2_2 = mPlaneMeshA.posInt(intersection2.intersectionEdge2.vertex_to());
            SubDet subdet1 = mPlaneMeshA.pos(mPlaneMeshB.edge(intersection1.intersectionEdge1.edge()), mPlaneMeshA.face(polygon1), mPlaneMeshB.face(polygon2));
            SubDet subdet2 = mPlaneMeshA.pos(mPlaneMeshB.edge(intersection1.intersectionEdge2.edge()), mPlaneMeshA.face(polygon1), mPlaneMeshB.face(polygon2));
            int8_t sign1 = ob::classify_vertex(subdet1, mPlaneMeshA.edge(intersection2.intersectionEdge1.edge()));
            sign1 *= mPlaneMeshA.halfedge(intersection2.intersectionEdge1);
            //Changed direction. Direction computation with plane edge not det edge ...
            int8_t sign2 = ob::classify_vertex(subdet2, mPlaneMeshA.edge(intersection2.intersectionEdge1.edge()));
            sign2 *= mPlaneMeshA.halfedge(intersection2.intersectionEdge1);
            TG_ASSERT(sign1 != 0 || sign2 != 0);
            bool sharedPoint = false;

            auto nonPlanarIntersection = std::make_shared<TrianlgeIntersectionNonPlanar>();
            //Edges
            nonPlanarIntersection->triangle1.intersectionEdge1 = intersection2.intersectionEdge1;
            nonPlanarIntersection->triangle1.intersectionEdge2 = intersection2.intersectionEdge2;
            nonPlanarIntersection->triangle2.intersectionEdge1 = intersection1.intersectionEdge1;
            nonPlanarIntersection->triangle2.intersectionEdge2 = intersection1.intersectionEdge2;
            //Vertices
            nonPlanarIntersection->triangle1.intersectVertex1 = intersection2.intersectVertex1;
            nonPlanarIntersection->triangle1.intersectVertex2 = intersection2.intersectVertex2;
            nonPlanarIntersection->triangle2.intersectVertex1 = intersection1.intersectVertex1;
            nonPlanarIntersection->triangle2.intersectVertex2 = intersection1.intersectVertex2;
            //state 

            nonPlanarIntersection->state = TrianlgeIntersectionNonPlanar::NonPlanarState::PROPER_INTERSECTION;

            if (intersection2.intersection == intersection_result::touching)
                nonPlanarIntersection->state = TrianlgeIntersectionNonPlanar::NonPlanarState::TOUCHING_1_ON_2;

            if (intersection1.intersection == intersection_result::touching)
                nonPlanarIntersection->state = TrianlgeIntersectionNonPlanar::NonPlanarState::TOUCHING_2_ON_1;

            if (sign1 == 0 || sign2 == 0)
                sharedPoint = true;

            if (sign1 != sign2 && !sharedPoint)
                return nonPlanarIntersection;

            int8_t sign = sign1 != 0 ? sign1 : sign2;
            sign1 = ob::classify_vertex(subdet1, mPlaneMeshA.edge(intersection2.intersectionEdge2.edge()));
            sign1 *= mPlaneMeshA.halfedge(intersection2.intersectionEdge2);

            if (sharedPoint && sign1 == 0)
                return nonPlanarIntersection;

            if (sign == sign1 && sign1 != 0)
                return nonPlanarIntersection;

            sign2 = ob::classify_vertex(subdet2, mPlaneMeshA.edge(intersection2.intersectionEdge2.edge()));
            sign2 *= mPlaneMeshA.halfedge(intersection2.intersectionEdge2);

            auto test1 = mPlaneMeshA.halfedge(intersection2.intersectionEdge2);
            //auto test1 = mesh1.halfedge(intersection2.intersectionEdge2);

            if (!(sign1 != 0 || sign2 != 0))
                showFaces(mPlaneMeshA, polygon1, mPlaneMeshB, polygon2);

            TG_ASSERT(sign1 != 0 || sign2 != 0);

            if (sharedPoint && sign2 == 0)
                return nonPlanarIntersection;

            if (sign == sign2 && sign2 != 0)
                return nonPlanarIntersection;
        }
        return std::make_shared<TrianlgeIntersection>(); //overlapInterval(IntersectionHandle& intersection);
    }

    IntersectionObject() = delete;
    IntersectionObject(PlaneMesh& mMeshA, PlaneMesh& mMeshB) : mPlaneMeshA(mMeshA), mPlaneMeshB(mMeshB) {

    }

    private:
        PlaneMesh& mPlaneMeshA;
        PlaneMesh& mPlaneMeshB;
};

using IsectOb = IntersectionObject;
