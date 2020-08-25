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

    static bool intersection(PlaneMesh& mMesh, const pm::face_handle& face) {
        return false;
    }

    IntersectionObject() = delete;
    IntersectionObject(PlaneMesh& mMeshA, PlaneMesh& mMeshB) : mPlaneMeshA(mMeshA), mPlaneMeshB(mMeshB) {}
    static void showFaces(PlaneMesh& mesh1, const pm::face_handle& polygon1, PlaneMesh& mesh2, const pm::face_handle& polygon2);

    //Also checks if there is a vertex on the plane
    IntersectionHandle planeBaseIntersection(const PlaneMesh& mesh1, pm::face_handle const& planeBase, const PlaneMesh& mesh2, pm::face_handle const& polygon);
    std::tuple<bool, bool> isInnerPointisEdgePoint(const std::vector<int8_t>& signDistanceToPlane);
    int8_t isInnerPoint(SubDet& pos, const std::vector<TrianlgeIntersectionPlanar::EdgeData>& edges, const PlaneMesh& edgesMesh);

    void classifyIntersectionEdges(const PlaneMesh& edgeMesh, std::vector<TrianlgeIntersectionPlanar::EdgeData>& edgesData, const PlaneMesh& intersectionMesh, const std::vector<TrianlgeIntersectionPlanar::EdgeData>& intersectionData);
    void classifyNotIntersectionEdges(const PlaneMesh& mesh1, const PlaneMesh& mesh2, TrianlgeIntersectionPlanar& intersectionPlanar);
    void setStateOnIntersection(TrianlgeIntersectionPlanar::PlanarState& state);
   
    SharedTriIntersect getClassifiedEdgesFromSignChange(std::vector<int8_t>& signsFirstPoint, const std::vector<pm::halfedge_handle>& edges1, const std::vector<pm::halfedge_handle>& edges2);
    SharedTriIntersect handleCoplanarIntersection(const pm::face_handle& polygon1, const pm::face_handle& polygon2);
    SharedTriIntersect handleIntersection(const pm::face_handle& polygon1, const pm::face_handle& polygon2, IntersectionHandle& intersection1, IntersectionHandle& intersection2);

    template <class GeometryT>
    bool intersect(const pm::face_index& polygon1, const pm::face_index& polygon2)
    {
        return intersect<GeometryT>(polygon1.of(mPlaneMeshA.mesh()), polygon2.of(mPlaneMeshB.mesh()))->intersectionState != TrianlgeIntersection::IntersectionState::NON_INTERSECTING;
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
            SharedTriIntersect Coplanar = handleCoplanarIntersection(polygon1, polygon2);
            return Coplanar;
        }

        IntersectionHandle intersection2 = planeBaseIntersection(mPlaneMeshB, polygon2, mPlaneMeshA, polygon1);
        if (intersection2.intersection == intersection_result::non_intersecting)
            return std::make_shared<TrianlgeIntersection>();

        if (intersection2.intersection == intersection_result::touching && intersection1.intersection == intersection_result::touching)
            return std::make_shared<TrianlgeIntersection>();

        return handleIntersection(polygon1, polygon2, intersection1, intersection2);
    }

    private:
        PlaneMesh& mPlaneMeshA;
        PlaneMesh& mPlaneMeshB;
};

using IsectOb = IntersectionObject;
