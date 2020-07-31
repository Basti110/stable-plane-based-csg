#pragma once
#include <map>
#include <polymesh/pm.hh>
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

struct IntersectionHandle {
    TrianlgeIntersection::IntersectionState intersection;
    pm::halfedge_handle intersectionEdge1;
    pm::halfedge_handle intersectionEdge2;
};

using SharedTriIntersect = std::shared_ptr<TrianlgeIntersection>;
using SharedTriIntersectNonPlanar = std::shared_ptr<TrianlgeIntersectionNonPlanar>;
using SharedTriIntersectPlanar = std::shared_ptr<TrianlgeIntersectionPlanar>;