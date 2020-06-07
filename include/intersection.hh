#pragma once
#include <map>
#include <polymesh/pm.hh>

struct IntersectionHandle {
    TrianlgeIntersection::IntersectionState intersection;
    pm::halfedge_handle intersectionEdge1;
    pm::halfedge_handle intersectionEdge2;
};

class TrianlgeIntersection {
public:
    struct IntersectionEdges {
        pm::halfedge_handle intersectionEdge1;
        pm::halfedge_handle intersectionEdge2;
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
    TrianlgeIntersectionNonPlanar() 
    {
        intersectionState = IntersectionState::NON_PLANAR;
    }

    IntersectionEdges triangle1;
    IntersectionEdges triangle2;
};

class TrianlgeIntersectionPlanar : public TrianlgeIntersection {
public:
    enum class PlanarState
    {
        NON_INTERSECTING,
        ONE_EDGE,
        TWO_EDGES,
    };

    struct EdgeData {
        PlanarState state;
        IntersectionEdges edges;
    };

    TrianlgeIntersectionPlanar() {
        intersectionState = IntersectionState::PLANAR;
    }

    std::map<pm::halfedge_handle, EdgeData> edgeDataT1;
    std::map<pm::vertex_handle, bool> vertexIsInnerPointT1;
    std::map<pm::halfedge_handle, EdgeData> vertexIsInnerPointT2;
    std::map<pm::vertex_handle, bool> vertexDataT2;
};