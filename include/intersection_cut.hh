#include <plane_polygon.hh>
#include <nexus/test.hh>
#include <aabb.hh>
#include <glow-extras/viewer/view.hh>
#include <unordered_map>

typedef TrianlgeIntersection::IntersectionState state;
typedef TrianlgeIntersectionPlanar::EdgeData EdgeData;
typedef TrianlgeIntersectionPlanar::PlanarState PlanarState;

struct PlaneMeshInfo {
    PlaneMesh& planeMesh;
    pm::face_handle face;
};

struct NewFaces {
    std::vector<pm::face_handle> facesT1;
    std::vector<pm::face_handle> facesT2;
};

typedef TrianlgeIntersection::IntersectionEdges IntersectionEdges;

class IntersectionCut {
public:
    IntersectionCut(PlaneMesh* mMeshA, PlaneMesh* mMeshB);
    IntersectionCut();

    void cutPolygons(std::vector<pm::face_index>& facesMeshA, std::vector<pm::face_index>& facesMeshB);
    std::vector<pm::face_handle> splitAccordingToIntersection(pm::face_handle triangle, std::vector<pm::face_handle>& triangles);
    SharedTriIntersect getIntersectionStateWithTimer(pm::face_handle& t1, pm::face_handle& t2);

    void printTimes() {
        std::cout << "total time intersection: " << (double)intersectionTimeCount / 1000000 << "ms" << std::endl;
        std::cout << "total time split: " << (double)splitTimeCount / 1000000 << "ms" << std::endl;
        std::cout << "intersection calls: " << intersectionCount << std::endl;
        std::cout << "total time split: " << splitCount << std::endl;
    }

    const pm::edge_attribute<bool>& getIntersectionEdgesMarkerA() const {
        return mIntersectionEdgesMarkerA;
    }

    const pm::edge_attribute<bool>& getIntersectionEdgesMarkerB() const {
        return mIntersectionEdgesMarkerB;
    }

private:

    pm::vertex_handle splitHalfEdgeLowAPI(pm::Mesh& mesh, pm::halfedge_handle& h);
    pm::face_handle addFaceFromCycleAndStopPoint(pm::Mesh& mesh, pm::halfedge_handle& startEdge, pm::vertex_handle stopVertex);
    IntersectionEdges getIntersectionEdges(PlaneMeshInfo& faceInfo, Plane& plane, int8_t planeSign);
    
    //Schnitt Plane Erstes Face (direction * plane) zeigt nach auﬂen (signed distance = -1)
    //Schnitt Plane Zweites Face (direction * plane) zeigt nach innen (signed distance = 1)
    std::tuple<pm::face_handle, pm::face_handle> splitFace(PlaneMeshInfo& faceInfo, Plane& plane, int8_t direction);
    
    //TODO: Remove Debug entries
    std::vector<pm::face_handle> split(PlaneMeshInfo& planeMesh, IntersectionEdges& intersectionEdges, Plane& iSectPlane);  
    std::vector<pm::face_handle> splitFacesWithAllIntersectionEdges(PlaneMeshInfo& planeMeshInfo1, PlaneMeshInfo& planeMeshInfo2, std::vector<TrianlgeIntersectionPlanar::EdgeData>& edgeData);
    
    NewFaces splitPlanar(SharedTriIntersectPlanar& isectPlanar, PlaneMeshInfo& planeMeshInfo1, PlaneMeshInfo& planeMeshInfo2);
    NewFaces split(SharedTriIntersect& intersection, PlaneMeshInfo& planeMeshInfo1, PlaneMeshInfo& planeMeshInfo2);
    NewFaces splitnWithTimer(pm::face_handle& t1, pm::face_handle& t2, SharedTriIntersect i);

    bool edgeIsSplitEdge(PlanarState& state);
   
    void planeToTriangles(Plane& plane, int sideLen, std::vector<tg::triangle3>& insertVec);
    void showFaces(PlaneMesh& planeMesh, std::vector<pm::face_handle> faces);
    void showFaces(pm::face_handle t1, pm::face_handle t2);
    void checkLookupAndSplit(std::vector<pm::face_handle>& faces1, std::vector<pm::face_handle>& faces2, pm::face_handle& index);
    void fillFacesFromLookupBInVec(std::vector<pm::face_handle>& vec, pm::face_handle& index);
    
private:
    //Todo: Only debug
    long splitTimeCount = 0;
    long intersectionTimeCount = 0;
    long intersectionCount = 0;
    long splitCount = 0;

    pm::edge_attribute<bool> mIntersectionEdgesMarkerA;
    pm::edge_attribute<bool> mIntersectionEdgesMarkerB;

    PlaneMesh* mMeshA = nullptr;
    PlaneMesh* mMeshB = nullptr;

    std::unordered_map<int, std::vector<pm::face_handle>> mLookupFacesA;
    std::unordered_map<int, std::vector<pm::face_handle>> mLookupFacesB;
};
