#include <plane_polygon.hh>
#include <nexus/test.hh>
#include <aabb.hh>
#include <glow-extras/viewer/view.hh>

typedef TrianlgeIntersection::IntersectionState state;
typedef TrianlgeIntersectionPlanar::EdgeData EdgeData;
typedef TrianlgeIntersectionPlanar::PlanarState PlanarState;

struct PlaneMeshInfo {
    PlaneMesh& planeMesh;
    pm::face_handle face;
};

struct NewFaces {
    std::tuple<pm::face_handle, pm::face_handle> facesT1;
    std::tuple<pm::face_handle, pm::face_handle> facesT2;
};

typedef TrianlgeIntersection::IntersectionEdges IntersectionEdges;

pm::face_handle addHalfFaceDependOnSplitEdges(pm::Mesh& mesh, pm::halfedge_handle& h1, pm::halfedge_handle& h2, pm::vertex_handle vStart, pm::vertex_handle vEnd) {
    auto h1From = h1.vertex_from();
    auto h1To = h1.vertex_to();
    auto h2From = h2.vertex_from();
    auto h2To = h2.vertex_to();

    std::vector<pm::halfedge_handle> hEdges;
    hEdges.push_back(mesh.halfedges().add_or_get(vStart, h1To));
    auto hTmp = h1.next();

    while (true) {
        if (hTmp == h2) {
            hEdges.push_back(mesh.halfedges().add_or_get(h2From, vEnd));
            hEdges.push_back(mesh.halfedges().add_or_get(vEnd, vStart));
            break;
        }
        hEdges.push_back(hTmp);
        hTmp = hTmp.next();
    }
    return mesh.faces().add(hEdges);
}

std::tuple<pm::face_handle, pm::face_handle> split(PlaneMeshInfo& planeMesh, IntersectionEdges& intersectionEdges, Plane& iSectPlane) {
    pm::Mesh& mesh = planeMesh.planeMesh.mesh();

    auto h1 = intersectionEdges.intersectionEdge1;
    auto h2 = intersectionEdges.intersectionEdge2;
    auto v1New = mesh.vertices().add();
    auto v2New = mesh.vertices().add();
    Plane plane = planeMesh.planeMesh.face(planeMesh.face);
    mesh.faces().remove(planeMesh.face);
    auto newFace1 = addHalfFaceDependOnSplitEdges(mesh, h1, h2, v1New, v2New);
    auto newFace2 = addHalfFaceDependOnSplitEdges(mesh, h2, h1, v2New, v1New);

    //Todo: Remove data in attributes
    Plane planeH1 = planeMesh.planeMesh.edge(h1);
    Plane planeH2 = planeMesh.planeMesh.edge(h2);
    int8_t signH1 = planeMesh.planeMesh.halfedge(h1);
    int8_t signH2 = planeMesh.planeMesh.halfedge(h2);

    auto edge = mesh.halfedges().add_or_get(v1New, v2New).edge();
    auto edge1_1 = mesh.halfedges().add_or_get(h1.vertex_from(), v1New);
    auto edge1_2 = mesh.halfedges().add_or_get(v1New, h1.vertex_to());
    auto edge2_1 = mesh.halfedges().add_or_get(h2.vertex_from(), v2New);
    auto edge2_2 = mesh.halfedges().add_or_get(v2New, h2.vertex_to());

    mesh.halfedges().remove_edge(h1);
    mesh.halfedges().remove_edge(h2);

    //Init Data
    planeMesh.planeMesh.setFace(newFace1, plane);
    planeMesh.planeMesh.setFace(newFace2, plane);
    planeMesh.planeMesh.setEdge(edge, iSectPlane);
    planeMesh.planeMesh.setEdge(edge1_1.edge(), planeH1);
    planeMesh.planeMesh.setEdge(edge1_2.edge(), planeH1);
    planeMesh.planeMesh.setEdge(edge2_1.edge(), planeH2);
    planeMesh.planeMesh.setEdge(edge2_2.edge(), planeH2);

    planeMesh.planeMesh.setHalfedge(edge1_1, signH1);
    planeMesh.planeMesh.setHalfedge(edge1_2, signH1);
    planeMesh.planeMesh.setHalfedge(edge2_1, signH2);
    planeMesh.planeMesh.setHalfedge(edge2_2, signH2);

    return std::make_tuple(newFace1, newFace2);
}

std::vector<pm::face_handle> splitPlanar(PlaneMeshInfo& planeMeshInfo1, PlaneMeshInfo& planeMeshInfo2, std::vector<EdgeData> edgesData) {
    PlaneMesh& planeMesh1 = planeMeshInfo1.planeMesh;
    PlaneMesh& planeMesh2 = planeMeshInfo2.planeMesh;
    std::vector<pm::vertex_handle> vertices;
    std::vector<pm::halfedge_handle> halfEdgesToDelete;
    bool deleteFace = false;

    for (int i = 0; i < edgesData.size(); ++i) {
        TG_ASSERT(edgesData[i].state != PlanarState::ONE_EDGE && edgesData[i].state != PlanarState::UNKNOWN);
        if (edgesData[i].state == PlanarState::NON_INTERSECTING_OUT)
            continue;

        if (edgesData[i].state == PlanarState::NON_INTERSECTING_IN) {
            auto newNodeIn = planeMesh2.mesh().vertices().add();
            auto newNodeOut = planeMesh2.mesh().vertices().add();
            auto intersectEdgeIn = edgesData[i].intersectionEdges.intersectionEdge1;
            auto intersectEdgeOut = edgesData[i].intersectionEdges.intersectionEdge2;
            halfEdgesToDelete.push_back(intersectEdgeIn);
            halfEdgesToDelete.push_back(intersectEdgeOut);
            vertices.push_back(newNodeIn);
            vertices.push_back(newNodeOut);

            std::vector<pm::halfedge_handle> halfEdges;
            auto halfEdge1 = planeMesh2.mesh().halfedges().add_or_get(newNodeIn, intersectEdgeIn.vertex_to());
            auto halfEdge = intersectEdgeIn.next();
            while (halfEdge != intersectEdgeOut) {
                halfEdges.push_back(halfEdge);
            }
            auto halfEdge2 = planeMesh2.mesh().halfedges().add_or_get(halfEdge.vertex_from(), newNodeOut);
            auto halfEdge3 = planeMesh2.mesh().halfedges().add_or_get(newNodeIn, newNodeOut);
            halfEdges.push_back(halfEdge1);
            halfEdges.push_back(halfEdge2);
            halfEdges.push_back(halfEdge3);
            planeMesh2.mesh().faces().add(halfEdges);
        }

        if (edgesData[i].state == PlanarState::NON_INTERSECTING_IN) {

        }
    }

    //TODO: delete Face
}


NewFaces split(SharedTriIntersect& intersection, PlaneMeshInfo& planeMeshInfo1, PlaneMeshInfo& planeMeshInfo2) {
    TG_ASSERT(intersection->intersectionState != TrianlgeIntersection::IntersectionState::NON_INTERSECTING);
    NewFaces splitFaces;

    if (intersection->intersectionState == TrianlgeIntersection::IntersectionState::NON_PLANAR) {
        SharedTriIntersectNonPlanar isectNonPlanar = std::static_pointer_cast<TrianlgeIntersectionNonPlanar>(intersection);
        Plane intersectionPlane1 = planeMeshInfo2.planeMesh.face(planeMeshInfo2.face);
        Plane intersectionPlane2 = planeMeshInfo1.planeMesh.face(planeMeshInfo1.face);
        splitFaces.facesT1 = split(planeMeshInfo1, isectNonPlanar->triangle1, intersectionPlane1);
        splitFaces.facesT2 = split(planeMeshInfo2, isectNonPlanar->triangle2, intersectionPlane2);
    }
    else if (intersection->intersectionState == TrianlgeIntersection::IntersectionState::PLANAR) {
        SharedTriIntersectPlanar isectNonPlanar = std::static_pointer_cast<TrianlgeIntersectionPlanar>(intersection);
        //Plane intersectionPlane1 = planeMeshInfo2.planeMesh.face(planeMesh2.face);
        //Plane intersectionPlane2 = planeMeshInfo1.planeMesh.face(planeMesh1.face);
        //splitFaces.facesT1 = splitPlanar(planeMeshInfo1, planeMeshInfo2, isectNonPlanar->getEdgeDataT1());
        //splitFaces.facesT2 = splitPlanar(planeMeshInfo2, planeMeshInfo1, isectNonPlanar->getEdgeDataT2());
    }
    return splitFaces;
}

void splitAccordingToIntersection(pm::face_handle triangle, std::vector<pm::face_handle>& triangles, PlaneMesh& planeMesh1, PlaneMesh& planeMesh2)
{
    pm::face_handle t2;
    //std::vector<pm::face_handle> trianglesTMP = triangles;
    for (int i = 0; i < triangles.size(); ++i) {
        pm::face_handle t2 = triangles[i];
        auto intersection = ob::intersect<geometry128>(planeMesh1, triangle, planeMesh2, t2);
        if (intersection->intersectionState != TrianlgeIntersection::IntersectionState::NON_INTERSECTING) {
            triangles.erase(triangles.begin() + i);
            auto splits = split(intersection, PlaneMeshInfo{ planeMesh1, triangle }, PlaneMeshInfo{ planeMesh2, t2 });
            //planeMesh1.mesh().compactify();

            /*{
                auto view = gv::view(planeMesh1.positions());
                gv::view(gv::lines(planeMesh1.positions()).line_width_world(0.1));
                gv::view(planeMesh2.positions());
                gv::view(gv::lines(planeMesh2.positions()).line_width_world(0.1));
            }*/
            splitAccordingToIntersection(std::get<0>(splits.facesT1), triangles, planeMesh1, planeMesh2);
            splitAccordingToIntersection(std::get<1>(splits.facesT1), triangles, planeMesh1, planeMesh2);
            triangles.push_back(std::get<0>(splits.facesT2));
            triangles.push_back(std::get<1>(splits.facesT2));
            break;
        }
    }
}

TEST("Test::Cut_Triangle_Normal") {
    tg::triangle<3, scalar_t> triangle1({ { 0, 0, -20 }, { 10, 0, 0 }, { -10, 0, 0 } });
    tg::triangle<3, scalar_t> triangle2({ { 0, 5, -15 }, { 5, -5, -5 }, { -5, -5, -5 } });
    tg::triangle<3, scalar_t> triangle3({ { 10, 5, -10 }, { 5, -5, 5 }, { 15, -5, 5 } });
    tg::triangle<3, scalar_t> triangle4({ { 0, 2, -7 }, { 0, -2, -9 }, { 0, -2, -5 } });

    PlaneMesh planeMesh1;
    PlaneMesh planeMesh2;

    std::vector<pm::face_handle> faces;
    scalar_t scale = 100;

    auto face1 = planeMesh1.insertTriangle(triangle1 * scale);
    faces.push_back(planeMesh2.insertTriangle(triangle2 * scale));
    faces.push_back(planeMesh2.insertTriangle(triangle3 * scale));
    //faces.push_back(planeMesh2.insertTriangle(triangle4 * scale));

    splitAccordingToIntersection(face1, faces, planeMesh1, planeMesh2);
    
    planeMesh1.checkAndComputePositions();
    planeMesh2.checkAndComputePositions();

    auto view = gv::view(planeMesh1.positions());
    gv::view(gv::lines(planeMesh1.positions()).line_width_world(10));
    gv::view(planeMesh2.positions());
    gv::view(gv::lines(planeMesh2.positions()).line_width_world(10));
}