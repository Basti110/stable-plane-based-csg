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

static pm::vertex_handle splitHalfEdgeLowAPI(pm::Mesh& mesh, pm::halfedge_handle& h) {
    return mesh.handle_of(pm::low_level_api(&mesh).halfedge_split(h.idx));
}

pm::face_handle addHalfFaceDependOnSplitEdges(pm::Mesh& mesh, pm::halfedge_handle& edgeOnFace, pm::vertex_handle firstVertex, pm::vertex_handle secondVertex) {
    TG_ASSERT(edgeOnFace.vertex_from() == secondVertex && "Edge must go out from vertex 2");
    /*auto h1From = h1.vertex_from();
    auto h1To = h1.vertex_to();
    auto h2From = h2.vertex_from();
    auto h2To = h2.vertex_to();*/

    std::vector<pm::halfedge_handle> hEdges;
    //hEdges.push_back(mesh.halfedges().add_or_get(vStart, h1To));
    auto hTmp = edgeOnFace;

    while (true) {
        hEdges.push_back(hTmp);
        if (hTmp.vertex_to() == firstVertex) {
            hEdges.push_back(mesh.halfedges().add_or_get(firstVertex, secondVertex));
            break;
        }
        hTmp = hTmp.next();
    }
    return mesh.faces().add(hEdges);
}

std::tuple<pm::face_handle, pm::face_handle> split(PlaneMeshInfo& planeMesh, IntersectionEdges& intersectionEdges, Plane& iSectPlane) {
    pm::Mesh& mesh = planeMesh.planeMesh.mesh();
    int countEdges1 = mesh.halfedges().count();
    //int countFaces1 = mesh.faces().count();
    auto h1 = intersectionEdges.intersectionEdge1;
    auto h2 = intersectionEdges.intersectionEdge2;
    auto h1From = h1.vertex_from();
    auto h1To = h1.vertex_to();
    auto h2From = h2.vertex_from();
    auto h2To = h2.vertex_to();



    // split edges
    auto v1New = splitHalfEdgeLowAPI(mesh, h1);
    auto v2New = splitHalfEdgeLowAPI(mesh, h2);

    //Oder andersrum? vertex_to == new?
    TG_ASSERT(h1.vertex_to() == v1New);
    TG_ASSERT(h2.vertex_to() == v2New);
    
    Plane plane = planeMesh.planeMesh.face(planeMesh.face);
    mesh.faces().remove(planeMesh.face);
    int countEdges4 = mesh.halfedges().count();

    //bool faceTest1 = h1.opposite().face().is_valid();
    //bool faceTest2 = h2.opposite().face().is_valid();
    auto h1Next = h1.next();
    auto h2Next = h2.next();
    
    auto newFace1 = addHalfFaceDependOnSplitEdges(mesh, h1Next, v2New, v1New);
    int countEdges5 = mesh.halfedges().count();
    auto newFace2 = addHalfFaceDependOnSplitEdges(mesh, h2Next, v1New, v2New);
    int countEdges6 = mesh.halfedges().count();

    //faceTest1 = h1.opposite().face().is_valid();
    //faceTest2 = h2.opposite().face().is_valid();
    TG_ASSERT(newFace1.is_valid());
    TG_ASSERT(newFace2.is_valid());

    bool invalidEdges = planeMesh.planeMesh.existsEdgesWithoutFace();
    TG_ASSERT(!invalidEdges);

    //Todo: Remove data in attributes
    Plane planeH1 = planeMesh.planeMesh.edge(h1);
    Plane planeH2 = planeMesh.planeMesh.edge(h2);
    int8_t signH1 = planeMesh.planeMesh.halfedge(h1);
    int8_t signH2 = planeMesh.planeMesh.halfedge(h2);

    auto edgeFace1 = mesh.halfedges().add_or_get(v2New, v1New);
    auto edgeFace2 = mesh.halfedges().add_or_get(v1New, v2New);

    auto edge1_1 = mesh.halfedges().add_or_get(v1New, h1To);
    auto edge1_2 = mesh.halfedges().add_or_get(h1From, v1New);
    
    auto edge2_1 = mesh.halfedges().add_or_get(v2New, h2To);
    auto edge2_2 = mesh.halfedges().add_or_get(h2From, v2New);
    
    //int countEdges3 = mesh.halfedges().count();
    //mesh.halfedges().remove_edge(h1);
    //mesh.halfedges().remove_edge(h2);


    //mesh.halfedges().remove_edge();
    //mesh.halfedges().remove_edge(h2.opposite());

    //Init Data
    
    planeMesh.planeMesh.setFace(newFace1, plane);
    planeMesh.planeMesh.setFace(newFace2, plane);
    planeMesh.planeMesh.setEdge(edgeFace1.edge(), iSectPlane);
    planeMesh.planeMesh.setEdge(edge1_1.edge(), planeH1);
    planeMesh.planeMesh.setEdge(edge1_2.edge(), planeH1);
    planeMesh.planeMesh.setEdge(edge2_1.edge(), planeH2);
    planeMesh.planeMesh.setEdge(edge2_2.edge(), planeH2);

    planeMesh.planeMesh.setHalfedge(edge1_1, signH1);
    planeMesh.planeMesh.setHalfedge(edge1_2, signH1);
    planeMesh.planeMesh.setHalfedge(edge2_1, signH2);
    planeMesh.planeMesh.setHalfedge(edge2_2, signH2);

    auto pos = planeMesh.planeMesh.pos(edgeFace1.next().vertex_to());
    auto sign = ob::classify_vertex(pos, iSectPlane);

    planeMesh.planeMesh.setHalfedge(edgeFace1, sign * -1);
    planeMesh.planeMesh.setHalfedge(edgeFace2, sign);

    int countEdges2 = mesh.halfedges().count();
    int countFaces2 = mesh.faces().count();

    invalidEdges = planeMesh.planeMesh.existsEdgesWithoutFace();
    TG_ASSERT(!invalidEdges);

    return std::make_tuple(newFace1, newFace2);
}

std::vector<pm::face_handle> splitPlanar(PlaneMeshInfo& planeMeshInfo1, PlaneMeshInfo& planeMeshInfo2, std::vector<EdgeData> iSectData) {
    PlaneMesh& planeMesh1 = planeMeshInfo1.planeMesh;
    PlaneMesh& planeMesh2 = planeMeshInfo2.planeMesh;
    std::vector<pm::face_handle> faces;
    std::vector<pm::halfedge_handle> halfEdgesToDelete;
    bool deleteFace = false;

    Plane plane = planeMesh1.face(planeMeshInfo1.face);
    planeMesh1.mesh().faces().remove(planeMeshInfo1.face);

    for (int i = 0; i < iSectData.size(); ++i) {
        TG_ASSERT(iSectData[i].state != PlanarState::ONE_EDGE && iSectData[i].state != PlanarState::UNKNOWN);
        if (iSectData[i].state == PlanarState::NON_INTERSECTING_OUT)
            continue;

        auto iSectEdge1 = iSectData[i].intersectionEdges.intersectionEdge1;
        auto iSectEdge2 = iSectData[i].intersectionEdges.intersectionEdge2;
        pm::halfedge_handle prevNewEdge;
        pm::halfedge_handle startEdgeNext;
        std::vector<pm::halfedge_handle> halfEdges = {};

        if (iSectData[i].state == PlanarState::NON_INTERSECTING_IN) {
            pm::vertex_handle start;
            pm::halfedge_handle hTmp;
            if (iSectData[i].intersectionEdges.intersectVertex1) {
                start = iSectEdge1.vertex_from();
                hTmp = iSectEdge1;
            }
            else {             
                start = splitHalfEdgeLowAPI(planeMesh1.mesh(), iSectEdge1);  
                hTmp = iSectEdge1.next();
                TG_ASSERT(hTmp.vertex_from() == start); //Only Test
            }

            pm::vertex_handle end;
            while (true) {
                halfEdges.push_back(hTmp);
                if (hTmp == iSectEdge2) {
                    if (iSectData[i].intersectionEdges.intersectVertex2) {
                        end = iSectEdge2.vertex_to();
                        startEdgeNext = iSectEdge2.next();
                    }
                    else {
                        end = splitHalfEdgeLowAPI(planeMesh1.mesh(), iSectEdge2);
                        startEdgeNext = iSectEdge2.next();
                        TG_ASSERT(startEdgeNext.vertex_from() == end);
                    }
                    prevNewEdge = planeMesh2.mesh().halfedges().add_or_get(end, start);
                    // Add Data
                    halfEdges.push_back(prevNewEdge);
                    break;
                }
                hTmp = hTmp.next();
            }
            faces.push_back(planeMesh1.mesh().faces().add(halfEdges));
        }

        if (iSectData[i].state == PlanarState::NON_INTERSECTING_IN) {
            pm::vertex_handle start = startEdgeNext.vertex_from();
            pm::vertex_handle end = startEdgeNext.vertex_from();
            auto hTmp = startEdgeNext;
            while (true) {
                halfEdges.push_back(hTmp);
                if (hTmp == iSectEdge2) {
                    if (iSectData[i].intersectionEdges.intersectVertex2) {
                        end = iSectEdge2.vertex_to();
                        startEdgeNext = iSectEdge2.next();
                    }
                    else {
                        end = splitHalfEdgeLowAPI(planeMesh1.mesh(), iSectEdge2);
                        startEdgeNext = iSectEdge2.next();
                        TG_ASSERT(startEdgeNext.vertex_from() == end); // Test
                    }
                    auto newVertex = splitHalfEdgeLowAPI(planeMesh1.mesh(), prevNewEdge);
                    prevNewEdge = planeMesh2.mesh().halfedges().add_or_get(end, newVertex);
                    // Add Data
                    halfEdges.push_back(prevNewEdge);
                    halfEdges.push_back(prevNewEdge.next());
                    TG_ASSERT(prevNewEdge.next().vertex_from() == newVertex); // Test
                    break;
                }
                hTmp = hTmp.next();
            }
            faces.push_back(planeMesh1.mesh().faces().add(halfEdges));
        }

        if (iSectData[i].state == PlanarState::TWO_EDGES) {
            pm::vertex_handle start;
            pm::vertex_handle end;
            pm::halfedge_handle hTmp;

            if (iSectData[i].intersectionEdges.intersectVertex1) {
                start = iSectEdge1.vertex_from();
                hTmp = iSectEdge1;
            }
            else {
                start = splitHalfEdgeLowAPI(planeMesh1.mesh(), iSectEdge1);
                hTmp = iSectEdge1.next();
                TG_ASSERT(hTmp.vertex_from() == start); //Only Test
            }

            while (true) {
                halfEdges.push_back(hTmp);
                if (hTmp == iSectEdge2) {
                    if (iSectData[i].intersectionEdges.intersectVertex2) {
                        end = iSectEdge2.vertex_to();
                        startEdgeNext = iSectEdge2.next();
                    }
                    else {
                        end = splitHalfEdgeLowAPI(planeMesh1.mesh(), iSectEdge2);
                        startEdgeNext = iSectEdge2.next();
                        TG_ASSERT(startEdgeNext.vertex_from() == end); // Test
                    }
                    prevNewEdge = planeMesh1.mesh().halfedges().add_or_get(end, start);
                    // Add Data
                    //planeMesh1.setEdge(prevNewEdge.)
                    halfEdges.push_back(prevNewEdge);
                    break;
                }
                hTmp = hTmp.next();
            }
            faces.push_back(planeMesh1.mesh().faces().add(halfEdges));
        }
    }
    return std::vector<pm::face_handle>();
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
    faces.push_back(planeMesh2.insertTriangle(triangle4 * scale));

    splitAccordingToIntersection(face1, faces, planeMesh1, planeMesh2);
    
    planeMesh1.checkAndComputePositions();
    planeMesh2.checkAndComputePositions();

    auto view = gv::view(planeMesh1.positions());
    gv::view(gv::lines(planeMesh1.positions()).line_width_world(10));
    gv::view(planeMesh2.positions());
    gv::view(gv::lines(planeMesh2.positions()).line_width_world(10));
}