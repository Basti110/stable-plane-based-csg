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
    std::vector<pm::face_handle> facesT1;
    std::vector<pm::face_handle> facesT2;
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

std::vector<pm::face_handle> split(PlaneMeshInfo& planeMesh, IntersectionEdges& intersectionEdges, Plane& iSectPlane) {
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

    return std::vector<pm::face_handle>{newFace1, newFace2};
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

/*static void assignEdgeHelper1(bool isVertex, IntersectionEdges& iSectEdges, pm::halfedge_handle& edge) {
    if (!foundFirst) {
        iSectEdges.intersectionEdge1 = edge;
        iSectEdges.intersectVertex1 = true;
    }
    else {
        iSectEdges.intersectionEdge2 = edge;
        iSectEdges.intersectVertex2 = true;
    }
}*/

void planeTriangle(Plane& plane, int sideLen, std::vector<tg::triangle3>& insertVec) {
    tg::vec3 upNormal = tg::normalize(tg::vec3(plane.to_dplane().normal));
    tg::vec3 upNormalFace = upNormal != tg::vec3(0, 0, -1) && upNormal != tg::vec3(0, 0, 1)  ? tg::vec3(0, 0, -1) : tg::vec3(0, 1, 0);
    tg::dplane3 dplane = plane.to_dplane();
    tg::vec3 normal = tg::vec3(dplane.normal);
    auto ortho_vec1 = tg::normalize(tg::cross(normal, upNormalFace));
    auto ortho_vec2 = tg::normalize(tg::cross(normal, ortho_vec1));
    auto plane_pos = normal * dplane.dis;
    auto pos_a = tg::pos3(plane_pos + 0.5 * sideLen * ortho_vec1 + 0.5 * sideLen * ortho_vec2);
    auto pos_b = tg::pos3(plane_pos + 0.5 * sideLen * ortho_vec1 - 0.5 * sideLen * ortho_vec2);
    auto pos_c = tg::pos3(plane_pos - 0.5 * sideLen * ortho_vec1 - 0.5 * sideLen * ortho_vec2);
    auto pos_d = tg::pos3(plane_pos - 0.5 * sideLen * ortho_vec1 + 0.5 * sideLen * ortho_vec2);
    insertVec.push_back({ pos_a , pos_b, pos_c });
    insertVec.push_back({ pos_c , pos_d, pos_a });
}

static IntersectionEdges getIntersectionEdges(PlaneMeshInfo& faceInfo, Plane& plane, int8_t planeSign) {
    std::vector<tg::triangle3> planePositions;
    planeTriangle(plane, 1000, planePositions); 
    auto faceColors1 = faceInfo.planeMesh.mesh().faces().make_attribute_with_default(tg::color3::white);
    faceColors1[faceInfo.face] = tg::color3::red;
    {
        faceInfo.planeMesh.checkAndComputePositions();
        auto view = gv::view(planePositions);
        gv::view(faceInfo.planeMesh.positions(), faceColors1);
        gv::view(gv::lines(faceInfo.planeMesh.positions()).line_width_world(10));
    }


    pm::face_handle face = faceInfo.face;
    int facCount = face.halfedges().count();
    PlaneMesh& planeMesh = faceInfo.planeMesh;
    IntersectionEdges iSectEdges;
    bool foundFirst = false;
 
    auto firstEdge = face.any_halfedge();
    auto firstPos = firstEdge.vertex_from();
    int8_t tmpSign = ob::classify_vertex(planeMesh.pos(firstPos), plane);


    if (tmpSign == 0) {
        iSectEdges.intersectionEdge1 = firstEdge;
        iSectEdges.intersectVertex1 = true;
        foundFirst = true;
    }
       
    auto hTmp = firstEdge;
    while (true) {
        hTmp = hTmp.next();

        auto det = planeMesh.pos(hTmp.vertex_from());
        int8_t sign = ob::classify_vertex(det, plane);

        TG_ASSERT(sign || tmpSign);
        if ((sign != tmpSign || sign == 0) && tmpSign != 0) {
            if (foundFirst) {
                iSectEdges.intersectionEdge2 = sign == 0 ? hTmp : hTmp.prev();
                iSectEdges.intersectVertex2 = sign == 0;
                break;
            }

            iSectEdges.intersectionEdge1 = sign == 0 ? hTmp : hTmp.prev();
            iSectEdges.intersectVertex1 = sign == 0;
            foundFirst = true;
        }
        tmpSign = sign;

        if (hTmp == firstEdge)
            TG_ASSERT(false && "No Intersection Edges found");
    }
    return iSectEdges;
}

//Schnitt Plane Erstes Face (direction * plane) zeigt nach auﬂen (signed distance = -1)
//Schnitt Plane Zweites Face (direction * plane) zeigt nach innen (signed distance = 1)
static std::tuple<pm::face_handle, pm::face_handle> splitFace(PlaneMeshInfo& faceInfo, Plane& plane, int8_t direction) {
    pm::halfedge_handle hTmp;
    pm::vertex_handle start;
    pm::vertex_handle end;
    pm::Mesh& mesh = faceInfo.planeMesh.mesh();

    IntersectionEdges iSectEdges = getIntersectionEdges(faceInfo, plane, 1);
    auto iSectEdge1 = iSectEdges.intersectionEdge1;
    auto iSectEdge2 = iSectEdges.intersectionEdge2;
    std::vector<pm::halfedge_handle> halfEdges = {};
    
    if (iSectEdges.intersectVertex1) {
        start = iSectEdge1.vertex_from();
        hTmp = iSectEdge1;
    }
    else {
        start = splitHalfEdgeLowAPI(mesh, iSectEdge1);
        Plane edgePlane = faceInfo.planeMesh.edge(iSectEdge1.edge());
        int8_t sign = faceInfo.planeMesh.halfedge(iSectEdge1);
        faceInfo.planeMesh.setEdge(iSectEdge1.next().edge(), edgePlane);
        faceInfo.planeMesh.setHalfedge(iSectEdge1.next(), sign);
        faceInfo.planeMesh.setHalfedge(iSectEdge1.next().opposite(), -sign);
        hTmp = iSectEdge1.next();
    }

    TG_ASSERT(hTmp.vertex_from() == start); //Only Test
    int8_t signToFace1 = ob::classify_vertex(faceInfo.planeMesh.pos(hTmp.vertex_to()), plane);
    int8_t wantedDirection = signToFace1 * direction;
    
    while (true) {
        if (hTmp == iSectEdge2) {
            break;
        }
        halfEdges.push_back(hTmp);
        hTmp = hTmp.next();
    }

    pm::halfedge_handle startEdgeNext;
    if (iSectEdges.intersectVertex2) {
        end = iSectEdge2.vertex_from();
        startEdgeNext = iSectEdge2;
    }
    else {
        end = splitHalfEdgeLowAPI(mesh, iSectEdge2);
        Plane edgePlane = faceInfo.planeMesh.edge(iSectEdge2.edge());
        int8_t sign = faceInfo.planeMesh.halfedge(iSectEdge2);
        faceInfo.planeMesh.setEdge(iSectEdge2.next().edge(), edgePlane);
        faceInfo.planeMesh.setHalfedge(iSectEdge2.next(), sign);
        faceInfo.planeMesh.setHalfedge(iSectEdge2.next().opposite(), -sign);
        halfEdges.push_back(iSectEdge2);
        startEdgeNext = iSectEdge2.next();
        TG_ASSERT(iSectEdge2.vertex_to() == end); //Only Test
        TG_ASSERT(startEdgeNext.vertex_from() == end); // Test
    }
    Plane facePlane = faceInfo.planeMesh.face(faceInfo.face);
    mesh.faces().remove(faceInfo.face);
    auto newEdge = mesh.halfedges().add_or_get(end, start);
    faceInfo.planeMesh.setEdge(newEdge.edge(), plane);
    // Add Data
    //planeMesh1.setEdge(prevNewEdge.)
    halfEdges.push_back(newEdge);
    

    auto face1 = mesh.faces().add(halfEdges);
    halfEdges.clear();

    hTmp = startEdgeNext;
    while (true) {
        halfEdges.push_back(hTmp);
        if (hTmp.vertex_to() == start) {
            break;
        }
        hTmp = hTmp.next();
    }
    halfEdges.push_back(newEdge.opposite());
    auto face2 = mesh.faces().add(halfEdges);
    faceInfo.planeMesh.setFace(face1, facePlane);
    faceInfo.planeMesh.setFace(face2, facePlane);

    //halfedge Attribut = 1 wenn signed distance = -1 und umgekehrt
    faceInfo.planeMesh.setHalfedge(newEdge, -1 * signToFace1);
    faceInfo.planeMesh.setHalfedge(newEdge.opposite(), signToFace1);
    
    if(wantedDirection == 1)
        return std::tuple<pm::face_handle, pm::face_handle>(face1, face2);
    else
        return std::tuple<pm::face_handle, pm::face_handle>(face2, face1);
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
        SharedTriIntersectPlanar isectPlanar = std::static_pointer_cast<TrianlgeIntersectionPlanar>(intersection);
        std::vector<pm::face_handle> faces1;
        auto edgeDataT1 = isectPlanar->getEdgeDataT1();
        auto edgeDataT2 = isectPlanar->getEdgeDataT2();
        auto face = planeMeshInfo2.face;
        Plane& planePrev = Plane();

        planeMeshInfo1.planeMesh.checkAndComputePositions();
        planeMeshInfo2.planeMesh.checkAndComputePositions();
        {
            auto faceColors1 = planeMeshInfo1.planeMesh.mesh().faces().make_attribute_with_default(tg::color3::white);
            faceColors1[planeMeshInfo1.face] = tg::color3::red;
            auto faceColors2 = planeMeshInfo2.planeMesh.mesh().faces().make_attribute_with_default(tg::color3::white);
            faceColors2[planeMeshInfo2.face] = tg::color3::blue;
            auto view = gv::view(planeMeshInfo1.planeMesh.positions(), faceColors1);
            gv::view(gv::lines(planeMeshInfo1.planeMesh.positions()).line_width_world(10));
            gv::view(planeMeshInfo2.planeMesh.positions(), faceColors2);
            gv::view(gv::lines(planeMeshInfo2.planeMesh.positions()).line_width_world(10));
        }


        for (auto& edgeData : edgeDataT1) {
            if (edgeData.state == PlanarState::ONE_EDGE_TO_IN 
                || edgeData.state == PlanarState::NON_INTERSECTING_IN 
                || edgeData.state == PlanarState::ONE_EDGE_TO_OUT
                || edgeData.state == PlanarState::TWO_EDGES ) {

                PlaneMeshInfo newFaceInfo = { planeMeshInfo2.planeMesh, face };
                Plane plane = planeMeshInfo1.planeMesh.edge(edgeData.edge);
                if (plane == planePrev)
                    continue;

                int8_t dir = planeMeshInfo1.planeMesh.halfedge(edgeData.edge);
                auto newFaces = splitFace(newFaceInfo, plane, dir);

                faces1.push_back(std::get<0>(newFaces));
                face = std::get<1>(newFaces);
                planePrev = plane;
            }
        }
        faces1.push_back(face);

        face = planeMeshInfo1.face;
        std::vector<pm::face_handle> faces2;
        planePrev = Plane();
        for (auto& edgeData : edgeDataT2) {
            if (edgeData.state == PlanarState::ONE_EDGE_TO_IN
                || edgeData.state == PlanarState::NON_INTERSECTING_IN
                || edgeData.state == PlanarState::ONE_EDGE_TO_OUT 
                || edgeData.state == PlanarState::TWO_EDGES ) {
                PlaneMeshInfo newFaceInfo = { planeMeshInfo1.planeMesh, face };
                Plane plane = planeMeshInfo2.planeMesh.edge(edgeData.edge);
                //Plane& planePrev = planeMeshInfo2.planeMesh.edge(edgeData.edge.prev());
                if (plane == planePrev)
                    continue;
                int8_t dir = planeMeshInfo2.planeMesh.halfedge(edgeData.edge);
                auto newFaces = splitFace(newFaceInfo, plane, dir);
                /*planeMeshInfo1.planeMesh.checkAndComputePositions();
                planeMeshInfo2.planeMesh.checkAndComputePositions();
                {
                    auto view = gv::view(planeMeshInfo1.planeMesh.positions());
                    gv::view(gv::lines(planeMeshInfo1.planeMesh.positions()).line_width_world(10));
                }*/

                //gv::view(planeMeshInfo2.planeMesh.positions());
                //gv::view(gv::lines(planeMeshInfo2.planeMesh.positions()).line_width_world(10));

                faces2.push_back(std::get<0>(newFaces));
                face = std::get<1>(newFaces);
                planePrev = plane;
            }
        }
        faces2.push_back(face);
        splitFaces.facesT1 = faces2;
        splitFaces.facesT2 = faces1;
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
            for (auto& triangle : splits.facesT1) {
                splitAccordingToIntersection(triangle, triangles, planeMesh1, planeMesh2);
            }

            /*for (auto& triangle : splits.facesT2) {
                splitAccordingToIntersection(triangle, triangles, planeMesh1, planeMesh2);
            }*/

            /*for (auto& triangle : splits.facesT1) {
                triangles.push_back(triangle);
            }*/

            for (auto& triangle : splits.facesT2) {
                triangles.push_back(triangle);
            }

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

    /*auto view = gv::view(planeMesh1.positions());
    gv::view(gv::lines(planeMesh1.positions()).line_width_world(10));
    gv::view(planeMesh2.positions());
    gv::view(gv::lines(planeMesh2.positions()).line_width_world(10));*/
}

TEST("Test::Cut_Triangle_Planar_1") {
    PlaneMesh planeMesh1;
    PlaneMesh planeMesh2;
    auto face1 = planeMesh1.insertPolygon({ 0, 0, -20 }, { 10, 0, 0 }, { -10, 0, 0 });
    auto face2 = planeMesh2.insertPolygon({ 0, 2, -7 }, { 0, -2, -9 }, { 0, -2, -5 });
    auto face3 = planeMesh2.insertPolygon({ -3, 2, -7 }, { -3, -2, -9 }, { -3, -2, -5 });
    auto face4 = planeMesh2.insertPolygon({ -5, 0, -15 }, { -10, 0, 10 }, { 0, 0, 10 });
    auto face5 = planeMesh2.insertPolygon({ 10, 0, -5 }, { 5, 0, 5 }, { 15, 0, 5 });
    auto face6 = planeMesh2.insertPolygon({ 0, 0, -20 }, { -20, 0, -20 }, { -10, 0, 0 });
    auto face7 = planeMesh2.insertPolygon({ -13, 0, -5 }, { -8, 0, 5 }, { -18, 0, 5 });
    auto face8 = planeMesh2.insertPolygon({ 0, 0, -30 }, { 5, 0, -20 }, { -5, 0, -20 });
    PlaneMeshInfo info = { planeMesh1, face1 };
    splitFace(info, planeMesh2.face(face2), 1);
    planeMesh1.checkAndComputePositions();

    /*auto view = gv::view(planeMesh1.positions());
    gv::view(gv::lines(planeMesh1.positions()).line_width_world(0.1));
    gv::view(planeMesh2.positions());
    gv::view(gv::lines(planeMesh2.positions()).line_width_world(0.1));*/
}

TEST("Test::Cut_Triangle_Planar_2") {
    tg::triangle<3, scalar_t> triangle1({ { 0, 0, -20 }, { 10, 0, 0 }, { -10, 0, 0 } });
    tg::triangle<3, scalar_t> triangle2({ { 0, 0, -15 }, { 5, 0, -5 }, { -5, 0, -5 } });
    tg::triangle<3, scalar_t> triangle3({ { 0, 0, -10 }, { 20, 0, -10 }, { 10, 0, -30 } });
    tg::triangle<3, scalar_t> triangle4({ { -5, 0, -15 }, { -10, 0, 10 }, { 0, 0, 10 } });
    tg::triangle<3, scalar_t> triangle5({ { 10, 0, -5 }, { 5, 0, 5 }, { 15, 0, 5 } });
    tg::triangle<3, scalar_t> triangle6({ { 0, 0, -20 }, { -20, 0, -20 }, { -10, 0, 0 } });
    tg::triangle<3, scalar_t> triangle7({ { -13, 0, -5 }, { -8, 0, 5 }, { -18, 0, 5 } });
    tg::triangle<3, scalar_t> triangle8({ { 0, 0, -30 }, { 5, 0, -20 }, { -5, 0, -20 } });

    PlaneMesh planeMesh1;
    PlaneMesh planeMesh2;

    std::vector<pm::face_handle> faces;
    scalar_t scale = 100;

    auto face1 = planeMesh1.insertTriangle(triangle1 * scale);
    faces.push_back(planeMesh2.insertTriangle(triangle2 * scale));
    faces.push_back(planeMesh2.insertTriangle(triangle3 * scale));
    faces.push_back(planeMesh2.insertTriangle(triangle4 * scale));
    faces.push_back(planeMesh2.insertTriangle(triangle5 * scale));
    faces.push_back(planeMesh2.insertTriangle(triangle6 * scale));
    faces.push_back(planeMesh2.insertTriangle(triangle7 * scale));
    faces.push_back(planeMesh2.insertTriangle(triangle8 * scale));

    splitAccordingToIntersection(face1, faces, planeMesh1, planeMesh2);

    planeMesh1.checkAndComputePositions();
    planeMesh2.checkAndComputePositions();

    {
        auto view = gv::view(planeMesh1.positions());
        gv::view(gv::lines(planeMesh1.positions()).line_width_world(10));
    }
    {
        auto view = gv::view(planeMesh2.positions());
        gv::view(gv::lines(planeMesh2.positions()).line_width_world(10));
    }

}