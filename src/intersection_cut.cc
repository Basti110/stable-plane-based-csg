#include <intersection_cut.hh>

pm::vertex_handle IntersectionCut::splitHalfEdgeLowAPI(pm::Mesh& mesh, pm::halfedge_handle& h) {
    return mesh.handle_of(pm::low_level_api(&mesh).halfedge_split(h.idx));
}

pm::face_handle IntersectionCut::addFaceFromCycleAndStopPoint(pm::Mesh& mesh, pm::halfedge_handle& startEdge, pm::vertex_handle stopVertex) {
    auto secondVertex = startEdge.vertex_from();
    std::vector<pm::halfedge_handle> hEdges;
    auto hTmp = startEdge;

    while (true) {
        hEdges.push_back(hTmp);
        if (hTmp.vertex_to() == stopVertex) {
            hEdges.push_back(mesh.halfedges().add_or_get(stopVertex, secondVertex));
            break;
        }
        hTmp = hTmp.next();
    }
    return mesh.faces().add(hEdges);
}

void IntersectionCut::planeToTriangles(Plane& plane, int sideLen, std::vector<tg::triangle3>& insertVec) {
    tg::vec3 upNormal = tg::normalize(tg::vec3(plane.to_dplane().normal));
    tg::vec3 upNormalFace = upNormal != tg::vec3(0, 0, -1) && upNormal != tg::vec3(0, 0, 1) ? tg::vec3(0, 0, -1) : tg::vec3(0, 1, 0);
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

IntersectionEdges IntersectionCut::getIntersectionEdges(PlaneMeshInfo& faceInfo, Plane& plane, int8_t planeSign) {
    pm::face_handle face = faceInfo.face;
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
std::tuple<pm::face_handle, pm::face_handle> IntersectionCut::splitFace(PlaneMeshInfo& faceInfo, Plane& plane, int8_t direction) {
    pm::halfedge_handle hTmp;
    pm::vertex_handle start;
    pm::vertex_handle end;
    pm::Mesh& mesh = faceInfo.planeMesh.mesh();

    //Todo sign
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

    if (wantedDirection == 1)
        return std::tuple<pm::face_handle, pm::face_handle>(face1, face2);
    else
        return std::tuple<pm::face_handle, pm::face_handle>(face2, face1);
}
//TODO: Remove Debug entries
std::vector<pm::face_handle> IntersectionCut::split(PlaneMeshInfo& planeMesh, IntersectionEdges& intersectionEdges, Plane& iSectPlane) {
    //Only Test
    TG_ASSERT(planeMesh.planeMesh.allFacesAreValid());

    pm::Mesh& mesh = planeMesh.planeMesh.mesh();
    int countEdges1 = mesh.halfedges().count();
    //int countFaces1 = mesh.faces().count();
    auto h1 = intersectionEdges.intersectionEdge1;
    auto h2 = intersectionEdges.intersectionEdge2;
    auto isectVertex1 = intersectionEdges.intersectVertex1;
    auto isectVertex2 = intersectionEdges.intersectVertex2;

    auto h1From = h1.vertex_from();
    auto h1To = h1.vertex_to();
    auto h2From = h2.vertex_from();
    auto h2To = h2.vertex_to();

    pm::vertex_handle v1New;
    pm::vertex_handle v2New;

    // split edges or not
    if (intersectionEdges.intersectVertex1) {
        v1New = h1.vertex_from();
        h1 = h1.prev();
    }
    else
        v1New = splitHalfEdgeLowAPI(mesh, h1);

    if (intersectionEdges.intersectVertex2) {
        v2New = h2.vertex_from();
        h2 = h2.prev();
    }
    else
        v2New = splitHalfEdgeLowAPI(mesh, h2);

    //auto v1New = splitHalfEdgeLowAPI(mesh, h1);
    //auto v2New = splitHalfEdgeLowAPI(mesh, h2);
    //Oder andersrum? vertex_to == new?
    TG_ASSERT(h1.vertex_to() == v1New);
    TG_ASSERT(h2.vertex_to() == v2New);

    //Only Test
    bool testFace = planeMesh.planeMesh.faceHasValidHalfEdges(planeMesh.face);

    Plane plane = planeMesh.planeMesh.face(planeMesh.face);
    //Only Test
    if (planeMesh.face.idx.value == 3108)
        int k = 10;

    mesh.faces().remove(planeMesh.face);
    int countEdges4 = mesh.halfedges().count();
    //TG_ASSERT(planeMesh.planeMesh.allFacesAreValid());

    //bool faceTest1 = h1.opposite().face().is_valid();
    //bool faceTest2 = h2.opposite().face().is_valid();
    auto h1Next = h1.next();
    auto h2Next = h2.next();
    auto newFace1 = addFaceFromCycleAndStopPoint(mesh, h1Next, v2New);
    int countEdges5 = mesh.halfedges().count();
    auto newFace2 = addFaceFromCycleAndStopPoint(mesh, h2Next, v1New);
    int countEdges6 = mesh.halfedges().count();
    TG_ASSERT(planeMesh.planeMesh.allFacesAreValid());
    //faceTest1 = h1.opposite().face().is_valid();
    //faceTest2 = h2.opposite().face().is_valid();
    TG_ASSERT(newFace1.is_valid());
    TG_ASSERT(newFace2.is_valid());

    bool invalidEdges = planeMesh.planeMesh.existsEdgesWithoutFace();
    TG_ASSERT(!invalidEdges);

    //Todo: Remove data in attributes
    //Set Planes on Faces
    planeMesh.planeMesh.setFace(newFace1, plane);
    planeMesh.planeMesh.setFace(newFace2, plane);
    //Set Planes on Edges if needed

    if (!intersectionEdges.intersectVertex1) {
        Plane planeH1 = planeMesh.planeMesh.edge(h1);
        auto edge1_1 = mesh.halfedges().add_or_get(v1New, h1To);
        auto edge1_2 = mesh.halfedges().add_or_get(h1From, v1New);
        planeMesh.planeMesh.setEdge(edge1_1.edge(), planeH1);
        planeMesh.planeMesh.setEdge(edge1_2.edge(), planeH1);

        int8_t signH1 = planeMesh.planeMesh.halfedge(h1);
        int8_t signH1Opp = planeMesh.planeMesh.halfedge(h1.opposite());
        planeMesh.planeMesh.setHalfedge(edge1_1, signH1);
        planeMesh.planeMesh.setHalfedge(edge1_2, signH1);
        planeMesh.planeMesh.setHalfedge(edge1_1.opposite(), signH1Opp); //<= bug line
        planeMesh.planeMesh.setHalfedge(edge1_2.opposite(), signH1Opp);

        if (planeMesh.planeMesh.id() == mMeshA->id()) {
            mIntersectionEdgesMarkerA[edge1_1] = mIntersectionEdgesMarkerA[edge1_2];
        }
        else {
            mIntersectionEdgesMarkerB[edge1_1] = mIntersectionEdgesMarkerB[edge1_2];
        }
    }

    if (!intersectionEdges.intersectVertex2) {
        Plane planeH2 = planeMesh.planeMesh.edge(h2);
        auto edge2_1 = mesh.halfedges().add_or_get(v2New, h2To);
        auto edge2_2 = mesh.halfedges().add_or_get(h2From, v2New);
        planeMesh.planeMesh.setEdge(edge2_1.edge(), planeH2);
        planeMesh.planeMesh.setEdge(edge2_2.edge(), planeH2);

        int8_t signH2 = planeMesh.planeMesh.halfedge(h2);
        int8_t signH2Opp = planeMesh.planeMesh.halfedge(h2.opposite());
        planeMesh.planeMesh.setHalfedge(edge2_1, signH2);
        planeMesh.planeMesh.setHalfedge(edge2_2, signH2);
        planeMesh.planeMesh.setHalfedge(edge2_1.opposite(), signH2Opp);
        planeMesh.planeMesh.setHalfedge(edge2_2.opposite(), signH2Opp);

        if (planeMesh.planeMesh.id() == mMeshA->id()) {
            mIntersectionEdgesMarkerA[edge2_1] = mIntersectionEdgesMarkerA[edge2_2];
        }
        else {
            mIntersectionEdgesMarkerB[edge2_1] = mIntersectionEdgesMarkerB[edge2_2];
        }
    }


    //int countEdges3 = mesh.halfedges().count();
    //mesh.halfedges().remove_edge(h1);
    //mesh.halfedges().remove_edge(h2);


    //mesh.halfedges().remove_edge();
    //mesh.halfedges().remove_edge(h2.opposite());

    //Init Data

    auto edgeFace1 = mesh.halfedges().add_or_get(v2New, v1New);
    auto edgeFace2 = mesh.halfedges().add_or_get(v1New, v2New);

    planeMesh.planeMesh.setEdge(edgeFace1.edge(), iSectPlane);
    auto pos = planeMesh.planeMesh.pos(edgeFace1.next().vertex_to());
    auto sign = ob::classify_vertex(pos, iSectPlane);

    planeMesh.planeMesh.setHalfedge(edgeFace1, sign * -1);
    planeMesh.planeMesh.setHalfedge(edgeFace2, sign);

    int countEdges2 = mesh.halfedges().count();
    int countFaces2 = mesh.faces().count();

    if (planeMesh.planeMesh.id() == mMeshA->id()) {
        mIntersectionEdgesMarkerA[edgeFace1.edge()] = true;
    }
    else {
        TG_ASSERT(planeMesh.planeMesh.id() == mMeshB->id());
        mIntersectionEdgesMarkerB[edgeFace1.edge()] = true;
    }
    invalidEdges = planeMesh.planeMesh.existsEdgesWithoutFace();
    TG_ASSERT(!invalidEdges);

    //Only Test
    bool v1 = planeMesh.planeMesh.faceHasValidHalfEdges(newFace1);
    bool v2 = planeMesh.planeMesh.faceHasValidHalfEdges(newFace2);

    if (!v1 || !v2)
        showFaces(planeMesh.planeMesh, std::vector<pm::face_handle>{newFace1, newFace2});

    return std::vector<pm::face_handle>{newFace1, newFace2};
}

bool IntersectionCut::edgeIsSplitEdge(PlanarState& state) {
    return state == PlanarState::ONE_EDGE_TO_IN || state == PlanarState::NON_INTERSECTING_IN
        || state == PlanarState::ONE_EDGE_TO_OUT || state == PlanarState::TWO_EDGES;
}

std::vector<pm::face_handle> IntersectionCut::splitFacesWithAllIntersectionEdges(PlaneMeshInfo& planeMeshInfo1, PlaneMeshInfo& planeMeshInfo2, std::vector<TrianlgeIntersectionPlanar::EdgeData>& edgeData) {
    Plane& planePrev = Plane();
    auto face = planeMeshInfo2.face;
    std::vector<pm::face_handle> faces;
    for (auto& edgeData : edgeData) {
        if (edgeIsSplitEdge(edgeData.state)) {
            PlaneMeshInfo newFaceInfo = { planeMeshInfo2.planeMesh, face };
            Plane plane = planeMeshInfo1.planeMesh.edge(edgeData.edge);
            if (plane == planePrev)
                continue;

            int8_t dir = planeMeshInfo1.planeMesh.halfedge(edgeData.edge);
            auto newFaces = splitFace(newFaceInfo, plane, dir);

            faces.push_back(std::get<0>(newFaces));
            face = std::get<1>(newFaces);
            planePrev = plane;
        }
    }
    faces.push_back(face);
    return faces;
}

NewFaces IntersectionCut::splitPlanar(SharedTriIntersectPlanar& isectPlanar, PlaneMeshInfo& planeMeshInfo1, PlaneMeshInfo& planeMeshInfo2) {
    NewFaces splitFaces;
    auto edgeDataT1 = isectPlanar->getEdgeDataT1();
    auto edgeDataT2 = isectPlanar->getEdgeDataT2();

    std::vector<pm::face_handle> faces1 = splitFacesWithAllIntersectionEdges(planeMeshInfo1, planeMeshInfo2, edgeDataT1);
    std::vector<pm::face_handle> faces2 = splitFacesWithAllIntersectionEdges(planeMeshInfo2, planeMeshInfo1, edgeDataT2);

    splitFaces.facesT1 = faces2;
    splitFaces.facesT2 = faces1;
    return splitFaces;
}

NewFaces IntersectionCut::split(SharedTriIntersect& intersection, PlaneMeshInfo& planeMeshInfo1, PlaneMeshInfo& planeMeshInfo2) {
    TG_ASSERT(intersection->intersectionState != TrianlgeIntersection::IntersectionState::NON_INTERSECTING);
    NewFaces splitFaces;

    /*TG_ASSERT(planeMeshInfo1.planeMesh.allHalfEdgesAreValid());
    TG_ASSERT(planeMeshInfo2.planeMesh.allHalfEdgesAreValid());
    TG_ASSERT(planeMeshInfo1.planeMesh.allFacesAreValid());
    TG_ASSERT(planeMeshInfo2.planeMesh.allFacesAreValid());*/

    if (intersection->intersectionState == TrianlgeIntersection::IntersectionState::NON_PLANAR) {
        SharedTriIntersectNonPlanar isectNonPlanar = std::static_pointer_cast<TrianlgeIntersectionNonPlanar>(intersection);
        //TODO: nicht intuitiv
        if (isectNonPlanar->state == TrianlgeIntersectionNonPlanar::NonPlanarState::PROPER_INTERSECTION) {
            Plane intersectionPlane1 = planeMeshInfo2.planeMesh.face(planeMeshInfo2.face);
            Plane intersectionPlane2 = planeMeshInfo1.planeMesh.face(planeMeshInfo1.face);
            splitFaces.facesT1 = split(planeMeshInfo1, isectNonPlanar->triangle1, intersectionPlane1);
            splitFaces.facesT2 = split(planeMeshInfo2, isectNonPlanar->triangle2, intersectionPlane2);
        }
        else if (isectNonPlanar->state == TrianlgeIntersectionNonPlanar::NonPlanarState::TOUCHING_1_ON_2) {
            Plane intersectionPlane2 = planeMeshInfo1.planeMesh.face(planeMeshInfo1.face);
            splitFaces.facesT1 = std::vector<pm::face_handle>{ planeMeshInfo1.face };
            splitFaces.facesT2 = split(planeMeshInfo2, isectNonPlanar->triangle2, intersectionPlane2);
        }
        else if (isectNonPlanar->state == TrianlgeIntersectionNonPlanar::NonPlanarState::TOUCHING_2_ON_1) {
            Plane intersectionPlane1 = planeMeshInfo2.planeMesh.face(planeMeshInfo2.face);
            splitFaces.facesT1 = split(planeMeshInfo1, isectNonPlanar->triangle1, intersectionPlane1);
            splitFaces.facesT2 = std::vector<pm::face_handle>{ planeMeshInfo2.face };
        }
    }
    else if (intersection->intersectionState == TrianlgeIntersection::IntersectionState::PLANAR) {
        SharedTriIntersectPlanar isectPlanar = std::static_pointer_cast<TrianlgeIntersectionPlanar>(intersection);
        splitFaces = splitPlanar(isectPlanar, planeMeshInfo1, planeMeshInfo2);
    }
    /*TG_ASSERT(planeMeshInfo1.planeMesh.allHalfEdgesAreValid());
    TG_ASSERT(planeMeshInfo2.planeMesh.allHalfEdgesAreValid());
    TG_ASSERT(planeMeshInfo1.planeMesh.allFacesAreValid());
    TG_ASSERT(planeMeshInfo2.planeMesh.allFacesAreValid());*/
    return splitFaces;
}


IntersectionCut::IntersectionCut(PlaneMesh* mMeshA, PlaneMesh* mMeshB) : mMeshA(mMeshA), mMeshB(mMeshB) {
    mIntersectionEdgesMarkerA = mMeshA->mesh().edges().make_attribute_with_default(false);
    mIntersectionEdgesMarkerB = mMeshB->mesh().edges().make_attribute_with_default(false);
}

IntersectionCut::IntersectionCut() {

}

SharedTriIntersect IntersectionCut::getIntersectionStateWithTimer(pm::face_handle& t1, pm::face_handle& t2) {
    this->intersectionCount++;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    auto intersection = IsectOb(*mMeshA, *mMeshB).intersect<geometry128>(t1, t2);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    auto nSeconds = std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count();
    this->intersectionTimeCount += nSeconds;
    return intersection;
}

NewFaces IntersectionCut::splitnWithTimer(pm::face_handle& t1, pm::face_handle& t2, SharedTriIntersect i) {
    this->splitCount++;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    auto splits = split(i, PlaneMeshInfo{ *mMeshA, t1 }, PlaneMeshInfo{ *mMeshB, t2 });
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    auto nSeconds = std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count();
    this->splitTimeCount += nSeconds;
    return splits;
}

std::vector<pm::face_handle> IntersectionCut::splitAccordingToIntersection(pm::face_handle triangle, std::vector<pm::face_handle>& triangles)
{
    pm::face_handle t2;
    std::vector<pm::face_handle> trianglesTMP = triangles;
    int count = 0;

    while (trianglesTMP.size() > 0) {
        pm::face_handle t2 = trianglesTMP[0];
        //showFaces(triangle, t2);

        auto intersection = getIntersectionStateWithTimer(triangle, t2);
        trianglesTMP.erase(trianglesTMP.begin() + 0);

        if (intersection->intersectionState != TrianlgeIntersection::IntersectionState::NON_INTERSECTING) {
            triangles.erase(triangles.begin() + count);
            auto splits = splitnWithTimer(triangle, t2, intersection);

            if (mLookupFacesA.count(triangle.idx.value) == 0 && splits.facesT1.size() > 1) {
                mLookupFacesA[triangle.idx.value] = splits.facesT1;
                TG_ASSERT(mLookupFacesA.count(triangle.idx.value) == 1);
            }

            if (mLookupFacesB.count(t2.idx.value) == 0 && splits.facesT2.size() > 1) {
                mLookupFacesB[t2.idx.value] = splits.facesT2;
                TG_ASSERT(mLookupFacesB.count(t2.idx.value) == 1);
            }

            for (auto& tri : splits.facesT1) {
                if (tri.is_valid()) {
                    splitAccordingToIntersection(tri, trianglesTMP);
                }
            }

            triangles.erase(triangles.begin() + count, triangles.end());
            triangles.insert(triangles.end(), trianglesTMP.begin(), trianglesTMP.end());

            for (auto& triangle : splits.facesT2) {
                if (triangle.is_valid()) {
                    triangles.push_back(triangle);
                }
            }

            return splits.facesT1;
        }
        count++;
    }
    return std::vector<pm::face_handle>{triangle};
}

void IntersectionCut::showFaces(PlaneMesh& planeMesh, std::vector<pm::face_handle> faces) {
    mMeshA->checkAndComputePositions();
    mMeshB->checkAndComputePositions();

    auto faceColors = mMeshA->mesh().faces().make_attribute_with_default(tg::color3::white);
    for (auto t : faces)
        faceColors[t] = tg::color3::red;

    PlaneMesh* otherPlaneMesh = planeMesh.id() == mMeshA->id() ? mMeshB : mMeshA;

    auto view = gv::view(planeMesh.positions(), faceColors);
    gv::view(gv::lines(planeMesh.positions()).line_width_world(10000));
    gv::view(otherPlaneMesh->positions());
    gv::view(gv::lines(otherPlaneMesh->positions()).line_width_world(10000));
    return;
}

void IntersectionCut::showFaces(pm::face_handle t1, pm::face_handle t2) {
    mMeshA->checkAndComputePositions();
    mMeshB->checkAndComputePositions();

    auto faceColors1 = mMeshA->mesh().faces().make_attribute_with_default(tg::color3::white);
    faceColors1[t1] = tg::color3::red;
    auto faceColors2 = mMeshB->mesh().faces().make_attribute_with_default(tg::color3::white);
    faceColors2[t2] = tg::color3::blue;
    auto view = gv::view(mMeshA->positions(), faceColors1);
    gv::view(gv::lines(mMeshA->positions()).line_width_world(10000));
    gv::view(mMeshB->positions(), faceColors2);
    gv::view(gv::lines(mMeshB->positions()).line_width_world(10000));
    return;
}

void IntersectionCut::checkLookupAndSplit(std::vector<pm::face_handle>& faces1, std::vector<pm::face_handle>& faces2, pm::face_handle& index) {
    if (mLookupFacesA.count(index.idx.value) == 0) {
        TG_ASSERT(index.is_valid());
        TG_ASSERT(!index.is_removed());
        auto newFaces = splitAccordingToIntersection(index, faces2);
        faces1.insert(faces1.end(), newFaces.begin(), newFaces.end());
        return;
    }

    std::vector<pm::face_handle>& faceVec = mLookupFacesA[index.idx.value];
    for (pm::face_handle& face : faceVec) {
        checkLookupAndSplit(faces1, faces2, face);
    }
}

void IntersectionCut::fillFacesFromLookupBInVec(std::vector<pm::face_handle>& vec, pm::face_handle& index) {
    if (mLookupFacesB.count(index.idx.value) == 0) {
        TG_ASSERT(index.is_valid());
        TG_ASSERT(!index.is_removed());
        vec.push_back(index);
        return;
    }

    std::vector<pm::face_handle>& faceVec = mLookupFacesB[index.idx.value];
    for (pm::face_handle& face : faceVec) {
        fillFacesFromLookupBInVec(vec, face);
    }
}

void IntersectionCut::cutPolygons(std::vector<pm::face_index>& facesMeshA, std::vector<pm::face_index>& facesMeshB) {
    TG_ASSERT(mMeshA && mMeshB);

    std::vector<pm::face_handle> faces1;
    std::vector<pm::face_handle> faces2;

    //TODO: Cann remove first IF
    for (pm::face_index& face2Index : facesMeshB) {
        fillFacesFromLookupBInVec(faces2, face2Index.of(mMeshB->mesh()));
    }

    //Todo: the same as above
    for (pm::face_index& face1Index : facesMeshA) {
        checkLookupAndSplit(faces1, faces2, face1Index.of(mMeshA->mesh()));
    }

    facesMeshA.clear();
    facesMeshB.clear();
    facesMeshA.reserve(faces1.size());
    facesMeshB.reserve(faces2.size());

    //TODO: Only use face indices
    for (int i = 0; i < faces1.size(); ++i)
        facesMeshA.push_back(faces1[i].idx);

    for (int i = 0; i < faces2.size(); ++i)
        facesMeshB.push_back(faces2[i].idx);
}