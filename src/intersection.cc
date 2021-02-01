#include <intersection.hh>

//Also checks if there is a vertex on the plane
//Eigenschaften:
//1: Wenn touching: edge 1 zeigt hin, edge 2 zeigt weg
//2: wenn Vertex auf baseplane: Intersection edge is die edge die von der Plane weg zeigt. 
IntersectionObject::IntersectionHandle IntersectionObject::planeBaseIntersection(const PlaneMesh& mesh1, pm::face_handle const& planeBase, 
    const PlaneMesh& mesh2, pm::face_handle const& polygon)
{

    int sign = 0;
    int index = 0;
    TG_ASSERT(!polygon.is_removed());
    TG_ASSERT(polygon.is_valid());
    //auto halfedges = polygon.halfedges().to_vector();// [](pm::halfedge_handle i) { return i; });
    std::vector<pm::halfedge_handle> halfedges;
    halfedges.reserve(5);
    for (auto& he: polygon.halfedges())
        halfedges.push_back(he);


    pm::halfedge_handle halfedgeHandles[2];
    bool vertexOnEdge[2];
    sign = mesh2.getSign(halfedges[0].vertex_from(), mesh1.face(planeBase));
    //sign = ob::classify_vertex(mesh2.pos(halfedges[0].vertex_from()), mesh1.face(planeBase));
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
        signTmp = mesh2.getSign(halfedges[i].vertex_to(), mesh1.face(planeBase));
        //signTmp = ob::classify_vertex(mesh2.pos(halfedges[i].vertex_to()), mesh1.face(planeBase));

        if (signTmp > 0)
            signPosAcc++;
        else if (signTmp < 0)
            signNegAcc++;

        if (signTmp == sign)
            continue;


        if (signTmp == 0 && wasZeroBefore) {
            continue;
        }

        if (signTmp == 0 && index == 2) {
            continue;
        }

        //2
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
    else {
        
        intersection.intersection = intersection_result::touching;
        //1
        if (startWithSign0) { 
            std::swap(halfedgeHandles[0], halfedgeHandles[1]);
            std::swap(vertexOnEdge[0], vertexOnEdge[1]);
        }
        if (halfedgeHandles[1] == halfedgeHandles[0].next()) {
            intersection.intersection = intersection_result::non_intersecting;
            return intersection;
        }
    }
        
    intersection.intersectionEdge1 = halfedgeHandles[0];
    intersection.intersectionEdge2 = halfedgeHandles[1];
    intersection.intersectVertex1 = vertexOnEdge[0];
    intersection.intersectVertex2 = vertexOnEdge[1];
    return intersection;
}

std::tuple<bool, bool> IntersectionObject::isInnerPointisEdgePoint(const std::vector<int8_t>& signDistanceToPlane) {
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

int8_t IntersectionObject::isInnerPoint(SubDet& pos, const std::vector<TrianlgeIntersectionPlanar::EdgeData>& edges, const PlaneMesh& edgesMesh) {
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

void IntersectionObject::classifyIntersectionEdges(const PlaneMesh& edgeMesh,
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
                edgeData.state = PlanarState::NON_INTERSECTING_IN;
            }
            else {
                edgeData.state = PlanarState::NON_INTERSECTING_OUT;
            }
        }
        else if (edgeData.state == PlanarState::ONE_EDGE) {
            if (innerPoint >= 0) {
                edgeData.state = PlanarState::ONE_EDGE_TO_IN;
                innerPoint = -1;
            }
            else {
                edgeData.state = PlanarState::ONE_EDGE_TO_OUT;
                innerPoint = 1;
            }
        }
    }
}

void IntersectionObject::classifyNotIntersectionEdges(const PlaneMesh& mesh1, const PlaneMesh& mesh2, TrianlgeIntersectionPlanar& intersectionPlanar) {
    classifyIntersectionEdges(mesh1, intersectionPlanar.getEdgeDataT1(), mesh2, intersectionPlanar.getEdgeDataT2());
    classifyIntersectionEdges(mesh2, intersectionPlanar.getEdgeDataT2(), mesh1, intersectionPlanar.getEdgeDataT1());
}

void IntersectionObject::setStateOnIntersection(TrianlgeIntersectionPlanar::PlanarState& state) {
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
//marked only for the case:  --|--->|
SharedTriIntersect IntersectionObject::getClassifiedEdgesFromSignChange(std::vector<int8_t>& signsFirstPoint,
    const std::vector<pm::halfedge_handle>& edges1,
    const std::vector<pm::halfedge_handle>& edges2)
{
    bool isIntersecting = false;
    auto intersectionPlanar = std::make_shared<TrianlgeIntersectionPlanar>(edges1, edges2);

    for (int i = 0; i < edges1.size(); ++i) {
        pm::vertex_handle qOld = edges1[i].vertex_from();
        pm::vertex_handle q = edges1[(i + 1) % edges1.size()].vertex_from();
        TrianlgeIntersectionPlanar::EdgeData& edge1Data = intersectionPlanar->getEdgeDataT1(i);

        for (int j = 0; j < edges2.size(); ++j) {         
            int8_t signStorageTmp = mPlaneMeshA.getSign(q, mPlaneMeshB.edge(edges2[j].edge())) * mPlaneMeshB.halfedge(edges2[j]);
            int8_t signBefore = signsFirstPoint[j];
            if (signStorageTmp != signBefore && (signStorageTmp != 0 || signBefore == -1) && (signBefore != 0 || signStorageTmp == -1)) {
                auto pos1 = edges2[j].vertex_from();
                auto pos2 = edges2[j].vertex_to();
                auto edge = mPlaneMeshA.findEdge(qOld, q);
                int8_t t1 = mPlaneMeshA.halfedge(edge);
                TG_ASSERT(t1 != 0);
                
                int8_t sign = mPlaneMeshB.getSign(pos1, mPlaneMeshA.edge(edge)) * mPlaneMeshA.halfedge(edge);
                int8_t sign2 = mPlaneMeshB.getSign(pos2, mPlaneMeshA.edge(edge)) * mPlaneMeshA.halfedge(edge);
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
                        if (signBefore == 0)
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

SharedTriIntersect IntersectionObject::handleCoplanarIntersection(const pm::face_handle& polygon1, const pm::face_handle& polygon2) {
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

    showFaces(mPlaneMeshA, polygon1, mPlaneMeshB, polygon2);
    auto result = getClassifiedEdgesFromSignChange(signStorage, edges1, edges2);
    return result;
}

bool IntersectionObject::handleTouchIntersection(const pm::face_handle& polygon1, const pm::face_handle& polygon2, SharedTriIntersectNonPlanar iSect) {
    bool isMeshA = polygon1.mesh == &(mPlaneMeshA.mesh());
    auto& t1 = isMeshA ? iSect->triangle1 : iSect->triangle2;
    auto& t2 = isMeshA ? iSect->triangle2 : iSect->triangle1;
    PlaneMesh& planeMeshA = isMeshA ? mPlaneMeshA : mPlaneMeshB;
    PlaneMesh& planeMeshB = isMeshA ? mPlaneMeshB : mPlaneMeshA;

    pm::halfedge_handle edge = t2.intersectionEdge1;
    //SubDet subdet = planeMeshB.pos(edge.vertex_to());   
    int8_t sign1 = planeMeshB.getSign(edge.vertex_to(), planeMeshA.edge(t1.intersectionEdge1.edge()));
    sign1 *= planeMeshA.halfedge(t1.intersectionEdge1);
    int8_t sign2 = planeMeshB.getSign(edge.vertex_to(), planeMeshA.edge(t1.intersectionEdge2.edge()));
    sign2 *= planeMeshA.halfedge(t1.intersectionEdge2);
    edge = edge.next();


    if (sign1 != 0 && sign2 != 0 && sign1 != sign2) {
        int8_t sign1Tmp = 0;
        int8_t sign2Tmp = 0;
        while (edge != t2.intersectionEdge2) {
            //subdet = planeMeshB.pos(edge.vertex_to());
            int8_t sign1Tmp = planeMeshB.getSign(edge.vertex_to(), planeMeshA.edge(t1.intersectionEdge1.edge()));
            sign1Tmp *= planeMeshA.halfedge(t1.intersectionEdge1);
            int8_t sign2Tmp = planeMeshB.getSign(edge.vertex_to(), planeMeshA.edge(t1.intersectionEdge2.edge()));
            sign2Tmp *= planeMeshA.halfedge(t1.intersectionEdge2);
            TG_ASSERT(sign1Tmp || sign2Tmp);

            if (sign1 != sign1Tmp || sign2 != sign2Tmp)
                break;

            edge = edge.next();
        }

        if (edge == t2.intersectionEdge2)
            return false;

        t2.intersectionEdge1 = edge.prev();
        if (sign1 != sign1Tmp && sign2 != sign2Tmp) {
            sign1 = -sign1;
            sign2 = -sign2;
        }
        else {
            sign1 = sign1Tmp;
            sign2 = sign2Tmp;
        }
    }

    edge = edge.next();
    if (sign1 == 0 || sign2 == 0 || (sign1 == -1 && sign2 == -1)) {
        int8_t sign1Tmp = 0;
        int8_t sign2Tmp = 0;
        while (edge != t2.intersectionEdge2) {
            //subdet = planeMeshB.pos(edge.vertex_to());
            int8_t sign1Tmp = planeMeshB.getSign(edge.vertex_to(), planeMeshA.edge(t1.intersectionEdge1.edge()));
            sign1Tmp *= planeMeshA.halfedge(t1.intersectionEdge1);
            int8_t sign2Tmp = planeMeshB.getSign(edge.vertex_to(), planeMeshA.edge(t1.intersectionEdge2.edge()));
            sign2Tmp *= planeMeshA.halfedge(t1.intersectionEdge2);
            if(!(sign1Tmp || sign2Tmp))
                return false;
            TG_ASSERT(sign1Tmp || sign2Tmp);

            if (sign1 != sign1Tmp) {
                if (sign1 == 0 && sign1Tmp == 1)
                    return false;

                if (sign1 != 0) {
                    t2.intersectionEdge2 = edge.next();
                    break;
                }
            }

            if (sign2 != sign2Tmp) {
                if (sign2 == 0 && sign2Tmp == 1)
                    return false;

                if (sign2 != 0) {
                    t2.intersectionEdge2 = edge.next();
                    break;
                }
            }
            edge = edge.next();
        }
    }  
    else {
        t2.intersectionEdge2 = edge;
    }
    if (t2.intersectionEdge2 == t2.intersectionEdge1.next())
        return false;
    return true;
}

SharedTriIntersect IntersectionObject::handleIntersection(const pm::face_handle& polygon1, const pm::face_handle& polygon2, IntersectionHandle& intersection1, IntersectionHandle& intersection2) {
    
    const Plane& basePlane1 = mPlaneMeshA.getPlane(polygon1);
    const Plane& basePlane2 = mPlaneMeshB.getPlane(polygon2);

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

    if (intersection1.intersection == intersection_result::touching && intersection2.intersection == intersection_result::touching) {
        nonPlanarIntersection->state = TrianlgeIntersectionNonPlanar::NonPlanarState::TOUCHING;
        /*if(handleTouchIntersection(polygon1, polygon2, nonPlanarIntersection))
            if(handleTouchIntersection(polygon2, polygon1, nonPlanarIntersection))
                return nonPlanarIntersection;
        return std::make_shared<TrianlgeIntersection>();*/
    }       
    else if (intersection2.intersection == intersection_result::touching)
        nonPlanarIntersection->state = TrianlgeIntersectionNonPlanar::NonPlanarState::TOUCHING_1_ON_2;
    else if (intersection1.intersection == intersection_result::touching)
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

    if (!(sign1 != 0 || sign2 != 0))
        showFaces(mPlaneMeshA, polygon1, mPlaneMeshB, polygon2);

    TG_ASSERT(sign1 != 0 || sign2 != 0);

    if (sharedPoint && sign2 == 0)
        return nonPlanarIntersection;

    if (sign == sign2 && sign2 != 0)
        return nonPlanarIntersection;

    return std::make_shared<TrianlgeIntersection>(); //overlapInterval(IntersectionHandle& intersection);
}

void IntersectionObject::showFaces(PlaneMesh& mesh1, const pm::face_handle& polygon1, PlaneMesh& mesh2, const pm::face_handle& polygon2) {
    mesh1.checkAndComputePositions();
    mesh2.checkAndComputePositions();

    /*auto face1Mask = mesh1.mesh().faces().make_attribute_with_default(false);
    auto face2Mask = mesh2.mesh().faces().make_attribute_with_default(false);
    face1Mask[polygon1] = true;
    face2Mask[polygon2] = true;

    tg::aabb3 viewBox;
    viewBox.min = tg::pos3(mesh1.posInt(polygon1.any_vertex()));
    viewBox.max = tg::pos3(mesh2.posInt(polygon1.any_vertex()));*/
    auto faceColors1 = mesh1.mesh().faces().make_attribute_with_default(tg::color3::white);
    faceColors1[polygon1] = tg::color3::red;
    auto faceColors2 = mesh2.mesh().faces().make_attribute_with_default(tg::color3::white);
    faceColors2[polygon2] = tg::color3::blue;
    auto view = gv::view(mesh1.positions(), faceColors1);
    gv::view(gv::lines(mesh1.positions()).line_width_world(10000));
    gv::view(mesh2.positions(), faceColors2);
    gv::view(gv::lines(mesh2.positions()).line_width_world(10000));

    /*pm::Mesh mesh;
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

    auto view = gv::view(pos);
    gv::view(gv::lines(pos).line_width_world(10000));*/
    //gv::view(mesh2.positions(), gv::masked(face2Mask), viewBox);
}