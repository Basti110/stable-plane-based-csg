#include <intersection_cut.hh>
#include <face_component_finder.hh>
#include <polymesh/algorithms/deduplicate.hh>
#include <component_categorization.hh>
#include <octree.hh>

TEST("Test::Cut_Triangle_Normal_Marker_Case1") {

    scalar_t scale = 100;
    std::vector<pos_t> polygon1(4);
    polygon1[0] = pos_t{ -10, 0, -20 } * scale;
    polygon1[1] = pos_t{ -10, 0, 0 } * scale;
    polygon1[2] = pos_t{ 10, 0, 0 } * scale;
    polygon1[3] = pos_t{ 10, 0, -20 } * scale;

    std::vector<pos_t> polygon2(3);
    polygon2[0] = pos_t{ 0, 5, -15 } * scale;
    polygon2[1] = pos_t{ 3, 0, -10 } * scale;
    polygon2[2] = pos_t{ -3, 0, -10 } * scale;

    std::vector<pos_t> polygon3(3);
    polygon3[0] = pos_t{ 3, 0, -10 } * scale;
    polygon3[1] = pos_t{ -3, 0, -10 } * scale;
    polygon3[2] = pos_t{ 0, -5, -5 } * scale;

    std::vector<pos_t> polygon4(3);
    polygon4[0] = pos_t{ 15, 7, -10 } * scale;
    polygon4[1] = pos_t{ 10, 0, -5 } * scale;
    polygon4[2] = pos_t{ 10, 0, -15 } * scale;

    std::vector<pos_t> polygon5(3);
    polygon5[0] = pos_t{ 5, -7, -10 } * scale;
    polygon5[1] = pos_t{ 10, 0, -5 } * scale;
    polygon5[2] = pos_t{ 10, 0, -15 } * scale;

    //Gerade 1
    std::vector<pos_t> polygon6(4);
    polygon6[0] = pos_t{ -15, 5, -10 } *scale;
    polygon6[1] = pos_t{ -5, 5, -10 } * scale;
    polygon6[2] = pos_t{ -5, -5, -10 } * scale;
    polygon6[3] = pos_t{ -15, -5, -10 } * scale;

    std::vector<pos_t> polygon7(4);
    polygon7[0] = pos_t{ -5, 5, -10 } *scale;
    polygon7[1] = pos_t{ 5, 5, -10 } *scale;
    polygon7[2] = pos_t{ 5, -5, -10 } *scale;
    polygon7[3] = pos_t{ -5, -5, -10 } *scale;

    std::vector<pos_t> polygon8(4);
    polygon8[0] = pos_t{ 5, 5, 5 } * scale;
    polygon8[1] = pos_t{ 5, 5, -10 } * scale;
    polygon8[2] = pos_t{ 5, -5, -10 } * scale;
    polygon8[3] = pos_t{ 5, -5, 5 } * scale;

    //Ecke 
    std::vector<pos_t> polygon9(4);
    polygon9[0] = pos_t{ 0, 5, -15 } *scale;
    polygon9[1] = pos_t{ 0, 5, -25 } *scale;
    polygon9[2] = pos_t{ 0, -5, -25 } *scale;
    polygon9[3] = pos_t{ 0, -5, -15 } *scale;

    std::vector<pos_t> polygon10(4);
    polygon10[0] = pos_t{ -15, 5, -15 } *scale;
    polygon10[1] = pos_t{ 0, 5, -15 } *scale;
    polygon10[2] = pos_t{ 0, -5, -15 } *scale;
    polygon10[3] = pos_t{ -15, -5, -15 } *scale;

    PlaneMesh planeMesh1;
    PlaneMesh planeMesh2;

    std::vector<pm::face_handle> faces;
    auto face1 = planeMesh1.insertPolygon(polygon1);
    //faces.push_back(planeMesh2.insertPolygon(polygon2));
    //faces.push_back(planeMesh2.insertPolygon(polygon3));
    //faces.push_back(planeMesh2.insertPolygon(polygon4));
    //faces.push_back(planeMesh2.insertPolygon(polygon5));
    faces.push_back(planeMesh2.insertPolygon(polygon6));
    faces.push_back(planeMesh2.insertPolygon(polygon7));
    faces.push_back(planeMesh2.insertPolygon(polygon8));
    faces.push_back(planeMesh2.insertPolygon(polygon9));
    faces.push_back(planeMesh2.insertPolygon(polygon10));

    auto octree = std::make_shared<Octree>(&planeMesh1, &planeMesh2, AABB{{-3200, -3200, -3200}, {3200, 3200, 3200}});

    for (auto f : planeMesh1.allFaces()) {
        octree->insert_polygon(planeMesh1.id(), f);
    }

    for (auto f : planeMesh2.allFaces()) {
        octree->insert_polygon(planeMesh2.id(), f);
    }
    auto iCut = octree->cutPolygons();

    planeMesh1.checkAndComputePositions();
    planeMesh2.checkAndComputePositions();

    /*auto view = gv::view(planeMesh1.positions());
    gv::view(gv::lines(planeMesh1.positions()).line_width_world(10));
    gv::view(gv::lines(planeMesh1.positions()).line_width_world(20), gv::masked(iCut.getIntersectionEdgesMarkerA()), tg::color3::color(0.0));*/
    //auto view = gv::view(planeMesh2.positions());
    /*gv::view(planeMesh2.positions());
    gv::view(gv::lines(planeMesh2.positions()).line_width_world(10));
    gv::view(gv::lines(planeMesh2.positions()).line_width_world(20), gv::masked(iCut.getIntersectionEdgesMarkerB()), tg::color3::color(0.0));*/
}

TEST("Test::Cut_Triangle_Normal_Marker_Case2") {

    scalar_t scale = 100;
    std::vector<pos_t> polygon1(4);
    polygon1[0] = pos_t{ -10, 0, -20 } *scale;
    polygon1[1] = pos_t{ -10, 0, 0 } *scale;
    polygon1[2] = pos_t{ 10, 0, 0 } *scale;
    polygon1[3] = pos_t{ 10, 0, -20 } *scale;

    std::vector<pos_t> polygon2(4);
    polygon2[0] = pos_t{ 0, 5, 5 } *scale;
    polygon2[1] = pos_t{ 0, 5, -10 } *scale;
    polygon2[2] = pos_t{ 0, -5, -10 } *scale;
    polygon2[3] = pos_t{ 0, -5, 5 } *scale;

    std::vector<pos_t> polygon3(4);
    polygon3[0] = pos_t{ -15, 5, -10 } *scale;
    polygon3[1] = pos_t{ -5, 5, -10 } *scale;
    polygon3[2] = pos_t{ -5, -5, -10 } *scale;
    polygon3[3] = pos_t{ -15, -5, -10 } *scale;

    std::vector<pos_t> polygon4(4);
    polygon4[0] = pos_t{ -5, 5, -10 } *scale;
    polygon4[1] = pos_t{ 0, 5, -10 } *scale;
    polygon4[2] = pos_t{ 0, -5, -10 } *scale;
    polygon4[3] = pos_t{ -5, -5, -10 } *scale;
 
    std::vector<pos_t> polygon5(4);
    polygon5[0] = pos_t{ 0, 5, -15 } *scale;
    polygon5[1] = pos_t{ 0, 5, -25 } *scale;
    polygon5[2] = pos_t{ 0, -5, -25 } *scale;
    polygon5[3] = pos_t{ 0, -5, -15 } *scale;

    std::vector<pos_t> polygon6(4);
    polygon6[0] = pos_t{ -15, 5, -15 } *scale;
    polygon6[1] = pos_t{ 0, 5, -15 } *scale;
    polygon6[2] = pos_t{ 0, -5, -15 } *scale;
    polygon6[3] = pos_t{ -15, -5, -15 } *scale;

    PlaneMesh planeMesh1;
    PlaneMesh planeMesh2;

    std::vector<pm::face_handle> faces;
    auto face1 = planeMesh1.insertPolygon(polygon1);
    faces.push_back(planeMesh2.insertPolygon(polygon2));
    faces.push_back(planeMesh2.insertPolygon(polygon3));
    faces.push_back(planeMesh2.insertPolygon(polygon4));
    faces.push_back(planeMesh2.insertPolygon(polygon5)); 
    faces.push_back(planeMesh2.insertPolygon(polygon6));

    auto octree = std::make_shared<Octree>(&planeMesh1, &planeMesh2, AABB{ {-3200, -3200, -3200}, {3200, 3200, 3200} });

    for (auto f : planeMesh1.allFaces()) {
        octree->insert_polygon(planeMesh1.id(), f);
    }

    for (auto f : planeMesh2.allFaces()) {
        octree->insert_polygon(planeMesh2.id(), f);
    }
    auto iCut = octree->cutPolygons();

    planeMesh1.checkAndComputePositions();
    planeMesh2.checkAndComputePositions();

    /*auto view = gv::view(planeMesh1.positions());
    gv::view(gv::lines(planeMesh1.positions()).line_width_world(10));
    gv::view(gv::lines(planeMesh1.positions()).line_width_world(20), gv::masked(iCut.getIntersectionEdgesMarkerA()), tg::color3::color(0.0));*/
    /*auto view = gv::view(planeMesh2.positions());
    //gv::view(planeMesh2.positions());
    gv::view(gv::lines(planeMesh2.positions()).line_width_world(10));
    gv::view(gv::lines(planeMesh2.positions()).line_width_world(20), gv::masked(iCut.getIntersectionEdgesMarkerB()), tg::color3::color(0.0));*/
}

TEST("Test::Cut_Triangle_Normal_Marker_Case3") {

    scalar_t scale = 100;
    std::vector<pos_t> polygon1(4);
    polygon1[0] = pos_t{ -10, 0, -20 } *scale;
    polygon1[1] = pos_t{ -10, 0, 0 } *scale;
    polygon1[2] = pos_t{ 10, 0, 0 } *scale;
    polygon1[3] = pos_t{ 10, 0, -20 } *scale;

    std::vector<pos_t> polygon2(4);
    polygon2[0] = pos_t{ -11, 5, -5 } *scale;
    polygon2[1] = pos_t{ -5, 5, -5 } *scale;
    polygon2[2] = pos_t{ -5, -5, -5 } *scale;
    polygon2[3] = pos_t{ -11, -5, -5 } *scale;

    std::vector<pos_t> polygon3(4);
    polygon3[0] = pos_t{ -5, 5, -5 } *scale;
    polygon3[1] = pos_t{ 5, 5, -5 } *scale;
    polygon3[2] = pos_t{ 5, -5, -5 } *scale;
    polygon3[3] = pos_t{ -5, -5, -5 } *scale;

    std::vector<pos_t> polygon4(4);
    polygon4[0] = pos_t{ 5, 5, -10 } *scale;
    polygon4[1] = pos_t{ 11, 5, -15 } *scale;
    polygon4[2] = pos_t{ 11, -5, -15 } *scale;
    polygon4[3] = pos_t{ 5, -5, -10 } *scale;

    std::vector<pos_t> polygon5(4);
    polygon5[0] = pos_t{ 5, 5, -5 } *scale;
    polygon5[1] = pos_t{ 5, 5, -10 } *scale;
    polygon5[2] = pos_t{ 5, -5, -10 } *scale;
    polygon5[3] = pos_t{ 5, -5, -5 } *scale;

    PlaneMesh planeMesh1;
    PlaneMesh planeMesh2;

    std::vector<pm::face_handle> faces;
    auto face1 = planeMesh1.insertPolygon(polygon1);
    faces.push_back(planeMesh2.insertPolygon(polygon2));
    faces.push_back(planeMesh2.insertPolygon(polygon3));
    faces.push_back(planeMesh2.insertPolygon(polygon4));
    faces.push_back(planeMesh2.insertPolygon(polygon5));

    auto octree = std::make_shared<Octree>(&planeMesh1, &planeMesh2, AABB{ {-3200, -3200, -3200}, {3200, 3200, 3200} });

    for (auto f : planeMesh1.allFaces()) {
        octree->insert_polygon(planeMesh1.id(), f);
    }

    for (auto f : planeMesh2.allFaces()) {
        octree->insert_polygon(planeMesh2.id(), f);
    }
    auto iCut = octree->cutPolygons();

    planeMesh1.checkAndComputePositions();
    planeMesh2.checkAndComputePositions();

    /*auto view = gv::view(planeMesh1.positions());
    gv::view(gv::lines(planeMesh1.positions()).line_width_world(10));
    gv::view(gv::lines(planeMesh1.positions()).line_width_world(20), gv::masked(iCut.getIntersectionEdgesMarkerA()), tg::color3::color(0.0));*/
    //auto view = gv::view(planeMesh2.positions());
    /*gv::view(planeMesh2.positions());
    gv::view(gv::lines(planeMesh2.positions()).line_width_world(10));
    gv::view(gv::lines(planeMesh2.positions()).line_width_world(20), gv::masked(iCut.getIntersectionEdgesMarkerB()), tg::color3::color(0.0));*/
}

TEST("Test::Cut_Triangle_Normal_Marker_Case4") {

    scalar_t scale = 100;
    std::vector<pos_t> polygon1(4);
    polygon1[0] = pos_t{ -10, 0, -20 } *scale;
    polygon1[1] = pos_t{ -10, 0, 0 } *scale;
    polygon1[2] = pos_t{ 10, 0, 0 } *scale;
    polygon1[3] = pos_t{ 10, 0, -20 } *scale;

    std::vector<pos_t> polygon2(4);
    polygon2[0] = pos_t{ -15, 5, -10 } *scale;
    polygon2[1] = pos_t{ -5, 5, -10 } *scale;
    polygon2[2] = pos_t{ -5, -5, -10 } *scale;
    polygon2[3] = pos_t{ -15, -5, -10 } *scale;

    std::vector<pos_t> polygon3(4);
    polygon3[0] = pos_t{ -5, 5, -10 } *scale;
    polygon3[1] = pos_t{ -5, 5, -15 } *scale;
    polygon3[2] = pos_t{ -5, -5, -15 } *scale;
    polygon3[3] = pos_t{ -5, -5, -10 } *scale;

    std::vector<pos_t> polygon4(4);
    polygon4[0] = pos_t{ -5, 5, -15 } *scale;
    polygon4[1] = pos_t{ 2, 5, -15 } *scale;
    polygon4[2] = pos_t{ 2, -5, -15 } *scale;
    polygon4[3] = pos_t{ -5, -5, -15 } *scale;

    std::vector<pos_t> polygon5(4);
    polygon5[0] = pos_t{ 2, 5, -15 } *scale;
    polygon5[1] = pos_t{ 2, 5, -10 } *scale;
    polygon5[2] = pos_t{ 2, -5, -10 } *scale;
    polygon5[3] = pos_t{ 2, -5, -15 } *scale;

    std::vector<pos_t> polygon6(4);
    polygon6[0] = pos_t{ 2, 5, -10 } *scale;
    polygon6[1] = pos_t{ 11, 5, -10 } *scale;
    polygon6[2] = pos_t{ 11, -5, -10 } *scale;
    polygon6[3] = pos_t{ 2, -5, -10 } *scale;

    PlaneMesh planeMesh1;
    PlaneMesh planeMesh2;

    std::vector<pm::face_handle> faces;
    auto face1 = planeMesh1.insertPolygon(polygon1);
    faces.push_back(planeMesh2.insertPolygon(polygon2));
    faces.push_back(planeMesh2.insertPolygon(polygon3));
    faces.push_back(planeMesh2.insertPolygon(polygon4));
    faces.push_back(planeMesh2.insertPolygon(polygon5));
    faces.push_back(planeMesh2.insertPolygon(polygon6));

    auto octree = std::make_shared<Octree>(&planeMesh1, &planeMesh2, AABB{ {-3200, -3200, -3200}, {3200, 3200, 3200} });

    for (auto f : planeMesh1.allFaces()) {
        octree->insert_polygon(planeMesh1.id(), f);
    }

    for (auto f : planeMesh2.allFaces()) {
        octree->insert_polygon(planeMesh2.id(), f);
    }
    auto iCut = octree->cutPolygons();

    planeMesh1.checkAndComputePositions();
    planeMesh2.checkAndComputePositions();

    /*auto view = gv::view(planeMesh1.positions());
    gv::view(gv::lines(planeMesh1.positions()).line_width_world(10));
    gv::view(gv::lines(planeMesh1.positions()).line_width_world(20), gv::masked(iCut.getIntersectionEdgesMarkerA()), tg::color3::color(0.0));*/
    //auto view = gv::view(planeMesh2.positions());
    /*gv::view(planeMesh2.positions());
    gv::view(gv::lines(planeMesh2.positions()).line_width_world(10));
    gv::view(gv::lines(planeMesh2.positions()).line_width_world(20), gv::masked(iCut.getIntersectionEdgesMarkerB()), tg::color3::color(0.0));*/
}

TEST("Test::Co_Planar_Cut") {
    scalar_t scale = 100;
    auto meshA = pm::Mesh();
    pm::vertex_attribute<tg::pos3> mPosA(meshA);
    pm::vertex_handle v1_1 = meshA.vertices().add();
    pm::vertex_handle v1_2 = meshA.vertices().add();
    pm::vertex_handle v1_3 = meshA.vertices().add();
    pm::vertex_handle v1_4 = meshA.vertices().add();
    mPosA[v1_1] = tg::pos3{ -10, 0, -15 };
    mPosA[v1_2] = tg::pos3{ 0, 0, 0 };
    mPosA[v1_3] = tg::pos3{ 10, 0, -15 };
    mPosA[v1_4] = tg::pos3{ 0, -5, -7 };
    meshA.faces().add(v1_1, v1_2, v1_3);
    meshA.faces().add(v1_1, v1_4, v1_2);
    meshA.faces().add(v1_2, v1_4, v1_3);
    meshA.faces().add(v1_3, v1_4, v1_1);

    auto meshB = pm::Mesh();
    pm::vertex_attribute<tg::pos3> mPosB(meshB);
    v1_1 = meshB.vertices().add();
    v1_2 = meshB.vertices().add();
    v1_3 = meshB.vertices().add();
    v1_4 = meshB.vertices().add();
    pm::vertex_handle v1_5 = meshB.vertices().add();
    pm::vertex_handle v1_6 = meshB.vertices().add();
    mPosB[v1_1] = tg::pos3{ -3, 0, -10 };
    mPosB[v1_2] = tg::pos3{ 3, 0, -10 };
    mPosB[v1_3] = tg::pos3{ 0, 0, -4 };
    mPosB[v1_4] = tg::pos3{ 0, 3, -10 };
    mPosB[v1_5] = tg::pos3{ -4, -1, -12 };
    mPosB[v1_6] = tg::pos3{ 4, -1, -12 };
    meshB.faces().add(v1_1, v1_2, v1_3);
    meshB.faces().add(v1_2, v1_4, v1_3);
    meshB.faces().add(v1_3, v1_4, v1_1);
    //New Faces
    meshB.faces().add(v1_1, v1_5, v1_2);
    meshB.faces().add(v1_5, v1_6, v1_2);
    meshB.faces().add(v1_4, v1_2, v1_6);
    meshB.faces().add(v1_4, v1_5, v1_1);
    meshB.faces().add(v1_4, v1_6, v1_5);

    PlaneMesh planeMesh1(meshA, mPosA, scale);
    PlaneMesh planeMesh2(meshB, mPosB, scale);

    auto octree = std::make_shared<Octree>(&planeMesh1, &planeMesh2, AABB{ {-3200, -3200, -3200}, {3200, 3200, 3200} });

    for (auto f : planeMesh1.allFaces()) {
        octree->insert_polygon(planeMesh1.id(), f);
    }

    for (auto f : planeMesh2.allFaces()) {
        octree->insert_polygon(planeMesh2.id(), f);
    }

    auto iCut = octree->cutPolygons();
    planeMesh1.checkAndComputePositions();
    planeMesh2.checkAndComputePositions();
    std::vector<AABB> boxes;
    octree->insertAABB(boxes);

    std::vector<tg::aabb3> returnBoxes;
    returnBoxes.reserve(boxes.size());

    for (auto box : boxes) {
        returnBoxes.push_back(tg::aabb3(tg::pos3(box.min), tg::pos3(box.max)));
    }

    
    std::vector<gv::label> labels;
    for (auto v : planeMesh1.mesh().all_vertices())
        if(v.idx.value == 2)
            labels.push_back(gv::label{ std::string("Mesh1 V") + std::to_string(v.idx.value), {}, tg::pos3(planeMesh1.posInt(v)) });

    for (auto v : planeMesh2.mesh().all_vertices())
        if (v.idx.value == 3)
            labels.push_back(gv::label{ std::string("Mesh2 V") + std::to_string(v.idx.value), {}, tg::pos3(planeMesh2.posInt(v)) });

    {
        auto view = gv::view(planeMesh1.positions());
        gv::view(gv::lines(planeMesh1.positions()).line_width_world(5));
        gv::view(planeMesh2.positions());
        gv::view(gv::lines(planeMesh2.positions()).line_width_world(5));
        gv::view(gv::lines(returnBoxes).line_width_world(10), tg::color3::blue, "gv::lines(pos)");
        gv::view(labels);
    }
    octree->repairOctree(iCut);
    std::shared_ptr<FaceComponentFinder> components1 = std::make_shared<FaceComponentFinder>(planeMesh1, iCut.getIntersectionEdgesMarkerA());
    std::shared_ptr<FaceComponentFinder> components2 = std::make_shared<FaceComponentFinder>(planeMesh2, iCut.getIntersectionEdgesMarkerB());
    auto components = std::make_shared<ComponentCategorization>(octree, components1, components2, iCut);
    components->renderFinalResult(iCut, 1);


    
    auto view = gv::view(planeMesh1.positions());
    gv::view(gv::lines(planeMesh1.positions()).line_width_world(10));
    //gv::view(gv::lines(planeMesh1.positions()).line_width_world(20), gv::masked(iCut.getIntersectionEdgesMarkerA()), tg::color3::color(0.0));
    //auto view = gv::view(planeMesh2.positions());
    gv::view(planeMesh2.positions());
    gv::view(gv::lines(planeMesh2.positions()).line_width_world(10));
    //gv::view(gv::lines(planeMesh2.positions()).line_width_world(20), gv::masked(iCut.getIntersectionEdgesMarkerB()), tg::color3::color(0.0));*/
}

TEST("Test::Cut_Triangle_Normal") {
    tg::triangle<3, scalar_t> triangle1({ { 0, 0, -20 }, { 10, 0, 0 }, { -10, 0, 0 } });
    tg::triangle<3, scalar_t> triangle2({ { 0, 5, -15 }, { 5, -5, -5 }, { -5, -5, -5 } });
    tg::triangle<3, scalar_t> triangle3({ { 10, 5, -10 }, { 5, -5, 5 }, { 15, -5, 5 } });
    tg::triangle<3, scalar_t> triangle4({ { 0, 2, -7 }, { 0, -2, -9 }, { 0, -2, -5 } });
    tg::triangle<3, scalar_t> triangle5({ { 5, -5, 0 }, { 5, 5, 0 }, { 10, -5, -10 } });

    PlaneMesh planeMesh1;
    PlaneMesh planeMesh2;

    std::vector<pm::face_handle> faces;
    scalar_t scale = 100;

    auto face1 = planeMesh1.insertTriangle(triangle1 * scale);
    faces.push_back(planeMesh2.insertTriangle(triangle2 * scale));
    faces.push_back(planeMesh2.insertTriangle(triangle3 * scale));
    faces.push_back(planeMesh2.insertTriangle(triangle4 * scale));
    faces.push_back(planeMesh2.insertTriangle(triangle5 * scale));

    IntersectionCut intersectionCut(&planeMesh1, &planeMesh2);
    intersectionCut.splitAccordingToIntersection(face1, faces);
    
    planeMesh1.checkAndComputePositions();
    planeMesh2.checkAndComputePositions();

    //planeMesh1.mesh().compactify();
    //planeMesh2.mesh().compactify();
    auto test1 = planeMesh1.noDuplicatedVerticesInFaces();
    auto test2 = planeMesh2.noDuplicatedVerticesInFaces();

    /*std::cout << "Mesh1 " <<  planeMesh1.positions().count() << " positions" << std::endl;
    std::cout << "Mesh1 " << planeMesh1.edges().count() << " edges" << std::endl;
    std::cout << "Mesh2 " << planeMesh2.positions().count() << " positions" << std::endl;
    std::cout << "Mesh2 " << planeMesh2.edges().count() << " edges" << std::endl;

    auto view = gv::view(planeMesh1.positions());
    gv::view(gv::lines(planeMesh1.positions()).line_width_world(10));
    gv::view(planeMesh2.positions());
    gv::view(gv::lines(planeMesh2.positions()).line_width_world(10));
    gv::view(gv::points(planeMesh2.positions()).point_size_world(15));*/
}

TEST("Test::Cut_Triangle_Planar_1") {
    PlaneMesh planeMesh1;
    PlaneMesh planeMesh2;
    std::vector<pm::face_handle> faces;
    auto face1 = (planeMesh1.insertPolygon({ 0, 0, -20 }, { 10, 0, 0 }, { -10, 0, 0 }));
    faces.push_back(planeMesh2.insertPolygon({ 0, 2, -7 }, { 0, -2, -9 }, { 0, -2, -5 }));
    faces.push_back(planeMesh2.insertPolygon({ -3, 2, -7 }, { -3, -2, -9 }, { -3, -2, -5 }));
    faces.push_back(planeMesh2.insertPolygon({ -5, 0, -15 }, { -10, 0, 10 }, { 0, 0, 10 }));
    faces.push_back(planeMesh2.insertPolygon({ 10, 0, -5 }, { 5, 0, 5 }, { 15, 0, 5 }));
    faces.push_back(planeMesh2.insertPolygon({ 0, 0, -20 }, { -20, 0, -20 }, { -10, 0, 0 }));
    faces.push_back(planeMesh2.insertPolygon({ -13, 0, -5 }, { -8, 0, 5 }, { -18, 0, 5 }));
    faces.push_back(planeMesh2.insertPolygon({ 0, 0, -30 }, { 5, 0, -20 }, { -5, 0, -20 }));

    IntersectionCut intersectionCut(&planeMesh1, &planeMesh2);
    intersectionCut.splitAccordingToIntersection(face1, faces);
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

    IntersectionCut intersectionCut(&planeMesh1, &planeMesh2);
    intersectionCut.splitAccordingToIntersection(face1, faces);

    planeMesh1.checkAndComputePositions();
    planeMesh2.checkAndComputePositions();

    /*{
        auto view = gv::view(planeMesh1.positions());
        gv::view(gv::lines(planeMesh1.positions()).line_width_world(10));
    }
    {
        auto view = gv::view(planeMesh2.positions());
        gv::view(gv::lines(planeMesh2.positions()).line_width_world(10));
    }*/

}

TEST("Test::Cut_Triangle_Polygon") {
    pm::Mesh mesh;
    pm::vertex_attribute<pos_t> positions(mesh);

    scalar_t scale = 100;

    std::vector<pos_t> polygon1(5);
    polygon1[0] = pos_t{ -10, 0, 10 } * scale;
    polygon1[1] = pos_t{ -15, 0, 0 } * scale;
    polygon1[2] = pos_t{ -10, 0, -10 } * scale;
    polygon1[3] = pos_t{ 10, 0, -10 } * scale;
    polygon1[4] = pos_t{ 10, 0, 10 } * scale;

    std::vector<pos_t> polygon2(4);
    polygon2[0] = pos_t{ 5, 0, -15 } * scale;
    polygon2[1] = pos_t{ 15, 0, -15 } * scale;
    polygon2[2] = pos_t{ 15, 0, -5 } * scale;
    polygon2[3] = pos_t{ 5, 0, -5 } * scale;

    std::vector<pos_t> polygon3(6);
    polygon3[0] = pos_t{ -2, 5, -2 } * scale;
    polygon3[1] = pos_t{ 2, 5, -2 } * scale;
    polygon3[2] = pos_t{ 5, 0, 0 } * scale;
    polygon3[3] = pos_t{ 2, -5, 2 } * scale;
    polygon3[4] = pos_t{ -2, -5, 2 } * scale;
    polygon3[5] = pos_t{ -5, 0, 0 } * scale;

    std::vector<pos_t> polygon4(5);
    polygon4[0] = pos_t{ -10, 5, -6 } * scale;
    polygon4[1] = pos_t{ -8, 5, -6 } * scale;
    polygon4[2] = pos_t{ -8, -5, -2 } * scale;
    polygon4[3] = pos_t{ -12, -5, -2 } * scale;
    polygon4[4] = pos_t{ -15, 0, -4 } * scale;

    std::vector<pos_t> polygon5(6);
    polygon5[0] = pos_t{ -2, -5, -12 } * scale;
    polygon5[1] = pos_t{ 2, -5, -12 } * scale;
    polygon5[2] = pos_t{ 5, 0, -10 }  * scale;
    polygon5[3] = pos_t{ 2, 5, -8 } * scale;
    polygon5[4] = pos_t{ -2, 5, -8 } * scale;
    polygon5[5] = pos_t{ -5, 0, -10 } * scale;

    std::vector<pos_t> polygon6(6);
    polygon6[0] = pos_t{ -7, -5, 13 } * scale;
    polygon6[1] = pos_t{ -3, -5, 13 } * scale;
    polygon6[2] = pos_t{ 0, 0, 11 } * scale;
    polygon6[3] = pos_t{ -3, 5, 9 } * scale;
    polygon6[4] = pos_t{ -5, 5, 9 } * scale;
    polygon6[5] = pos_t{ -10, 0, 11 } * scale;

    std::vector<pos_t> polygon7(4);
    polygon7[0] = pos_t{ 11, 0, 0 } * scale;
    polygon7[1] = pos_t{ 21, 0, 0 } * scale;
    polygon7[2] = pos_t{ 21, 0, 10 } * scale;
    polygon7[3] = pos_t{ 11, 0, 10 } * scale;

    std::vector<pos_t> polygon8(4);
    polygon8[0] = pos_t{ -5, 1, 0 } * scale;
    polygon8[1] = pos_t{ 5, 1, 0 } * scale;
    polygon8[2] = pos_t{ 5, 1, 10 } * scale;
    polygon8[3] = pos_t{ -5, 1, 10 } * scale;

    PlaneMesh planeMesh1;
    PlaneMesh planeMesh2;

    std::vector<pm::face_handle> faces;
    

    auto face1 = planeMesh1.insertPolygon(polygon1);
    faces.push_back(planeMesh2.insertPolygon(polygon2));
    faces.push_back(planeMesh2.insertPolygon(polygon3));
    faces.push_back(planeMesh2.insertPolygon(polygon4));
    faces.push_back(planeMesh2.insertPolygon(polygon5));
    faces.push_back(planeMesh2.insertPolygon(polygon6));
    faces.push_back(planeMesh2.insertPolygon(polygon7));
    faces.push_back(planeMesh2.insertPolygon(polygon8));

    //splitAccordingToIntersection(face1, faces, planeMesh1, planeMesh2);

    planeMesh1.checkAndComputePositions();
    planeMesh2.checkAndComputePositions();

    /*auto view = gv::view(planeMesh1.positions());
    gv::view(gv::lines(planeMesh1.positions()).line_width_world(10));
    gv::view(planeMesh2.positions());
    gv::view(gv::lines(planeMesh2.positions()).line_width_world(10));*/
}