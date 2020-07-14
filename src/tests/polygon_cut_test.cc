#include <intersection_utils.hh>

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

    splitAccordingToIntersection(face1, faces, planeMesh1, planeMesh2);

    planeMesh1.checkAndComputePositions();
    planeMesh2.checkAndComputePositions();

    auto view = gv::view(planeMesh1.positions());
    gv::view(gv::lines(planeMesh1.positions()).line_width_world(10));
    gv::view(planeMesh2.positions());
    gv::view(gv::lines(planeMesh2.positions()).line_width_world(10));
}