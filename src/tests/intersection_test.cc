#include <plane_polygon.hh>
#include <nexus/test.hh>
#include <aabb.hh>
#include <glow-extras/viewer/view.hh>

TEST("Test::Intersection_Triangle_Normal") {
    tg::triangle<3, scalar_t> triangle1({ { 0, 0, -20 }, { 10, 0, 0 }, { -10, 0, 0 } });
    tg::triangle<3, scalar_t> triangle2({ { 0, 5, -15 }, { 5, -5, -5 }, { -5, -5, -5 } });
    tg::triangle<3, scalar_t> triangle3({ { 0, 0, -15 }, { 20, 0, -15 }, { 10, 10, -30 } });
    tg::triangle<3, scalar_t> triangle4({ { -5, 0, -5 }, { -10, -5, 10 }, { 0, -5, 10 } });
    tg::triangle<3, scalar_t> triangle5({ { 10, 5, -10 }, { 5, -5, 5 }, { 15, -5, 5 } });
    tg::triangle<3, scalar_t> triangle6({ { 0, 0, -20 }, { -20, 10, -20 }, { -10, 0, 0 } });
    tg::triangle<3, scalar_t> triangle7({ { -13, -5, -5 }, { -8, 5, 5 }, { -18, 5, 5 } });
    tg::triangle<3, scalar_t> triangle8({ { 0, 5, -25 }, { 5, -5, -15 }, { -5, -5, -15 } });
    tg::triangle<3, scalar_t> triangle9({ { 10, 5, -5 }, { 15, -5, -5 }, { 5, -5, -5 } });

    PlaneMesh planeMesh1;
    PlaneMesh planeMesh2;
    auto face1 = planeMesh1.insertTriangle(triangle1);
    auto face2 = planeMesh2.insertTriangle(triangle2);
    auto face3 = planeMesh2.insertTriangle(triangle3);
    auto face4 = planeMesh2.insertTriangle(triangle4);
    auto face5 = planeMesh2.insertTriangle(triangle5);
    auto face6 = planeMesh2.insertTriangle(triangle6);
    auto face7 = planeMesh2.insertTriangle(triangle7);
    auto face8 = planeMesh2.insertTriangle(triangle8);
    auto face9 = planeMesh2.insertTriangle(triangle9);

    TG_ASSERT(ob::intersect<geometry128>(planeMesh1, face1, planeMesh2, face2) == true);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh1, face1, planeMesh2, face3) == false);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh1, face1, planeMesh2, face4) == false);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh1, face1, planeMesh2, face5) == true);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh1, face1, planeMesh2, face6) == false);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh1, face1, planeMesh2, face7) == false);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh1, face1, planeMesh2, face8) == false);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh1, face1, planeMesh2, face9) == false);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh2, face5, planeMesh2, face9) == true);

    //auto view = gv::view(planeMesh.positions());
    //gv::view(gv::lines(planeMesh.positions()).line_width_world(0.1));
}

TEST("Test::Intersection_Triangle_Planar") {
    PlaneMesh planeMesh1;
    PlaneMesh planeMesh2;
    auto face1 = planeMesh1.insertPolygon({ 0, 0, -20 }, { 10, 0, 0 }, { -10, 0, 0 });
    auto face2 = planeMesh2.insertPolygon({ 0, 0, -15 }, { 5, 0, -5 }, { -5, 0, -5 });
    /*auto face3 = planeMesh2.insertPolygon({ 0, 0, -10 }, { 20, 0, -10 }, { 10, 0, -30 });
    auto face4 = planeMesh2.insertPolygon({ -5, 0, -15 }, { -10, 0, 10 }, { 0, 0, 10 });
    auto face5 = planeMesh2.insertPolygon({ 10, 0, -5 }, { 5, 0, 5 }, { 15, 0, 5 });
    auto face6 = planeMesh2.insertPolygon({ 0, 0, -20 }, { -20, 0, -20 }, { -10, 0, 0 });
    auto face7 = planeMesh2.insertPolygon({ -13, 0, -5 }, { -8, 0, 5 }, { -18, 0, 5 });
    auto face8 = planeMesh2.insertPolygon({ 0, 0, -30 }, { 5, 0, -20 }, { -5, 0, -20 });*/

    TG_ASSERT(ob::intersect<geometry128>(planeMesh1, face1, planeMesh2, face2) == true);
    /*TG_ASSERT(ob::intersect<geometry128>(planeMesh1, face1, planeMesh2, face3) == true);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh1, face1, planeMesh2, face4) == true);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh1, face1, planeMesh2, face5) == true);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh1, face1, planeMesh2, face6) == false);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh1, face1, planeMesh2, face7) == false);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh1, face1, planeMesh2, face8) == false);*/

    auto view = gv::view(planeMesh1.positions());
    gv::view(gv::lines(planeMesh1.positions()).line_width_world(0.1));
    gv::view(planeMesh2.positions());
    gv::view(gv::lines(planeMesh2.positions()).line_width_world(0.1));
}

TEST("Test::Intersection_Polygon") {
    pm::Mesh mesh;
    pm::vertex_attribute<pos_t> positions(mesh);
    std::vector<pos_t> polygon1(5);
    polygon1[0] = { -10, 0, 10 };
    polygon1[1] = { -15, 0, 0 };
    polygon1[2] = { -10, 0, -10 };
    polygon1[3] = { 10, 0, -10 };
    polygon1[4] = { 10, 0, 10 };

    std::vector<pos_t> polygon2(4);
    polygon2[0] = { 5, 0, -15 };
    polygon2[1] = { 15, 0, -15 };
    polygon2[2] = { 15, 0, -5 };
    polygon2[3] = { 5, 0, -5 };

    std::vector<pos_t> polygon3(6);
    polygon3[0] = { -2, 5, -2 };
    polygon3[1] = { 2, 5, -2 };
    polygon3[2] = { 5, 0, 0 };
    polygon3[3] = { 2, -5, 2 };
    polygon3[4] = { -2, -5, 2 };
    polygon3[5] = { -5, 0, 0 };

    std::vector<pos_t> polygon4(5);
    polygon4[0] = { -10, 5, -6 };
    polygon4[1] = { -8, 5, -6 };
    polygon4[2] = { -8, -5, -2 };
    polygon4[3] = { -12, -5, -2 };
    polygon4[4] = { -15, 0, -4 };

    std::vector<pos_t> polygon5(6);
    polygon5[0] = { -2, -5, -12 };
    polygon5[1] = { 2, -5, -12 };
    polygon5[2] = { 5, 0, -10 };
    polygon5[3] = { 2, 5, -8 };
    polygon5[4] = { -2, 5, -8 };
    polygon5[5] = { -5, 0, -10 };

    std::vector<pos_t> polygon6(6);
    polygon6[0] = { -7, -5, 13 };
    polygon6[1] = { -3, -5, 13 };
    polygon6[2] = { 0, 0, 11 };
    polygon6[3] = { -3, 5, 9 };
    polygon6[4] = { -5, 5, 9 };
    polygon6[5] = { -10, 0, 11 };

    std::vector<pos_t> polygon7(4);
    polygon7[0] = { 11, 0, 0 };
    polygon7[1] = { 21, 0, 0 };
    polygon7[2] = { 21, 0, 10 };
    polygon7[3] = { 11, 0, 10 };

    std::vector<pos_t> polygon8(4);
    polygon8[0] = { -5, 1, 0 };
    polygon8[1] = { 5, 1, 0 };
    polygon8[2] = { 5, 1, 10 };
    polygon8[3] = { -5, 1, 10 };

    PlaneMesh planeMesh1;
    PlaneMesh planeMesh2;
    auto face1 = planeMesh1.insertPolygon(polygon1);
    auto face2 = planeMesh2.insertPolygon(polygon2);
    auto face3 = planeMesh2.insertPolygon(polygon3);
    auto face4 = planeMesh2.insertPolygon(polygon4);
    auto face5 = planeMesh2.insertPolygon(polygon5);
    auto face6 = planeMesh2.insertPolygon(polygon6);
    auto face7 = planeMesh2.insertPolygon(polygon7);
    auto face8 = planeMesh2.insertPolygon(polygon8);

    TG_ASSERT(ob::intersect<geometry128>(planeMesh1, face1, planeMesh2, face2) == true);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh1, face1, planeMesh2, face3) == true);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh1, face1, planeMesh2, face4) == true);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh1, face1, planeMesh2, face5) == false);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh1, face1, planeMesh2, face6) == false);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh1, face1, planeMesh2, face7) == false);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh1, face1, planeMesh2, face8) == false);

    //auto view = gv::view(planeMesh.positions());
    //gv::view(gv::lines(planeMesh.positions()).line_width_world(0.1));
}