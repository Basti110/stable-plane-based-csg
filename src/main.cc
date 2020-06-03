#include <string>
#include <iostream>
#include <glow-extras/viewer/view.hh>
//#include <tg/typed-geometry.hh>
#include <typed-geometry/tg.hh>
#include <glow-extras/glfw/GlfwContext.hh>
#include <plane_triangle.hh>
#include <plane_polygon.hh>
#include <octree.hh>
#include <aabb.hh>
#include <nexus/Nexus.hh>

#include <tuple>
#include <utility>

void testIntersectionTriangleNormal(); 
void testIntersectionTrianglePlanar();
void testIntersectionPolygon();
void testIntersection();
void test_plane_visu();
void test_octree();
void test_transpose();
void test_octree_two_meshes();
void test_trianle_classification();
void transformation(pm::vertex_attribute<tg::pos3>& pos, tg::mat4& mat);

int main() {
    nx::Nexus tests;
    tests.run();
    pm::vertex_attribute<tg::pos3> test3;

    glow::glfw::GlfwContext ctx;

    std::vector<tg::ipos3> positions;
    positions.push_back({ 2, 0, 0 });
    positions.push_back({ 2, 0, 2 });
    positions.push_back({ 0, 0, 2 });
    positions.push_back({ 0, 0, 0 });
    positions.push_back({ 1, 0, -2 });

    //test_trianle_classification();
    
    //testIntersectionTriangleNormal();
    //testIntersectionTrianglePlanar();
    testIntersectionPolygon();
    //test_plane_visu();
    //test_transpose();
    //test_octree();
    //test_octree_two_meshes();

    //std::vector<tg::triangle3> planes;

    /// add face to topology
    //m.faces().add(vh0, vh1, vh2, vh1);

    //auto triangle = PlaneTriangle({ -1, 1, -1 }, { 1, 1, -1 }, { 0, 1, 1 });
    //triangle.compute_subdetermiants();

    //auto polygon = PlanePolygon(positions);
    //polygon.compute_subdetermiants();
    
    //auto planes = polygon.get_planes_triangles(5);
    //polygon.getPolygone(pos, m);
    //gv::view(pos);
    //gv::view(planes, "tg::triangle soup");
    //gv::view(triangle.getTriangle(), "tg::triangle soup");

    //view(test2);
}

void test_plane_visu()
{
    std::vector<tg::ipos3> positions;
    positions.push_back({ 2, 0, 0 });
    positions.push_back({ 2, 0, 2 });
    positions.push_back({ 0, 0, 2 });
    positions.push_back({ 0, 0, 0 });
    positions.push_back({ 1, 0, -2 });

    auto mesh = PlaneMesh();
    mesh.insertPolygon({ 4, 0, 0 }, { 2, -2, 3 }, { 0, 0, 0 });
    mesh.insertPolygon({ 2, -2, 3 }, { -2, -2, 3 }, { 0, 0, 0 });
    mesh.insertPolygon({ -2, -2, 3 }, { -4, 0, 0 }, { 0, 0, 0 });
    mesh.insertPolygon({ -4, 0, 0 }, { -2, 2, -3 }, { 0, 0, 0 });
    mesh.insertPolygon({ -2, 2, -3 }, { 2, 2, -3 }, { 0, 0, 0 });
    mesh.insertPolygon({ 2, 2, -3 }, { 4, 0, 0 }, { 0, 0, 0 });
    std::vector<tg::triangle3> planes;
    mesh.planesTriangles(10, planes);

    //auto colorV1 = tg::vec3(0, 119, 255) / 255; base
    //auto colorV1 = tg::vec3(0, 115, 230) / 255;
    //auto colorV2 = tg::vec3(237, 5, 82) / 255;

    auto colorV2 = tg::vec3(0, 115, 230) / 255;
    auto colorV1 = tg::vec3(214, 246, 255) / 255;
    //auto colorV2 = tg::vec3(215, 247, 7) / 255;

    auto color1 = tg::color3::color(colorV1.x, colorV1.y, colorV1.z);
    auto color2 = tg::color3::color(colorV2.x, colorV2.y, colorV2.z);

    auto view = gv::view();
    for (int i = 0; i < 6; ++i) {
        int o = i * 8;
        gv::view(std::vector<tg::triangle3>(planes.begin() + o, planes.begin() + o + 2), color1);
        gv::view(std::vector<tg::triangle3>(planes.begin() + o + 2, planes.begin() + o + 6), color2);
    }

    
    //auto view = gv::view(mesh.positions());
    //gv::view(gv::lines(mesh.positions()), "gv::lines(pos)");


}

void test_octree() {
    scalar_t scale = 1e5;
    AABB box({ -60 * scale, -60 * scale, -40 * scale }, { 60 * scale, 60 * scale, 80 * scale });
    std::vector<AABB> boxes1;
    std::vector<tg::aabb3> boxes2;
    
    pm::Mesh mesh;
    pm::vertex_attribute<tg::pos3> pos(mesh);
    pm::load("../data/mesh/fox.obj", mesh, pos);

    auto translation1 = tg::translation(tg::vec{ 0.f, -50.f, 15.f });
    auto rotatation1 = tg::rotation_x(tg::angle::from_degree(-90));
    auto trans1 = translation1 * rotatation1;
    transformation(pos, trans1);

    PlaneMesh meshA(mesh, pos, scale);
    PlaneMesh meshB;
    SharedOctree octree = std::make_shared<Octree>(&meshA, &meshB, box);
    octree->setOption(Octree::Options::SPLIT_ONE_MESH);

    for (auto f : meshA.allFaces()) {
        octree->insert_polygon(meshA.id(), f);
    }

    octree->insertAABB(boxes1);

    for (auto box : boxes1) {
        boxes2.push_back(tg::aabb3(tg::pos3(box.min), tg::pos3(box.max)));
    }
    //auto g = gv::grid();
    auto view = gv::view(meshA.positions());
    gv::view(gv::lines(meshA.positions()).line_width_world(10000), "gv::lines(pos)");
    //auto view = gv::view(ipos);
    gv::view(gv::lines(boxes2).line_width_world(50000), tg::color3::blue, "gv::lines(pos)");
}

void test_transpose() {
    pm::Mesh mesh1;
    pm::vertex_attribute<tg::pos3> pos1(mesh1);
    pm::load("../data/mesh/fox.obj", mesh1, pos1);

    pm::Mesh mesh2;
    pm::vertex_attribute<tg::pos3> pos2(mesh2);
    pm::load("../data/mesh/teddy.obj", mesh2, pos2);

    auto translation1 = tg::translation(tg::vec{ 0.f, -50.f, 15.f });  
    auto rotatation1 = tg::rotation_x(tg::angle::from_degree(-90));
    auto trans1 = translation1 * rotatation1;

    auto trans2 = tg::translation(tg::vec{ 0.0f, 10.f, 25.f });

    transformation(pos1, trans1);
    transformation(pos2, trans2);

    auto view = gv::view(pos1);
    gv::view(pos2);
    /*auto view = gv::view(gv::lines(pos1).line_width_world(1));
    gv::view(gv::lines(pos2).line_width_world(1));*/
}

void test_octree_two_meshes() {
    pm::Mesh mesh1;
    pm::vertex_attribute<tg::pos3> pos1(mesh1);
    pm::load("../data/mesh/fox.obj", mesh1, pos1);

    pm::Mesh mesh2;
    pm::vertex_attribute<tg::pos3> pos2(mesh2);
    pm::load("../data/mesh/teddy.obj", mesh2, pos2);

    auto translation1 = tg::translation(tg::vec{ 0.f, -50.f, 15.f });
    auto rotatation1 = tg::rotation_x(tg::angle::from_degree(-90));
    auto trans1 = translation1 * rotatation1;

    auto trans2 = tg::translation(tg::vec{ 0.0f, 10.f, 25.f });

    transformation(pos1, trans1);
    transformation(pos2, trans2);

    
    scalar_t scale = 1e5;
    AABB box({ -60 * scale, -60 * scale, -40 * scale }, { 60 * scale, 60 * scale, 80 * scale });
    std::vector<AABB> boxes1;
    std::vector<tg::aabb3> boxes2;

    PlaneMesh meshA(mesh1, pos1, scale);
    PlaneMesh meshB(mesh2, pos2, scale);

    SharedOctree octree = std::make_shared<Octree>(&meshA, &meshB, box);

    for (auto f : meshA.allFaces()) {
        octree->insert_polygon(meshA.id(), f);
    }

    for (auto f : meshB.allFaces()) {
        octree->insert_polygon(meshB.id(), f);
    }

    octree->insertAABB(boxes1);

    for (auto box : boxes1) {
        boxes2.push_back(tg::aabb3(tg::pos3(box.min), tg::pos3(box.max)));
    }
    //auto g = gv::grid();
    auto view = gv::view(meshA.positions());
    gv::view(gv::lines(meshA.positions()).line_width_world(10000), "gv::lines(pos)");
    gv::view(meshB.positions());
    gv::view(gv::lines(meshB.positions()).line_width_world(10000), "gv::lines(pos)");
    //auto view = gv::view(ipos);
    gv::view(gv::lines(boxes2).line_width_world(50000), tg::color3::blue, "gv::lines(pos)");

}

void transformation(pm::vertex_attribute<tg::pos3>& pos, tg::mat4& mat) {
    pos.apply([&mat](tg::pos3& pos) {
        auto pos4 = tg::vec4(pos, 1);
        pos4 = mat * pos4;
        pos = tg::pos3(pos4);
        }
    );
}

void test_trianle_classification() {
    pos_t p0 = { 0, 0, 0};
    pos_t p1 = { 0, 0, -1};
    pos_t p2 = { 1, 0, 0};
    pos_t p3 = { 0, 1, 0 };
    Plane plane1 = Plane::from_points(p0, p1, p2);
    Plane plane2 = Plane::from_points(p0, p1, p3);
    Plane plane3 = Plane::from_points(p0, p2, p3);

    Plane plane_comp1 = Plane::from_points({0, 0, -1}, { 1, 0, -1 }, { 0, 1, -1 });
    Plane plane_comp2 = Plane::from_points({ 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 });
    Plane plane_comp3 = Plane::from_points({ 0, 0, 1 }, { 1, 0, 1 }, { 0, 1, 1 });

    geometry128::subdeterminants_t subs;
    ob::compute_subdeterminants(plane1, plane2, plane3, subs);
    int comp1 = ob::classify_vertex(subs, plane_comp1);
    int comp2 = ob::classify_vertex(subs, plane_comp2);
    int comp3 = ob::classify_vertex(subs, plane_comp3);

    std::cout << "[START TEST]" << std::endl;
    std::cout << "comp 1 " << comp1 << std::endl;
    std::cout << "comp 2 " << comp2 << std::endl;
    std::cout << "comp 3 " << comp3 << std::endl;
}

void testIntersectionTrianglePlanar() {
    PlaneMesh planeMesh;
    auto face1 = planeMesh.insertPolygon({ 0, 0, -20 }, { 10, 0, 0 }, { -10, 0, 0 });
    auto face2 = planeMesh.insertPolygon({ 0, 0, -15 }, { 5, 0, -5 }, { -5, 0, -5 });
    auto face3 = planeMesh.insertPolygon({ 0, 0, -10 }, { 20, 0, -10 }, { 10, 0, -30 });
    auto face4 = planeMesh.insertPolygon({ -5, 0, -15 }, { -10, 0, 10 }, { 0, 0, 10 });
    auto face5 = planeMesh.insertPolygon({ 10, 0, -5 }, { 5, 0, 5 }, { 15, 0, 5 });
    auto face6 = planeMesh.insertPolygon({ 0, 0, -20 }, { -20, 0, -20 }, { -10, 0, 0 });
    auto face7 = planeMesh.insertPolygon({ -13, 0, -5 }, { -8, 0, 5 }, { -18, 0, 5 });
    auto face8 = planeMesh.insertPolygon({ 0, 0, -30 }, { 5, 0, -20 }, { -5, 0, -20 });

    TG_ASSERT(ob::intersect<geometry128>(planeMesh, face1, face2) == true);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh, face1, face3) == true);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh, face1, face4) == true);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh, face1, face5) == true);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh, face1, face6) == false);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh, face1, face7) == false);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh, face1, face8) == false);

    //auto view = gv::view(planeMesh.positions());
    //gv::view(gv::lines(planeMesh.positions()).line_width_world(0.1));
}

void testIntersectionTriangleNormal() {

    tg::triangle<3, scalar_t> triangle1({ { 0, 0, -20 }, { 10, 0, 0 }, { -10, 0, 0 } });
    tg::triangle<3, scalar_t> triangle2({ { 0, 5, -15 }, { 5, -5, -5 }, { -5, -5, -5 } });
    tg::triangle<3, scalar_t> triangle3({ { 0, 0, -15 }, { 20, 0, -15 }, { 10, 10, -30 } });
    tg::triangle<3, scalar_t> triangle4({ { -5, 0, -5 }, { -10, -5, 10 }, { 0, -5, 10 } });
    tg::triangle<3, scalar_t> triangle5({ { 10, 5, -10 }, { 5, -5, 5 }, { 15, -5, 5 } });
    tg::triangle<3, scalar_t> triangle6({ { 0, 0, -20 }, { -20, 10, -20 }, { -10, 0, 0 } });
    tg::triangle<3, scalar_t> triangle7({ { -13, -5, -5 }, { -8, 5, 5 }, { -18, 5, 5 } });
    tg::triangle<3, scalar_t> triangle8({ { 0, 5, -25 }, { 5, -5, -15 }, { -5, -5, -15 } });
    tg::triangle<3, scalar_t> triangle9({ { 10, 5, -5 }, { 15, -5, -5 }, { 5, -5, -5 } });

    PlaneMesh planeMesh;
    auto face1 = planeMesh.insertTriangle(triangle1);
    auto face2 = planeMesh.insertTriangle(triangle2);
    auto face3 = planeMesh.insertTriangle(triangle3);
    auto face4 = planeMesh.insertTriangle(triangle4);
    auto face5 = planeMesh.insertTriangle(triangle5);
    auto face6 = planeMesh.insertTriangle(triangle6);
    auto face7 = planeMesh.insertTriangle(triangle7);
    auto face8 = planeMesh.insertTriangle(triangle8);
    auto face9 = planeMesh.insertTriangle(triangle9);

    TG_ASSERT(ob::intersect<geometry128>(planeMesh, face1, face2) == true);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh, face1, face3) == false);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh, face1, face4) == false);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh, face1, face5) == true);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh, face1, face6) == false);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh, face1, face7) == false);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh, face1, face8) == false);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh, face1, face9) == false);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh, face5, face9) == true);

    //auto view = gv::view(planeMesh.positions());
    //gv::view(gv::lines(planeMesh.positions()).line_width_world(0.1));
}

void testIntersectionPolygon() {
    //Polygons
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

    /*auto addPolygon = [&](std::vector<pos_t> polygon) {
        std::vector<pm::vertex_handle> handles;
        for (auto p : polygon) {
            auto handle = mesh.vertices().add();
            handles.push_back(handle);
            positions[handle] = p;
        }
        mesh.faces().add(handles);
    };

    addPolygon(polygon1);
    addPolygon(polygon2);
    addPolygon(polygon3);
    addPolygon(polygon4);
    addPolygon(polygon5);
    addPolygon(polygon6);
    addPolygon(polygon7);
    addPolygon(polygon8);*/


    //PlaneMesh planeMesh(mesh, positions);
    //std::vector<tg::triangle3> planes;
    //planeMesh.planesTriangles(30, planes);
    PlaneMesh planeMesh;
    auto face1 = planeMesh.insertPolygon(polygon1);
    auto face2 = planeMesh.insertPolygon(polygon2);
    auto face3 = planeMesh.insertPolygon(polygon3);
    auto face4 = planeMesh.insertPolygon(polygon4);
    auto face5 = planeMesh.insertPolygon(polygon5);
    auto face6 = planeMesh.insertPolygon(polygon6);
    auto face7 = planeMesh.insertPolygon(polygon7);
    auto face8 = planeMesh.insertPolygon(polygon8);

    TG_ASSERT(ob::intersect<geometry128>(planeMesh, face1, face2) == true);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh, face1, face3) == true);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh, face1, face4) == true);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh, face1, face5) == false);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh, face1, face6) == false);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh, face1, face7) == false);
    TG_ASSERT(ob::intersect<geometry128>(planeMesh, face1, face8) == false);


    auto view = gv::view(planeMesh.positions());
    gv::view(gv::lines(planeMesh.positions()).line_width_world(0.1));
}