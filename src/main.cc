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
void test_color_in_mesh();
void test_plane_visu();
void test_octree();
void test_transpose();
void test_octree_two_meshes();
void test_trianle_classification();
void transformation(pm::vertex_attribute<tg::pos3>& pos, tg::mat4& mat);

int main() {
    glow::glfw::GlfwContext ctx;
    nx::Nexus tests;
    tests.run();
    pm::vertex_attribute<tg::pos3> test3;

    std::vector<tg::ipos3> positions;
    positions.push_back({ 2, 0, 0 });
    positions.push_back({ 2, 0, 2 });
    positions.push_back({ 0, 0, 2 });
    positions.push_back({ 0, 0, 0 });
    positions.push_back({ 1, 0, -2 });

    //test_trianle_classification();
    
    //testIntersectionTriangleNormal();
    //testIntersectionTrianglePlanar();
    //testIntersectionPolygon();
    //test_plane_visu();
    //test_transpose();
    test_color_in_mesh();
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

void test_color_in_mesh() {

    pm::Mesh mesh1;
    pm::vertex_attribute<tg::pos3> pos1(mesh1);
    pm::load("../data/mesh/fox.obj", mesh1, pos1);

    pm::Mesh mesh2;
    pm::vertex_attribute<tg::pos3> pos2(mesh2);
    pm::load("../data/mesh/fox.obj", mesh2, pos2);

    auto translation1 = tg::translation(tg::vec{ 0.f, -50.f, 15.f });
    auto rotatation1 = tg::rotation_x(tg::angle::from_degree(-90));
    auto trans1 = translation1 * rotatation1;
    transformation(pos1, trans1);

    scalar_t scale = 1e5;
    PlaneMesh planeMesh1(mesh1, pos1, scale);
    PlaneMesh planeMesh2(mesh2, pos2, scale);

    auto faceColors1 = planeMesh1.mesh().faces().make_attribute_with_default(tg::color3::cyan);
    auto faceColors2 = planeMesh2.mesh().faces().make_attribute_with_default(tg::color3::magenta);
    int intersectionCount = 0;

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    for (pm::face_handle face1 : planeMesh1.mesh().faces()) {
        for (pm::face_handle face2 : planeMesh2.mesh().faces()) {
            if (ob::intersect<geometry128>(planeMesh1, face1, planeMesh2, face2)) {
                faceColors1[face1] = tg::color3::black;
                faceColors2[face2] = tg::color3::black;
            }
            intersectionCount++;
            //std::cout << intersectionCount << std::endl;
        }
    }
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds> (end - begin).count();
    std::cout << intersectionCount << " intersections in " << seconds << "seconds" << std::endl;
    //test[mesh1.faces().first()] = tg::color3::black;
    auto view = gv::view(planeMesh1.positions(), faceColors1);
    gv::view(planeMesh2.positions(), faceColors2);
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