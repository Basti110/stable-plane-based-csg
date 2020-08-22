#include <string>
#include <iostream>
#include <glow-extras/viewer/view.hh>
#include <glow-extras/viewer/experimental.hh>
//#include <tg/typed-geometry.hh>
#include <typed-geometry/tg.hh>
#include <glow-extras/glfw/GlfwContext.hh>
#include <plane_triangle.hh>
#include <plane_polygon.hh>
#include <octree.hh>
#include <aabb.hh>
#include <nexus/Nexus.hh>
#include <face_component_finder.hh>
#include <imgui/imgui.h>
#include <polymesh/algorithms/deduplicate.hh>
#include <tuple>
#include <utility>
#include "obj_config.hh"

//std::string testObj = "../data/mesh/fox.obj";
std::string testObj = "../data/mesh/bun_zipper.obj";

void test_cut_testAB_meshes();
void test_picker();
void testIntersectionTriangleNormal(); 
void testIntersectionTrianglePlanar();
void testIntersectionPolygon();
void testIntersection();
void test_cut_mesh();
void test_color_in_mesh();
void test_plane_visu();
void test_octree();
void test_transpose();
void test_octree_two_meshes();
void test_trianle_classification();
void test_color_lines();
void mark_component_test();
void transformation(pm::vertex_attribute<tg::pos3>& pos, tg::mat4& mat);

int main() {
    ObjCollection::init();
    auto testest = tg::translation(tg::vec{ -.0f, .0f, .04f });
    if (bool test = true) {
        std::cout << test;
    }
    glow::glfw::GlfwContext ctx;
    nx::Nexus tests;
    //tests.run();

    pm::vertex_attribute<tg::pos3> test3;

    std::vector<tg::ipos3> positions;
    positions.push_back({ 2, 0, 0 });
    positions.push_back({ 2, 0, 2 });
    positions.push_back({ 0, 0, 2 });
    positions.push_back({ 0, 0, 0 });
    positions.push_back({ 1, 0, -2 });
    
    //test_picker();
    //test_color_in_mesh();
    //test_octree();
    test_cut_mesh();
    //test_octree_two_meshes();
    //mark_component_test();
    //test_color_in_mesh();
    /*while (true) {
        test_picker();
        test_cut_testAB_meshes();
    }*/

    //
    //test_cut_mesh();
    //test_color_lines();
    //test_trianle_classification();
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
    ObjConfig conf = ObjCollection::map.at("bunny_mesh_1");

    auto boxes = conf.getOctreeBoxes();
    //auto g = gv::grid();
    auto view = gv::view(conf.getMeshA()->positions());
    gv::view(gv::lines(conf.getMeshA()->positions()).line_width_world(10000), "gv::lines(pos)");
    //auto view = gv::view(ipos);
    gv::view(gv::lines(boxes).line_width_world(50000), tg::color3::blue, "gv::lines(pos)");
}

void test_transpose() {
    pm::Mesh mesh1;
    pm::vertex_attribute<tg::pos3> pos1(mesh1);
    pm::load(testObj, mesh1, pos1);

    pm::Mesh mesh2;
    pm::vertex_attribute<tg::pos3> pos2(mesh2);
    pm::load(testObj, mesh2, pos2);

    auto translation1 = tg::translation(tg::vec{ -.0f, .0f, .04f });  
    auto rotatation1 = tg::rotation_y(tg::angle::from_degree(-90));
    auto trans1 = translation1 * rotatation1;

    auto trans2 = tg::translation(tg::vec{ 0.f, .01f, .0f });

    transformation(pos1, trans1);
    transformation(pos2, trans2);

    auto view = gv::view(pos1);
    gv::view(pos2);
}

void test_color_in_mesh() {
    ObjConfig conf = ObjCollection::map.at("fox_mesh_2");
    auto planeMesh1 = conf.getMeshA();
    auto planeMesh2 = conf.getMeshB();
    auto faceColors1 = planeMesh1->mesh().faces().make_attribute_with_default(tg::color3::cyan);
    auto faceColors2 = planeMesh2->mesh().faces().make_attribute_with_default(tg::color3::magenta);

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    int intersectionCount = conf.getOctree()->markIntersections(faceColors1, faceColors2);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    auto seconds = std::chrono::duration_cast<std::chrono::milliseconds> (end - begin).count();
    std::cout << intersectionCount << " intersections in " << seconds << "ms" << std::endl;

    auto view = gv::view(planeMesh1->positions(), faceColors1);
    gv::view(planeMesh2->positions(), faceColors2);
    gv::view(gv::lines(planeMesh1->positions()).line_width_world(10000));
    gv::view(gv::lines(planeMesh2->positions()).line_width_world(10000));
}

void test_color_lines() {

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

    scalar_t scale = 1e6;
    PlaneMesh planeMesh1(mesh1, pos1, scale);
    PlaneMesh planeMesh2(mesh2, pos2, scale);

    auto edgeColors1 = planeMesh1.mesh().edges().make_attribute_with_default(tg::color3::cyan);
    auto edgeColors2 = planeMesh2.mesh().edges().make_attribute_with_default(tg::color3::magenta);

    auto view = gv::view(planeMesh1.positions());
    gv::view(planeMesh2.positions());
    gv::view(gv::lines(planeMesh1.positions()).line_width_world(100000), edgeColors1);
    gv::view(gv::lines(planeMesh2.positions()).line_width_world(100000), edgeColors2);
}

void test_cut_mesh() {
    ObjConfig conf = ObjCollection::map.at("bunny_mesh_2");
    auto planeMesh1 = conf.getMeshA();
    auto planeMesh2 = conf.getMeshB();

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    auto iCut = conf.getOctree()->cutPolygons();
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    auto seconds = std::chrono::duration_cast<std::chrono::milliseconds> (end - begin).count();
    std::cout << "Cut in " << seconds << "ms" << std::endl;
    if (planeMesh1->allFacesHaveEdges())
        return;

    if (planeMesh2->allFacesHaveEdges())
        return;

    planeMesh1->checkAndComputePositions();
    planeMesh2->checkAndComputePositions();

    planeMesh1->mesh().compactify();
    auto faceMask1 = planeMesh1->mesh().faces().make_attribute_with_default(false);
    auto faceMask2 = planeMesh2->mesh().faces().make_attribute_with_default(false);

    auto test1 = planeMesh1->noDuplicatedVerticesInFaces(faceMask1);
    auto test2 = planeMesh2->noDuplicatedVerticesInFaces(faceMask2);
   
    //int test = pm::deduplicate(planeMesh1.mesh(), planeMesh1.positions());
    //auto view = gv::view(planeMesh1.positions(), gv::masked(faceMask1));
    //auto view = gv::view(planeMesh2.positions(), tg::color3::color(0.5));
    //gv::view(gv::lines(planeMesh2.positions()).line_width_world(100000), gv::masked(iCut.getIntersectionEdgesMarkerB()), tg::color3::color(0.0));
    //gv::view(gv::lines(planeMesh2.positions()).line_width_world(10000), tg::color3::color(0.0));
    {
        auto view = gv::view(planeMesh2->positions());
        gv::view(gv::lines(planeMesh2->positions()).line_width_world(10000), tg::color3::color(0.0));
        gv::view(gv::lines(planeMesh2->positions()).line_width_world(100000), gv::masked(iCut.getIntersectionEdgesMarkerB()), tg::color3::color(0.0));
        gv::view(gv::lines(planeMesh1->positions()).line_width_world(10000), tg::color3::red);
    }

    scalar_t scale = 1e6;
    AABB box({ -60 * scale, -60 * scale, -40 * scale }, { 60 * scale, 60 * scale, 80 * scale });
    SharedOctree newOctree = std::make_shared<Octree>(&(*planeMesh1), &(*planeMesh2), box);

    for (auto f : planeMesh1->mesh().faces()) {
        bool test1 = f.is_removed();
        pos_t pos = planeMesh1->posInt(f.any_halfedge().vertex_from());
        int count = f.vertices().count();
        //std::cout << "count: " << count << std::endl;
        newOctree->insert_polygon(planeMesh1->id(), f);
    }

    for (auto f : planeMesh2->mesh().faces()) {
        newOctree->insert_polygon(planeMesh2->id(), f);
    }
    newOctree->startDebugView();
}

void test_octree_two_meshes() {
    ObjConfig conf = ObjCollection::map.at("bunny_mesh_2");
    auto planeMesh1 = conf.getMeshA();
    auto planeMesh2 = conf.getMeshB();
            
    auto boxes = conf.getOctreeBoxes();
    //octree->startDebugView();
    //auto g = gv::grid();
    auto view = gv::view(planeMesh1->positions());
    gv::view(gv::lines(planeMesh1->positions()).line_width_world(10000), "gv::lines(pos)");
    gv::view(planeMesh2->positions());
    gv::view(gv::lines(planeMesh2->positions()).line_width_world(10000), "gv::lines(pos)");
    gv::view(gv::lines(boxes).line_width_world(50000), tg::color3::blue, "gv::lines(pos)");

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

void mark_component_test() {
    ObjConfig conf = ObjCollection::map.at("bunny_mesh_2");
    auto planeMesh1 = conf.getMeshA();
    auto planeMesh2 = conf.getMeshB();

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    auto iCut = conf.getOctree()->cutPolygons();
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    auto seconds = std::chrono::duration_cast<std::chrono::milliseconds> (end - begin).count();

    FaceComponentFinder components1(*planeMesh1, iCut.getIntersectionEdgesMarkerA());
    FaceComponentFinder components2(*planeMesh2, iCut.getIntersectionEdgesMarkerB());
    std::cout << "Found " << components1.countComponents() << " components" << std::endl;

    std::cout << "Cut in " << seconds << "ms" << std::endl;
    if (planeMesh1->allFacesHaveEdges())
        return;

    if (planeMesh2->allFacesHaveEdges())
        return;

    planeMesh1->checkAndComputePositions();
    planeMesh2->checkAndComputePositions();

    planeMesh1->mesh().compactify();
    auto faceMask1 = planeMesh1->mesh().faces().make_attribute_with_default(false);
    auto faceMask2 = planeMesh2->mesh().faces().make_attribute_with_default(false);

    auto test1 = planeMesh1->noDuplicatedVerticesInFaces(faceMask1);
    auto test2 = planeMesh2->noDuplicatedVerticesInFaces(faceMask2);

    /*auto view = gv::view(planeMesh2->positions(), components2.getColorAssignment());
    gv::view(gv::lines(planeMesh2->positions()).line_width_world(100000), gv::masked(iCut.getIntersectionEdgesMarkerB()), tg::color3::color(0.0));
    gv::view(gv::lines(planeMesh2->positions()).line_width_world(10000), tg::color3::color(0.0));*/
    auto view = gv::view(planeMesh1->positions(), components1.getColorAssignment());
    gv::view(gv::lines(planeMesh1->positions()).line_width_world(10000), tg::color3::color(0.0));
    gv::view(gv::lines(planeMesh1->positions()).line_width_world(100000), gv::masked(iCut.getIntersectionEdgesMarkerA()), tg::color3::color(0.0));
}

void test_picker() {
    ObjConfig conf = ObjCollection::map.at("fox_mesh_2");
    conf.getOctree()->startDebugView();
}

void test_cut_testAB_meshes() {
    pm::Mesh mesh1;
    pm::vertex_attribute<tg::pos3> pos1(mesh1);
    pm::load("../data/mesh/meshA.obj", mesh1, pos1);
    mesh1.compactify();
    pm::deduplicate(mesh1, pos1);

    pm::Mesh mesh2;
    pm::vertex_attribute<tg::pos3> pos2(mesh2);
    pm::load("../data/mesh/meshB.obj", mesh2, pos2);
    mesh2.compactify();
    pm::deduplicate(mesh2, pos2);

    scalar_t scale = 1e6;
    PlaneMesh planeMesh1(mesh1, pos1, 1);
    PlaneMesh planeMesh2(mesh2, pos2, 1);

    AABB box({ -60 * scale, -60 * scale, -40 * scale }, { 60 * scale, 60 * scale, 80 * scale });
    SharedOctree octree = std::make_shared<Octree>(&planeMesh1, &planeMesh2, box);

    for (auto f : planeMesh1.allFaces()) {
        octree->insert_polygon(planeMesh1.id(), f);
    }

    for (auto f : planeMesh2.allFaces()) {
        octree->insert_polygon(planeMesh2.id(), f);
    }

    TG_ASSERT(planeMesh1.allFacesHaveValidHalfEdges());
    TG_ASSERT(planeMesh2.allFacesHaveValidHalfEdges());

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    auto iCut = octree->cutPolygons();
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    auto seconds = std::chrono::duration_cast<std::chrono::milliseconds> (end - begin).count();

    std::cout << "Cut in " << seconds << "ms" << std::endl;
    if (planeMesh1.allFacesHaveEdges())
        return;

    if (planeMesh2.allFacesHaveEdges())
        return;

    planeMesh1.checkAndComputePositions();
    planeMesh2.checkAndComputePositions();

    planeMesh1.mesh().compactify();

    {
        auto view = gv::view(planeMesh2.positions());
        gv::view(gv::lines(planeMesh2.positions()).line_width_world(10000), tg::color3::color(0.0));
        gv::view(gv::lines(planeMesh2.positions()).line_width_world(100000), gv::masked(iCut.getIntersectionEdgesMarkerB()), tg::color3::color(0.0));
        gv::view(gv::lines(planeMesh1.positions()).line_width_world(10000), tg::color3::red);
    }

    SharedOctree newOctree = std::make_shared<Octree>(&planeMesh1, &planeMesh2, box);

    for (auto f : planeMesh1.mesh().faces()) {
        bool test1 = f.is_removed();
        pos_t pos = planeMesh1.posInt(f.any_halfedge().vertex_from());
        int count = f.vertices().count();
        //std::cout << "count: " << count << std::endl;
        newOctree->insert_polygon(planeMesh1.id(), f);
    }

    for (auto f : planeMesh2.mesh().faces()) {
        newOctree->insert_polygon(planeMesh2.id(), f);
    }
    newOctree->startDebugView();
}