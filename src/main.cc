#include <string>
#include <glow-extras/viewer/view.hh>
#include <typed-geometry/tg.hh>
#include <glow-extras/glfw/GlfwContext.hh>
#include <plane_polygon.hh>
#include <octree.hh>
#include <nexus/Nexus.hh>
#include <face_component_finder.hh>
#include <imgui/imgui.h>
#include <polymesh/algorithms/deduplicate.hh>
#include <obj_config.hh>
#include <benchmark.h>
#include <ray_cast.hh>
#include <csg_app.h>
//---
#include <iomanip>

//std::string path = "E:/Thingi10K/Thingi10K/raw_meshes/";
std::string path = "E:/benchmark/files/";
//std::string path = "D:/benchmark/";

void convert();
int centerMesh(std::string s);
void transformation(pm::vertex_attribute<tg::pos3>& pos, tg::mat4& mat);

float gamma = 2.2;
#define OCTREE_BLUE tg::pow(47/255.f, gamma), tg::pow(85/255.f, gamma), tg::pow(151/255.f, gamma)
#define RAYCAST_GREEN tg::pow(112 / 255.f, gamma), tg::pow(173 / 255.f, gamma), tg::pow(71 / 255.f, gamma)
#define RAYCAST_ORANGE tg::pow(237 / 255.f, gamma), tg::pow(125 / 255.f, gamma), tg::pow(49 / 255.f, gamma)

int main(int argc, char* argv[]) {
    ObjCollection::init();
    glow::glfw::GlfwContext ctx;
    nx::Nexus tests;

    if (argc > 1) {
        //std::string file = std::string(argv[1]);
        std::string file = "36372.stl";
            
        std::cout << "Try to open " << file << std::endl;       
        ObjConfig conf = ObjConfig(1e6, 1e6, AABB({ -100, -100, -100 }, { 100, 100, 100 }),
            path + file, tg::mat4::identity, tg::mat4::identity,
            path + file, tg::mat4::identity, tg::rotation_y(tg::angle::from_degree(1)));

        conf.setMeshRepairBefore(true);
        //conf.setMoveToCenter(true);
        return Benchmark::testMesh(conf, "E:/benchmark/benchmark/" + file.substr(0, file.size() - 3) + "txt", true, 400);
    }
        
    //ObjConfig conf = ObjCollection::map.at("complex_1");
    //conf.viewMesh(true);
    
    //tests.run();
    //test_octree();    

    //char* arg[] = { "main", "Image::Cubes" };
    //char* arg[] = { "main", "Image::Intersection_2" };
    //char* argv[] = { "main", "Image::Touching_Face" };
    //char* argv[] = { "main", "Image::BasePlaneTest1" };
    //char* arg[] = { "main", "App::Plane_Geometry_Visu"};
    //char* arg[] = { "main", "App::Picker" };
    //char* argv[] = { "main", "App::Picker_Cut" };
    //char* argv[] = { "main", "App::component_classification" };
    //char* arg[] = { "main", "App:ShowMeshOrOctree" };
    //char* argv[] = { "main", "Benchmark:TestAvgTime" };   
    //char* arg[] = { "main", "Benchmark:TestOctree" };


    //Apps
    //char* arg[] = { "main", "Benchmark:OneIteration" };
    //char* arg[] = { "main", "App:PresentationDemo" }; // [x,y,z] rotate axes, [1,2] scale distance, E cut, Q reset
    //char* arg[] = { "main", "App::Picker" };
    

    //Demos
    //char* arg[] = { "main", "Demo:Co-Planar-Cubes" };
    //char* arg[] = { "main", "Demo:Normal-Cuts" };
    //char* arg[] = { "main", "Demo:Intersection-Demo-CoPlanar1" };
    //char* arg[] = { "main", "Demo:Intersection-Demo-CoPlanar2" };
    //char* arg[] = { "main", "Demo:Intersection-Demo-CoPlanar3" };
    //char* arg[] = { "main", "Demo:Intersection-Demo-CoPlanar4" };
    //char* arg[] = { "main", "Demo:Ray-Cast" };
    //char* arg[] = { "main", "Demo:Get-Poly-Mesh" };



    char* arg[] = { "main", "Test::Co_Planar_Cut2" };
    //char* arg[] = { "main", "Test::Co_Planar_Cut" };
    //char* argv[] = { "main", "Test::Cut_Triangle_Normal_Marker_Case4" };
    //char* argv[] = { "main", "App::Show_Mesh" };
    //char* argv[] = { "main", "App::count_intersections" };
    //char* argv[] = { "main", "Benchmark:SubdetVsIntPos" };

    tests.applyCmdArgs(2, arg);
    tests.run();
}

void convert() {
    pm::Mesh m;
    auto pos = m.vertices().make_attribute<tg::pos3>();
    load("../data/mesh/buddha.obj", m, pos);

    for (auto& p : pos)
        p = { p.x, -p.z, p.y };
    pm::deduplicate(m, pos);
    m.compactify();
    gv::view(pos);
    pm::save("../data/mesh/buddha_2.obj", pos);
}

void transformation(pm::vertex_attribute<tg::pos3>& pos, tg::mat4& mat) {
    pos.apply([&mat](tg::pos3& pos) {
        auto pos4 = tg::vec4(pos, 1);
        pos4 = mat * pos4;
        pos = tg::pos3(pos4);
        }
    );
}

int centerMesh(std::string filePath) {
    pm::Mesh m;
    pm::vertex_attribute<tg::pos3> pos(m);
    if (!pm::load(filePath, m, pos))
        return -1;
    auto aabb = tg::aabb_of(pos);
    auto distance = aabb.max - aabb.min;
    auto halfDistance = distance / 2;
    auto center = aabb.min + halfDistance;
    auto trans = tg::translation(-center);
    transformation(pos, trans);
    pm::save(filePath, pos);
    return 0;
}

//###############################################################
//###########         Für Später         ########################
//###############################################################

APP("Demo:Co-Planar-Cubes") {
    ObjConfig conf = ObjCollection::map.at("co-planar-cubes2");
    Benchmark::testMesh(conf, "../logs/out.log", true, 400);
    conf = ObjCollection::map.at("co-planar-cubes1");
    Benchmark::testMesh(conf, "../logs/out.log", true, 400);
}

APP("Demo:Intersection-Demo-CoPlanar1") {
    pm::Mesh m1;
    auto pos1 = m1.vertices().make_attribute<tg::pos3>();
    load("../data/mesh/37991.stl", m1, pos1);
    pm::deduplicate(m1, pos1);
    m1.compactify();

    pm::Mesh m2;
    auto pos2 = m2.vertices().make_attribute<tg::pos3>();
    load("../data/mesh/37991.stl", m2, pos2);
    pm::deduplicate(m2, pos2);
    m2.compactify();

    transformation(pos2, tg::rotation_y(tg::angle::from_degree(10)));
    auto csgResult = csg::computeCSG(pos1, pos2, 1e6, true);
}

APP("Demo:Intersection-Demo-CoPlanar2") {
    pm::Mesh m1;
    auto pos1 = m1.vertices().make_attribute<tg::pos3>();
    load("../data/mesh/37744.stl", m1, pos1);
    pm::deduplicate(m1, pos1);
    m1.compactify();

    pm::Mesh m2;
    auto pos2 = m2.vertices().make_attribute<tg::pos3>();
    load("../data/mesh/37744.stl", m2, pos2);
    pm::deduplicate(m2, pos2);
    m2.compactify();

    transformation(pos2, tg::rotation_y(tg::angle::from_degree(10)));
    auto csgResult = csg::computeCSG(pos1, pos2, 1e6, true);
}

APP("Demo:Intersection-Demo-CoPlanar3") {
    pm::Mesh m1;
    auto pos1 = m1.vertices().make_attribute<tg::pos3>();
    load("../data/mesh/37275.stl", m1, pos1);
    pm::deduplicate(m1, pos1);
    m1.compactify();

    pm::Mesh m2;
    auto pos2 = m2.vertices().make_attribute<tg::pos3>();
    load("../data/mesh/37275.stl", m2, pos2);
    pm::deduplicate(m2, pos2);
    m2.compactify();

    transformation(pos2, tg::rotation_y(tg::angle::from_degree(10)));
    auto csgResult = csg::computeCSG(pos1, pos2, 1e6, true);
}

APP("Demo:Intersection-Demo-CoPlanar4") {
    pm::Mesh m1;
    auto pos1 = m1.vertices().make_attribute<tg::pos3>();
    load("../data/mesh/36372.stl", m1, pos1);
    pm::deduplicate(m1, pos1);
    m1.compactify();

    pm::Mesh m2;
    auto pos2 = m2.vertices().make_attribute<tg::pos3>();
    load("../data/mesh/36372.stl", m2, pos2);
    pm::deduplicate(m2, pos2);
    m2.compactify();

    transformation(pos2, tg::rotation_y(tg::angle::from_degree(10)));
    auto csgResult = csg::computeCSG(pos1, pos2, 1e6, true);
}

APP("Demo:Normal-Cuts") {
    pm::Mesh m1;
    auto pos1 = m1.vertices().make_attribute<tg::pos3>();
    load("../data/mesh/37266.stl", m1, pos1);
    pm::deduplicate(m1, pos1);
    m1.compactify();

    pm::Mesh m2;
    auto pos2 = m2.vertices().make_attribute<tg::pos3>();
    load("../data/mesh/37266.stl", m2, pos2);
    transformation(pos2, tg::rotation_y(tg::angle::from_degree(10)));
    pm::deduplicate(m2, pos2);
    m2.compactify();

    pm::Mesh resultMesh;
    auto csgResult = csg::computeCSG(pos1, pos2, 1e6, true);
}

APP("Demo:Get-Poly-Mesh") {
    pm::Mesh m1;
    auto pos1 = m1.vertices().make_attribute<tg::pos3>();
    load("../data/mesh/fox.obj", m1, pos1);

    pm::Mesh m2;
    auto pos2 = m2.vertices().make_attribute<tg::pos3>();
    load("../data/mesh/fox.obj", m2, pos2);
    transformation(pos2, tg::rotation_y(tg::angle::from_degree(20)));

    pm::Mesh resultMesh;
    auto csgResult = csg::computeCSG(pos1, pos2);
    auto resultPos = csg::applyCSG(csgResult, csg::Operation::M1_MINUS_M2, resultMesh);
    gv::view(resultPos);
}

APP("Demo:Ray-Cast") {
    ObjConfig conf = ObjCollection::map.at("raycast");
    auto planeMesh1 = conf.getMeshA();
    auto planeMesh2 = conf.getMeshB();
    TG_ASSERT(planeMesh2->allFacesAreValid());
    auto octree = conf.getOctree();
    auto boxes = conf.getOctreeBoxes();

    pm::vertex_handle origin = planeMesh1->mesh().all_vertices().first();
    SharedDebugRayInfo rayInfo = std::make_shared<DebugRayInfo>();
    RayCast rayCast(octree->getPlaneMeshA(), octree->getPlaneMeshB(), octree, rayInfo);
    auto intersections = rayCast.countIntersectionsToOutside(origin);

    std::vector<tg::dsegment3> lines;
    for (int i = 0; i < (int)rayInfo->rayPath.size() - 1; ++i) {
        lines.push_back(tg::dsegment3{ ob::to_position(rayInfo->rayPath[i]), ob::to_position(rayInfo->rayPath[i + 1]) });
    }

    for (int i = 0; i < (int)rayInfo->nexPointsCell.size() - 1; ++i) {
        lines.push_back(tg::dsegment3{ tg::dpos3(rayInfo->nexPointsCell[i]), tg::dpos3(rayInfo->nexPointsCell[i + 1]) });
    }

    std::vector<tg::aabb3> returnBoxes;
    for (auto box : rayInfo->rayBoxesDirect) {
        returnBoxes.push_back(tg::aabb3(tg::pos3(box.min), tg::pos3(box.max)));
    }

    auto const octreeCells = gv::lines(boxes).line_width_world(600000);
    auto const rayCells = gv::lines(returnBoxes).line_width_world(5000000);
    auto const rayPath = gv::lines(lines).line_width_world(6000000);
    auto const positions1 = planeMesh1->positions();
    auto const positionLines1 = gv::lines(planeMesh1->positions()).line_width_world(100000);

    auto view = gv::view(rayCells, tg::color3::color(RAYCAST_ORANGE), gv::print_mode);
    gv::view(rayPath, tg::color3::color(RAYCAST_GREEN));
    gv::view(octreeCells, tg::color3::color(OCTREE_BLUE));
    gv::view(positions1);
    gv::view(positionLines1);   
}