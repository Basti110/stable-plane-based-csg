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
#include <component_categorization.hh>
#include <glow-extras/timing/CpuTimer.hh>
#include <obj_config.hh>
#include <polymesh/formats.hh>
#include <benchmark.h>
//---
#include <iomanip>
#define GIGA 1000000000
#define MEGA 1000000
#define CLOCK 3399740
//std::string testObj = "../data/mesh/fox.obj";
std::string testObj = "../data/mesh/buddha_2.obj";

void test_octree_cell_ray_cast();
void test_cut_testAB_meshes();
void test_cut_mesh();
void test_color_in_mesh();
void test_octree();
void test_transpose();
void test_octree_two_meshes();
void test_trianle_classification();
void test_color_lines();
void test_component_classification();
void convert();
void transformation(pm::vertex_attribute<tg::pos3>& pos, tg::mat4& mat);

int main(int argc, char* argv[]) {
    ObjCollection::init();
    auto testest = tg::translation(tg::vec{ -.0f, .0f, .04f });
    if (bool test = true) {
        std::cout << test;
    }
    glow::glfw::GlfwContext ctx;
    nx::Nexus tests;
    
    //convert();
    //return 0;
    if (argc > 1 || true) {
        //std::string file = std::string(argv[1]);
        std::string file = "36372.stl"; 
        {
            pm::Mesh mesh;
            pm::vertex_attribute<tg::pos3> pos(mesh);   
            pm::load("E:/benchmark/files/" + file, mesh, pos);
            pm::deduplicate(mesh, pos);
            mesh.compactify();
            for (auto h : mesh.halfedges()) {
                if (h.is_invalid())
                    return 2;
                if (h.face().is_invalid())
                    return 2;
            }
        }
               
        std::cout << "Try to open " << file << std::endl;       
        ObjConfig conf = ObjConfig(1e6, 1e6, AABB({ -100, -100, -100 }, { 100, 100, 100 }),
            "E:/benchmark/files/" + file, tg::mat4::identity, tg::mat4::identity,
            "E:/benchmark/files/" + file, tg::mat4::identity, tg::rotation_y(tg::angle::from_degree(-90)));
        conf.setMeshRepairBefore(true);
        return Benchmark::testMesh(conf, "E:/benchmark/benchmark/" + file.substr(0, file.size() - 3) + "txt");
    }
        
    //ObjConfig conf = ObjCollection::map.at("complex_1");
    //conf.viewMesh(true);
    
    //tests.run();
    //test_octree();    
    //char* argv[] = { "main", "Image::Intersection" };
    //char* argv[] = { "main", "Image::Touching_Face" };
    //char* argv[] = { "main", "Image::BasePlaneTest1" };
    //char* argv[] = { "main", "App::Plane_Geometry_Visu"};
    //char* argv[] = { "main", "App::Picker" };
    //char* argv[] = { "main", "App::Picker_Cut" };
    //char* argv[] = { "main", "App::component_classification" };
    //char* argv[] = { "main", "Benchmark:TestAvgTime" };   
    
    char* arg[] = { "main", "Benchmark:OneIteration" };
    //char* arg[] = { "main", "Test::Co_Planar_Cut" };
    //char* argv[] = { "main", "Test::Cut_Triangle_Normal_Marker_Case4" };
    //char* argv[] = { "main", "App::Show_Mesh" };
    //char* argv[] = { "main", "App::count_intersections" };
    //char* argv[] = { "main", "Benchmark:SubdetVsIntPos" };

    tests.applyCmdArgs(2, arg);
    tests.run();

    //test_transpose();  
    //test_color_in_mesh();
    //test_octree();
    //test_component_classification();
    //test_cut_mesh();
    //test_octree_cell_ray_cast();
    //mark_component_test();
    //test_color_in_mesh();
    
    //ObjConfig conf = ObjCollection::map.at("fox_mesh_2");
    /*while (true) {        
        //conf.getOctree()->startDebugView();
        test_cut_testAB_meshes();
    }*/

    
    //test_cut_mesh();
    //test_color_lines();
    //test_trianle_classification();
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

    auto translation1 = tg::translation(tg::vec{ -1.0f, 0.0f, 2.0f });  
    auto rotatation1 = tg::rotation_y(tg::angle::from_degree(-90));
    auto trans1 = translation1 * rotatation1;

    auto trans2 = tg::translation(tg::vec{ 0.f, 0.0f, .0f });

    transformation(pos1, trans1);
    //transformation(pos2, trans2);

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

void removeInvalidFaces() {
    pm::Mesh m;
    auto pos = m.vertices().make_attribute<tg::pos3>();
    load("../data/mesh/soma_gyroid_Z_2.obj", m, pos);

    pm::Mesh mMesh;
    VertexAttribute mPositions(mMesh);
    mMesh.copy_from(m);
    auto vertices1 = mMesh.all_vertices();
    auto vertices2 = m.all_vertices();
    TG_ASSERT(vertices1.size() == vertices2.size());
    for (int i = 0; i < vertices1.size(); ++i) {
        auto v1 = vertices1[i];
        auto v2 = vertices2[i];
        if (vertices1[i].idx.value != vertices2[i].idx.value)
            int i = 5;
        auto dpos = pos[vertices2[i]] * 1e6;
        mPositions[vertices1[i]] = pos_t(dpos);
    }

    //for (auto& face : mMesh.all_faces()) {
    auto af = mMesh.all_faces();
    auto afI = m.all_faces();
    auto invalidFacesFacescount = 0;
    auto invalidFacesFacescountI = 0;
    for (int i = 0; i < af.count(); i++) {
        auto& face = af[i];
        auto& faceI = afI[i];
        auto randomEdge = face.any_halfedge();
        auto t1 = randomEdge.vertex_from();
        auto t2 = randomEdge.vertex_to();
        auto t3 = randomEdge.next().vertex_to();
        auto x = mPositions[randomEdge.vertex_from()];
        auto y = mPositions[randomEdge.vertex_to()];
        auto z = mPositions[randomEdge.next().vertex_to()];
        auto test = tg::pos3(x);
        auto dir1 = y - x;
        auto dir2 = z - x;

        //######################################################
        auto randomEdgeI = faceI.any_halfedge();
        auto t1I = randomEdgeI.vertex_from();
        auto t2I = randomEdgeI.vertex_to();
        auto t3I = randomEdgeI.next().vertex_to();
        auto xI = pos[randomEdgeI.vertex_from()];
        auto yI = pos[randomEdgeI.vertex_to()];
        auto zI = pos[randomEdgeI.next().vertex_to()];
        auto dir1I = yI - xI;
        auto dir2I = zI - xI;
        auto n1 = tg::dvec3(tg::normalize(dir1I));
        auto n2 = tg::dvec3(tg::normalize(dir2I));
        auto n3 = n1 - n2;

        double eps = std::abs(n3.x) + std::abs(n3.y) + std::abs(n3.z);
        if (eps < 0.000001) {
            invalidFacesFacescount++;
            m.faces().remove(face);
        }

        //######################################################

        if (tg::normalize(tg::vec3(dir1)) == tg::normalize(tg::vec3(dir2)))
            invalidFacesFacescountI++;
    }
    std::cout << "Invalid faces: " << invalidFacesFacescount << " vs Invalid Faces Integer A. : " << invalidFacesFacescountI << std::endl;
    {
        auto view = gv::view(pos);
        gv::view(gv::lines(pos).line_width_world(0.01));
    }
    m.compactify();
    pm::save("../data/mesh/soma_gyroid_Z_2.obj", pos);
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
    ObjConfig conf = ObjCollection::map.at("fox_mesh_2");
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
    ObjConfig conf = ObjCollection::map.at("octree_easy");
    auto planeMesh1 = conf.getMeshA();
    auto planeMesh2 = conf.getMeshB();
            
    auto boxes = conf.getOctreeBoxes();
    //octree->startDebugView();
    //auto g = gv::grid();
    auto view = gv::view(planeMesh1->positions());
    gv::view(gv::lines(planeMesh1->positions()).line_width_world(1000000), "gv::lines(pos)");
    gv::view(planeMesh2->positions());
    gv::view(gv::lines(planeMesh2->positions()).line_width_world(1000000), "gv::lines(pos)");
    gv::view(gv::lines(boxes).line_width_world(500000), tg::color3::blue, "gv::lines(pos)");
}

void test_component_classification() {
    glow::timing::CpuTimer timer;
    timer.restart();
    ObjConfig conf = ObjCollection::map.at("Buddha");
    auto planeMesh1 = conf.getMeshA();
    auto planeMesh2 = conf.getMeshB();
    TG_ASSERT(planeMesh2->allFacesAreValid());
    auto octree = conf.getOctree();
    auto boxes = conf.getOctreeBoxes();
    std::cout << "Time for loading Mesh: " << timer.elapsedMilliseconds() << "ms" << std::endl;
    timer.restart();
    auto iCut = conf.getOctree()->cutPolygons();
    std::cout << "Time cutting the Mesh: " << timer.elapsedMilliseconds() << "ms" << std::endl;

    /*{
        planeMesh1->checkAndComputePositions();
        planeMesh2->checkAndComputePositions();

        planeMesh1->mesh().compactify();


        auto view = gv::view(planeMesh2->positions());
        gv::view(gv::lines(planeMesh2->positions()).line_width_world(1000000), gv::masked(iCut.getIntersectionEdgesMarkerB()), tg::color3::color(0.0));
        gv::view(gv::lines(planeMesh2->positions()).line_width_world(100000), tg::color3::color(0.0));
        //auto view = gv::view(planeMesh1->positions());
        //gv::view(gv::lines(planeMesh1->positions()).line_width_world(100000), tg::color3::color(0.0));
        //gv::view(gv::lines(planeMesh1->positions()).line_width_world(1000000), gv::masked(iCut.getIntersectionEdgesMarkerA()), tg::color3::color(0.0));
    }*/

    timer.restart();
    std::shared_ptr<FaceComponentFinder> components1 = std::make_shared<FaceComponentFinder>(*planeMesh1, iCut.getIntersectionEdgesMarkerA());
    std::shared_ptr<FaceComponentFinder> components2 = std::make_shared<FaceComponentFinder>(*planeMesh2, iCut.getIntersectionEdgesMarkerB());
    ComponentCategorization components(octree, components1, components2, iCut);
    std::cout << "Time categorization: " << timer.elapsedMilliseconds() << "ms" << std::endl;
    components.renderFinalResult(iCut);
}

void test_octree_cell_ray_cast() {
    ObjConfig conf = ObjCollection::map.at("raycast");
    auto planeMesh1 = conf.getMeshA();
    auto planeMesh2 = conf.getMeshB();
    TG_ASSERT(planeMesh2->allFacesAreValid());
    auto octree = conf.getOctree();
    auto boxes = conf.getOctreeBoxes();

    pm::vertex_handle origin = planeMesh1->mesh().all_vertices().first();
    SharedDebugRayInfo rayInfo = std::make_shared<DebugRayInfo>();
    auto intersections = octree->countIntersectionsToOutside2(origin, *planeMesh1, rayInfo);

    std::vector<tg::dsegment3> lines;
    for (int i = 0; i < (int)rayInfo->rayPath.size() - 1; ++i) {
        lines.push_back(tg::dsegment3{ ob::to_position(rayInfo->rayPath[i]), ob::to_position(rayInfo->rayPath[i + 1]) });
    }
    //lines.push_back(tg::dsegment3{ tg::dpos3(rayInfo->rayStartDirect), tg::dpos3(rayInfo->rayEndDirect) });

    for (int i = 0; i < (int)rayInfo->nexPointsCell.size() - 1; ++i) {
        lines.push_back(tg::dsegment3{ tg::dpos3(rayInfo->nexPointsCell[i]), tg::dpos3(rayInfo->nexPointsCell[i + 1]) });
    }

    std::vector<tg::aabb3> returnBoxes;
    for (auto box : rayInfo->rayBoxesDirect) {
        returnBoxes.push_back(tg::aabb3(tg::pos3(box.min), tg::pos3(box.max)));
    }

    auto const octreeCells = gv::lines(boxes).line_width_world(200000);
    auto const rayCells = gv::lines(returnBoxes).line_width_world(2500000);
    auto const rayPath = gv::lines(lines).line_width_world(3000000);
    auto const positions1 = planeMesh1->positions();
    auto const positionLines1 = gv::lines(planeMesh1->positions()).line_width_world(100000);
    auto const positions2 = planeMesh2->positions();
    auto const positionLines2 = gv::lines(planeMesh2->positions()).line_width_world(100000);
    bool tooglePolygons = true;

    if (tooglePolygons) {
        auto view = gv::view(octreeCells, tg::color3::blue);
        gv::view(rayCells, tg::color3::green);
        gv::view(rayPath, tg::color3::red);
        //gv::view(positions1);
        //gv::view(positionLines1);
        //gv::view(positions2);
        //gv::view(positionLines2);
    }

    /*gv::interactive([&](auto) {
        {
            //auto view = gv::view(planeMesh1->positions());
            //auto view = gv::view(gv::lines(planeMesh1->positions()).line_width_world(100000), "gv::lines(pos)");
            //gv::view(planeMesh2->positions());

            //gv::view(gv::lines(planeMesh2->positions()).line_width_world(100000), "gv::lines(pos)");
            auto view = gv::view(octreeCells, tg::color3::blue);
            gv::view(rayCells, tg::color3::green);
            gv::view(rayPath, tg::color3::red);

            if (tooglePolygons) {
                gv::view(positions1);
                gv::view(positionLines1);
                gv::view(positions2);
                gv::view(positionLines2);
            }

            ImGui::Begin("Move");
            if (ImGui::IsKeyPressed('T')) {
                tooglePolygons = !tooglePolygons;
                gv::view_clear_accumulation();
            }
            ImGui::End();
        }
    });*/

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

    planeMesh1.checkAndComputePositions();
    planeMesh2.checkAndComputePositions();

    planeMesh1.mesh().compactify();

    {
        auto view = gv::view(planeMesh2.positions());       
        gv::view(gv::lines(planeMesh2.positions()).line_width_world(10000), tg::color3::color(0.0));
        gv::view(gv::lines(planeMesh2.positions()).line_width_world(100000), gv::masked(iCut.getIntersectionEdgesMarkerB()), tg::color3::color(0.0));
        gv::view(planeMesh1.positions());
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