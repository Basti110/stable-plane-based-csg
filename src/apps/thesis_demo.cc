#include <nexus/app.hh>
#include <intersection_cut.hh>
#include <obj_config.hh>
#include <octree.hh>
#include <face_component_finder.hh>
#include <polymesh/algorithms/deduplicate.hh>
#include <component_categorization.hh>
#include <iomanip>
//#include <ctracer/trace-config.hh>
#define GIGA 1000000000
#define MEGA 1000000
#define CLOCK 3799999

APP("App::Plane_Geometry_Visu") {
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
}

APP("App::Picker") {
    ObjConfig conf = ObjCollection::map.at("fox_mesh_2");
    conf.getOctree()->startDebugView();
}

APP("App::Show_Mesh") {
    ObjConfig conf = ObjCollection::map.at("Buddha");
    conf.viewMesh(true);
}

APP("App::Picker_Cut") {
    ObjConfig conf = ObjCollection::map.at("fox_mesh_2");
    auto planeMesh1 = conf.getMeshA();
    auto planeMesh2 = conf.getMeshB();

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    auto iCut = conf.getOctree()->cutPolygons();
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    auto seconds = std::chrono::duration_cast<std::chrono::milliseconds> (end - begin).count();
    std::cout << "Cut in " << seconds << "ms" << std::endl;

    planeMesh1->mesh().compactify();
    planeMesh2->mesh().compactify();

    /*std::vector<pm::face_handle> facesMesh1;
    std::vector<pm::face_handle> facesMesh2;
    bool allVerticesOnPlane1 = planeMesh1->allVerticesInFacePlane(facesMesh1);
    bool allVerticesOnPlane2 = planeMesh2->allVerticesInFacePlane(facesMesh2);*/

    TG_ASSERT(planeMesh1->allFacesHaveEdges());
    TG_ASSERT(planeMesh2->allFacesHaveEdges());

    planeMesh1->checkAndComputePositions();
    planeMesh2->checkAndComputePositions();

    auto faceMask1 = planeMesh1->mesh().faces().make_attribute_with_default(false);
    auto faceMask2 = planeMesh2->mesh().faces().make_attribute_with_default(false);

    auto test1 = planeMesh1->noDuplicatedVerticesInFaces(faceMask1);
    auto test2 = planeMesh2->noDuplicatedVerticesInFaces(faceMask2);

    //int deduplicate1 = pm::deduplicate(planeMesh1->mesh(), planeMesh1->positions());
    //int deduplicate2 = pm::deduplicate(planeMesh2->mesh(), planeMesh2->positions());
    std::vector<pm::face_handle> facesMesh1;
    std::vector<pm::face_handle> facesMesh2;
    bool allVerticesOnPlane1 = planeMesh1->allVerticesInFacePlane(facesMesh1);
    bool allVerticesOnPlane2 = planeMesh2->allVerticesInFacePlane(facesMesh2);

    scalar_t scale = 1e6;
    AABB box({ -60 * scale, -60 * scale, -40 * scale }, { 60 * scale, 60 * scale, 80 * scale });
    SharedOctree newOctree = std::make_shared<Octree>(&(*planeMesh1), &(*planeMesh2), box);

    for (auto f : planeMesh1->mesh().faces()) {
        newOctree->insert_polygon(planeMesh1->id(), f);
    }

    for (auto f : planeMesh2->mesh().faces()) {
        newOctree->insert_polygon(planeMesh2->id(), f);
    }
    newOctree->startDebugView();
}

APP("App::test_cut_mesh") {
    ObjConfig conf = ObjCollection::map.at("fox_mesh_2");
    auto planeMesh1 = conf.getMeshA();
    auto planeMesh2 = conf.getMeshB();

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    auto iCut = conf.getOctree()->cutPolygons();
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    auto seconds = std::chrono::duration_cast<std::chrono::milliseconds> (end - begin).count();
    std::cout << "Cut in " << seconds << "ms" << std::endl;

    planeMesh1->checkAndComputePositions();
    planeMesh2->checkAndComputePositions();

    planeMesh1->mesh().compactify();
    auto faceMask1 = planeMesh1->mesh().faces().make_attribute_with_default(false);
    auto faceMask2 = planeMesh2->mesh().faces().make_attribute_with_default(false);

    auto test1 = planeMesh1->noDuplicatedVerticesInFaces(faceMask1);
    auto test2 = planeMesh2->noDuplicatedVerticesInFaces(faceMask2);

    //int deduplicate1 = pm::deduplicate(planeMesh1->mesh(), planeMesh1->positions());
    //int deduplicate2 = pm::deduplicate(planeMesh2->mesh(), planeMesh2->positions());
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

    int deduplicate1 = pm::deduplicate(planeMesh1->mesh(), planeMesh1->positions());
    int deduplicate2 = pm::deduplicate(planeMesh2->mesh(), planeMesh2->positions());
    {
        auto view = gv::view(planeMesh2->positions());
        gv::view(gv::lines(planeMesh2->positions()).line_width_world(10000), tg::color3::color(0.0));
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

APP("App::component_classification") {
    ObjConfig conf = ObjCollection::map.at("fox_mesh_2");
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

    planeMesh1->checkAndComputePositions();
    planeMesh2->checkAndComputePositions();

    //planeMesh1->mesh().compactify();
    //auto faceMask1 = planeMesh1->mesh().faces().make_attribute_with_default(false);
    //auto faceMask2 = planeMesh2->mesh().faces().make_attribute_with_default(false);

    //auto test1 = planeMesh1->noDuplicatedVerticesInFaces(faceMask1);
    //auto test2 = planeMesh2->noDuplicatedVerticesInFaces(faceMask2);

    /*auto view = gv::view(planeMesh1->positions(), components1.getColorAssignment());
    gv::view(gv::lines(planeMesh1->positions()).line_width_world(20000), gv::masked(iCut.getIntersectionEdgesMarkerA()), tg::color3::green);
    gv::view(gv::lines(planeMesh1->positions()).line_width_world(10000), tg::color3::color(0.0));
    gv::view(gv::lines(planeMesh2->positions()).line_width_world(10000), tg::color3::color(0.0));*/

    auto view = gv::view(planeMesh2->positions(), components2.getColorAssignment());
    gv::view(gv::lines(planeMesh2->positions()).line_width_world(20000), gv::masked(iCut.getIntersectionEdgesMarkerB()), tg::color3::green);
    gv::view(gv::lines(planeMesh2->positions()).line_width_world(10000), tg::color3::color(0.0));
    gv::view(gv::lines(planeMesh1->positions()).line_width_world(10000), tg::color3::color(0.0));
}

APP("App::count_intersections") {
    ObjConfig conf = ObjCollection::map.at("Buddha_90");
    auto planeMesh1 = conf.getMeshA();
    auto planeMesh2 = conf.getMeshB();

    /*int noBoundaryCount = ObjConfig::meshHasBoundaries(planeMesh1);
    std::cout << "Mesh 1 has Boundaries: " << noBoundaryCount << std::endl;
    noBoundaryCount = ObjConfig::meshHasBoundaries(planeMesh2);
    std::cout << "Mesh 2 has Boundaries: " << noBoundaryCount << std::endl;*/
    conf.viewMeshWithBoundaries(planeMesh2);
    std::vector<pm::halfedge_handle> boundaries2;
    ObjConfig::meshHasBoundaries(planeMesh2, boundaries2);
    /*int idx1 = boundaries2[0].idx.value;
    int idx2 = boundaries2[1].idx.value;
    int idx3 = boundaries2[2].idx.value;
    int idx1Next = boundaries2[0].next().idx.value;
    int idx1NextNext = boundaries2[0].next().next().idx.value;*/

    auto octree = conf.getOctree();

    glow::timing::CpuTimer timer;
    auto list = std::make_shared<std::unordered_map<int, std::unordered_set<int>>>();
    int intersections = octree->countIntersections(list);
    std::cout << "Intersections: " << intersections << std::endl;
    std::cout << "Time: " << timer.elapsedMilliseconds() << "ms" << std::endl;
}

APP("Benchmark:SubdetVsIntPos") {
    ObjConfig conf = ObjCollection::map.at("cubes");
    auto planeMesh = conf.getMeshA();
    
    int iterations = 30000;
    auto faces = planeMesh->faces().to_vector();
    auto positions = planeMesh->positions().to_vector();
    auto posHandles = planeMesh->mesh().vertices().to_vector();
    auto vec1 = std::vector<int8_t>(iterations);
    auto vec2 = std::vector<int8_t>(iterations);

    glow::timing::CpuTimer timer;
    for (int i = 0; i < iterations; ++i) {
        auto dis = ob::signed_distance(faces[i], positions[i]);
        vec1[i] = dis > 0 ? 1 : dis < 0 ? -1 : 0;
    }
    std::cout << "Time signed_distance: " << timer.elapsedMilliseconds() << "ms" << std::endl;

    timer.restart();
    std::cout << "Timer: " << timer.elapsedMilliseconds() << "ms" << std::endl;
    for (int i = 0; i < iterations; ++i) {
        SubDet det = planeMesh->pos(posHandles[i]);
        auto dis = ob::classify_vertex(det, faces[i]);
        vec2[i] = dis;
    }
    std::cout << "Time classify_vertex: " << timer.elapsedMilliseconds() << "ms" << std::endl;

    int differs = 0;
    for (int i = 0; i < iterations; ++i) {
        if (vec1[i] != vec2[i])
            differs++;
    }
    std::cout << "result differs in " << differs << " values" << std::endl;
}

APP("Benchmark:TestAvgTime") {
    int testCount = 100;   
    ct::scope s;
    glow::timing::CpuTimer timer;
    for (int i = 0; i < testCount; i++) {       
        TRACE("Test");
        ObjConfig conf = ObjCollection::map.at("fox_mesh_2");
        auto planeMesh1 = conf.getMeshA();
        auto planeMesh2 = conf.getMeshB();
        auto iCut = conf.getOctree()->cutPolygons();
    }

    auto t = s.trace().compute_location_stats();
    for (auto te : t) {
        std::cout << te.loc->name << ": " << te.total_cycles / (double)1000000000 <<  std::endl;
    }
    auto c = s.trace().elapsed_cycles();
    auto m = timer.elapsedMillisecondsD();
    std::cout << "approx CPU frequence = " << c / m << std::endl;
    std::cout << "approx time = "  << c / (double)CLOCK << std::endl;
    std::cout << "Time: " << m << "ms" << std::endl;
    std::cout << "scope  cycles " << s.trace().elapsed_cycles() / (double)1000000000 << "G" << std::endl;
    std::cout << "AVG Time with  " << testCount << " iterations: " << m / testCount  << "ms" << std::endl;
}

APP("Benchmark:TestOctree") {
    int testStartSize = 5;
    int testEndSize = 100;
    ct::scope s;
    ObjConfig conf = ObjCollection::map.at("Lucy_10");
    std::cout << "[";
    for (int i = testStartSize; i <= testEndSize; i+=2) {
        TRACE("Test");
        
        conf.setMaxObjInCell(i);
        auto planeMesh1 = conf.getMeshA();
        auto planeMesh2 = conf.getMeshB();
        glow::timing::CpuTimer timer;
        SharedOctree octree = conf.getOctree();
        IntersectionCut iCut = octree->cutPolygons();
        auto components1 = std::make_shared<FaceComponentFinder>(*(conf.getMeshA()), iCut.getIntersectionEdgesMarkerA());
        auto components2 = std::make_shared<FaceComponentFinder>(*(conf.getMeshB()), iCut.getIntersectionEdgesMarkerB());
        auto components = std::make_shared<ComponentCategorization>(octree, components1, components2, iCut);
        std::cout << "[" << i << ", " << timer.elapsedMillisecondsD() << "], ";
        conf.reset();
    }
    std::cout << "]" << std::endl;
}

void printStats(ct::scope& s) {
    auto trace = s.trace();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(trace.time_end() - trace.time_start()).count();
    std::cout << std::endl << "-Timing Stats [" << s.name() << "], total: " << elapsed << "ms" << std::endl;
    auto t = trace.compute_location_stats();
    for (auto te : t) {
        auto cycles = te.total_cycles;
        size_t length = std::strlen(te.loc->name);
        std::cout << "   |- " << te.loc->name << std::right << std::setw(50 - length);
        std::cout << cycles / (double)GIGA << "G" << " Cycles";
        std::cout << ", Time: " << cycles / (double)CLOCK << " ms";
        std::cout << ", Samples: " << te.samples << std::endl;
    }
}

APP("Benchmark:OneIteration") {
    int testCount = 1;
    ct::scope rootScope;
    ObjConfig conf = ObjCollection::map.at("co-planar-cubes2");

    std::cout << "#############################################################" << std::endl;
    std::cout << "#######                 Benchmark                     #######" << std::endl;
    std::cout << "#############################################################" << std::endl;

    //Load Mesh
    SharedPlaneMesh planeMesh1;
    SharedPlaneMesh planeMesh2;
    {   
        ct::scope s("Load Mesh");
        planeMesh1 = conf.getMeshA();
        planeMesh2 = conf.getMeshB();
        printStats(s);
    }
    auto facesInSceneBefore = planeMesh1->mesh().faces().size() + planeMesh2->mesh().faces().size();
    //bool test = planeMesh1->allHalfEdgesAreValid();
    glow::timing::CpuTimer timer;
    //Load Octree
    SharedOctree octree;
    {
        ct::scope s("Build Octree");
        octree = conf.getOctree();
        printStats(s);
    }
    //conf.viewMesh(true);
    //Cut Mesh
    IntersectionCut iCut;
    {
        ct::scope s("Cut Mesh");
        iCut = octree->cutPolygons();
        printStats(s);
    }
    //
    //Categorization
    int countComponents = 0;
    std::shared_ptr<ComponentCategorization> components;
    {
        ct::scope s("Categorization");
        std::shared_ptr<FaceComponentFinder> components1 = std::make_shared<FaceComponentFinder>(*planeMesh1, iCut.getIntersectionEdgesMarkerA());
        std::shared_ptr<FaceComponentFinder> components2 = std::make_shared<FaceComponentFinder>(*planeMesh2, iCut.getIntersectionEdgesMarkerB());
        components = std::make_shared<ComponentCategorization>(octree, components1, components2, iCut);
        countComponents = components1->countComponents() + components2->countComponents();
        printStats(s);
    }
    
    
    auto trace = rootScope.trace();
    auto c = trace.elapsed_cycles();
    auto m = std::chrono::duration_cast<std::chrono::milliseconds>(trace.time_end() - trace.time_start()).count();
    auto t = timer.elapsedMillisecondsD();
    auto facesInSceneAfter = planeMesh1->mesh().faces().size() + planeMesh2->mesh().faces().size();
    std::cout << std::endl;
    std::cout << "----------- Total ---------- " << std::endl;
    std::cout << "Time: " << m << "ms" << std::endl;
    std::cout << "Time CTracer: " << c / (double)CLOCK << "ms (Maybe not exact. Dependent on constant CPU frequence)" << std::endl;
    std::cout << "Time Without PM Load: " << t + conf.initMeshTime() << "ms" << std::endl;
    std::cout << "scope: " << rootScope.trace().elapsed_cycles() / (double)1000000000 << "G cycle" << std::endl;
    std::cout << "Components: " << countComponents << std::endl;
    std::cout << "Faces before: " << facesInSceneBefore << std::endl;
    std::cout << "Faces after: " << facesInSceneAfter << std::endl;
    conf.getOctree()->printOctreeStats();
    iCut.printTimes();
    //conf.viewMesh(true);
    components->renderFinalResult(iCut, 1000); //10000);
}


APP("App:ShowMeshOrOctree") {
    ObjConfig conf = ObjCollection::map.at("cubes");
    conf.setMaxObjInCell(200);
    auto octree = conf.getOctree();
    auto boxes = conf.getOctreeBoxes();
    SharedPlaneMesh planeMesh1 = conf.getMeshA();
    SharedPlaneMesh planeMesh2 = conf.getMeshB();
    planeMesh1->checkAndComputePositions();
    {
        auto view = gv::view(planeMesh1->positions(), gv::print_mode);
        gv::view(gv::lines(planeMesh1->positions()).line_width_world(10000), tg::color3::color(0.0));
        gv::view(planeMesh2->positions(), gv::camera_transform(tg::pos3(-704690.f, 1220410.f, 4364950.f), tg::pos3(848812.f, 735278.f, 2153600.f)));
        gv::view(gv::lines(planeMesh2->positions()).line_width_world(10000), tg::color3::color(0.0));
    }
    glow::info() << gv::get_last_close_info().cam_pos;
    glow::info() << gv::get_last_close_info().cam_target;
    //gv::view(gv::lines(boxes).line_width_world(500000), tg::color3::blue);
}

void transformation(const pm::vertex_attribute<tg::pos3>& in, pm::vertex_attribute<tg::dpos3>& out, tg::dmat4& mat, int scale) {
    const auto& mesh = in.mesh();
    for (auto f : mesh.all_vertices()) {
        auto pos4 = tg::dvec4(in[f] * scale, 1);
        pos4 = (mat * pos4);
        out[f] = tg::dpos3(pos4);
    }
}

APP("App:ShowCSG") {
    //const char* meshList[]{ "cube", "cubes1", "cubes2", "fox", "buddha", "bunny", "Armadillo_2" };
    const char* meshList[]{ "cubes1", "cubes2", "bunny", "Armadillo_2" };
    const char* operationList[]{ "Mesh 1 + Mesh 2", "Mesh 2 AND Mesh 1", "Mesh 2 OR Mesh 1", "Mesh 1 - Mesh 2", "Mesh 2 - Mesh 1", "Mesh 1", "Mesh 2" };
    const char* colorList[]{ "White", "Components", "In/Out" };


    int scaleList[]{ 1e7, 1e7, 1e6, 1e6 };
    int selectedMesh1 = 0;
    int selectedMesh2 = 1;
    int meshDistance = 80;

    std::string path_cube1 = std::string("../data/mesh/") + std::string(meshList[selectedMesh1]) + std::string(".obj");
    std::string path_cube2 = std::string("../data/mesh/") + std::string(meshList[selectedMesh2]) + std::string(".obj");
    //int scale = 1e6;
    //std::string path_cube = "../data/mesh/cubes2";
    //ObjConfig conf = ObjCollection::map.at("fox_mesh_2");
    pm::Mesh mesh1;
    pm::vertex_attribute<tg::pos3> posf1(mesh1);
    pm::vertex_attribute<tg::dpos3> pos1(mesh1);
    pm::load(path_cube1, mesh1, posf1);
    pm::Mesh mesh2;
    pm::vertex_attribute<tg::pos3> posf2(mesh2);
    pm::vertex_attribute<tg::dpos3> pos2(mesh2);
    pm::load(path_cube2, mesh2, posf2);

    for (auto f : mesh1.all_vertices())
        pos1[f] = ((tg::dpos3)posf1[f] * scaleList[selectedMesh1]);


    
    //for (auto f : mesh2.all_vertices())
        //pos2[f] = ((tg::dpos3)posf2[f] * scale);*/

    //SharedPlaneMesh planeMesh1 = conf.getMeshA();
    //SharedPlaneMesh planeMesh2 = conf.getMeshB();
    //SharedOctree octree = conf.getOctree();
    //IntersectionCut iCut = octree->cutPolygons();
    //std::shared_ptr<FaceComponentFinder> components1 = std::make_shared<FaceComponentFinder>(*planeMesh1, iCut.getIntersectionEdgesMarkerA());
    //std::shared_ptr<FaceComponentFinder> components2 = std::make_shared<FaceComponentFinder>(*planeMesh2, iCut.getIntersectionEdgesMarkerB());
    //std::shared_ptr<ComponentCategorization> components = std::make_shared<ComponentCategorization>(octree, components1, components2, iCut);
    //
    gv::SharedCameraController cam = gv::CameraController::create();
    bool isCut = false;
    ObjConfig config;

    //Data
    gv::SharedGeometricRenderable renderable1;
    gv::SharedGeometricRenderable renderable2;
    gv::Masking maskFaceAIn;
    gv::Masking maskFaceBIn;
    gv::Masking maskFaceAOut;
    gv::Masking maskFaceBOut;
    
    pm::face_attribute<bool> colorAtrMaskAIn;
    pm::face_attribute<bool> colorAtrMaskBIn;
    pm::face_attribute<bool> colorAtrMaskAOut;
    pm::face_attribute<bool> colorAtrMaskBOut;
    pm::face_attribute<tg::color3> colorsA;
    pm::face_attribute<tg::color3> colorsB;
    gv::SharedLineRenderable octreeRenderable;
    //std::string meshList[]{ "Mesh 1", "Mesh 2" };
    std::shared_ptr<FaceComponentFinder> components1;
    std::shared_ptr<FaceComponentFinder> components2;
    std::shared_ptr<ComponentCategorization> components;

    bool showOctree = false;
    int tooglePolygons = 0;
    int colorMode = 0;
    float degreeX = 0;
    float degreeY = 0;
    float degreeZ = 0;
    renderable1 = gv::make_renderable(pos1);
    //renderable2 = gv::make_renderable(pos2);

    gv::interactive([&](auto dt) {
        
        tg::mat4 projectionMatrix = cam->computeProjMatrix();
        tg::mat4 viewMatrix = cam->computeViewMatrix();
        auto const mousePos = gv::experimental::interactive_get_mouse_position();
        auto const windowSize = gv::experimental::interactive_get_window_size();

        float x = (2.0f * mousePos.x) / windowSize.width - 1.0f;
        float y = 1.0f - (2.0f * mousePos.y) / windowSize.height;
        tg::vec4 rayClip = tg::vec4(x, y, -1.0, 1.0);
        tg::vec4 rayEye = tg::inverse(projectionMatrix) * rayClip;
        rayEye = tg::vec4(rayEye.x, rayEye.y, -1, 0);
        tg::vec4 rayWorld4 = tg::inverse(viewMatrix) * rayEye;
        tg::vec3 rayWorld = tg::vec3(rayWorld4);
        rayWorld = tg::normalize(rayWorld);

        auto camPos = cam->getPosition();
        tg::dmat4 transform = tg::translation(tg::dvec3(camPos) + (tg::dvec3(rayWorld) * meshDistance * 1e6));
        auto rot = tg::rotation_x(tg::angle::from_degree(degreeX)) * tg::rotation_y(tg::angle::from_degree(degreeY)) * tg::rotation_z(tg::angle::from_degree(degreeZ));
        transformation(posf2, pos2, transform * tg::dmat4(rot), scaleList[selectedMesh2]);

        //if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        ImGui::Begin("Move");
        if (ImGui::IsKeyPressed('E')) {
            if (!isCut) {
                isCut = true;
                std::cout << transform[3][0] << ":" << transform[3][1] << ":" << transform[3][2] << ":" << transform[3][3] << std::endl;
                config = ObjConfig(1e7, AABB({ -15, -15, -10 }, { 15, 15, 20 }), pos1, pos2);
                glow::timing::CpuTimer timer;
                auto boxes = config.getOctreeBoxes();
                SharedOctree octree = config.getOctree();
                IntersectionCut iCut = octree->cutPolygons();
                components1 = std::make_shared<FaceComponentFinder>(*(config.getMeshA()), iCut.getIntersectionEdgesMarkerA());
                components2 = std::make_shared<FaceComponentFinder>(*(config.getMeshB()), iCut.getIntersectionEdgesMarkerB());
                components = std::make_shared<ComponentCategorization>(octree, components1, components2, iCut);
                octree->getPlaneMeshA().checkAndComputePositions();
                octree->getPlaneMeshB().checkAndComputePositions();
                colorsA = components->getColorToStateA();
                colorsB = components->getColorToStateB();
                std::cout << "CSG Operation: " << timer.elapsedMillisecondsD() << "ms" << std::endl;

                //Attributes
                colorAtrMaskAIn = pm::face_attribute<bool>(colorsA.map([](tg::color3 c) { return c == tg::color3::white; }));
                colorAtrMaskBIn = pm::face_attribute<bool>(colorsB.map([](tg::color3 c) { return c == tg::color3::white; }));
                colorAtrMaskAOut = pm::face_attribute<bool>(colorsA.map([](tg::color3 c) { return c != tg::color3::white; }));
                colorAtrMaskBOut = pm::face_attribute<bool>(colorsB.map([](tg::color3 c) { return c != tg::color3::white; }));

                //Maskes
                maskFaceAIn = gv::masked(colorAtrMaskAIn);
                maskFaceBIn = gv::masked(colorAtrMaskBIn);
                maskFaceAOut = gv::masked(colorAtrMaskAOut);
                maskFaceBOut = gv::masked(colorAtrMaskBOut);

                //Renderables
                renderable1 = gv::make_renderable(config.getMeshA()->positions());
                renderable2 = gv::make_renderable(config.getMeshB()->positions());
                octreeRenderable = gv::make_renderable(gv::lines(boxes).line_width_world(100000));
                gv::configure(*renderable1, tg::color3(0.95));
                gv::configure(*renderable2, tg::color3(0.5));
                int tooglePolygons = 0;
                int colorMode = 0;
            }
        }
        
        if (isCut) {
            //bool toogled = ImGui::Checkbox("Show Octree", &showOctree);
            bool toogled = ImGui::Combo("CSG Operation", &tooglePolygons, operationList, IM_ARRAYSIZE(operationList));
            toogled |= ImGui::Combo("Color", &colorMode, colorList, IM_ARRAYSIZE(colorList));

            if (toogled) {
                renderable1 = gv::make_renderable(config.getMeshA()->positions());
                renderable2 = gv::make_renderable(config.getMeshB()->positions());
                if (tooglePolygons == 1) {
                    renderable1->setMasking(maskFaceAOut);
                    renderable2->setMasking(maskFaceBOut);
                }
                else if (tooglePolygons == 2) {
                    renderable1->setMasking(maskFaceAIn);
                    renderable2->setMasking(maskFaceBIn);
                }
                else if (tooglePolygons == 3) {
                    renderable1->setMasking(maskFaceAOut);
                    renderable2->setMasking(maskFaceBIn);
                }
                else if (tooglePolygons == 4) {
                    renderable1->setMasking(maskFaceAIn);
                    renderable2->setMasking(maskFaceBOut);
                }
                if (colorMode == 0) {
                    gv::configure(*renderable1, tg::color3(0.95));
                    gv::configure(*renderable2, tg::color3(0.5));
                }
                else if (colorMode == 1) {
                    gv::configure(*renderable1, components1->getColorAssignment());
                    gv::configure(*renderable2, components2->getColorAssignment());
                }
                else if (colorMode == 2) {
                    gv::configure(*renderable1, colorsA);
                    gv::configure(*renderable2, colorsB);
                }
                //::view_clear_accumulation();
            }
        }
        else {
            bool newMesh1 = ImGui::Combo("Base Mesh", &selectedMesh1, meshList, IM_ARRAYSIZE(meshList));
            bool newMesh2 = ImGui::Combo("Mouse Mesh", &selectedMesh2, meshList, IM_ARRAYSIZE(meshList));

            if (newMesh1) {
                auto meshPath = std::string("../data/mesh/") + meshList[selectedMesh1] + std::string(".obj");
                pm::load(meshPath, mesh1, posf1);
                for (auto f : mesh1.all_vertices())
                    pos1[f] = ((tg::dpos3)posf1[f] * scaleList[selectedMesh1]);
                renderable1 = gv::make_renderable(pos1);
            }
            else if (newMesh2) {
                auto meshPath = std::string("../data/mesh/") + meshList[selectedMesh2] + std::string(".obj");
                pm::load(meshPath, mesh2, posf2);
            }
        }
        ImGui::Checkbox("Show Octree", &showOctree);

        if (ImGui::IsKeyPressed('Q')) {
            if (isCut) {
                isCut = false;
                tooglePolygons = 0;
                colorMode = 0;
                degreeX = 0;
                degreeY = 0;
                degreeZ = 0;
                renderable1 = gv::make_renderable(pos1);
                //renderable2 = gv::make_renderable(pos2);
            }             
        }

        if (ImGui::IsKeyPressed('2')) 
            meshDistance+=5;
       
        if (ImGui::IsKeyPressed('1')) 
            meshDistance-=5;

        if (ImGui::IsKeyPressed('X'))
            degreeX+=5;

        if (ImGui::IsKeyPressed('Z'))
            degreeY+= 5;

        if (ImGui::IsKeyPressed('Y'))
            degreeZ+= 5;
        
        auto view = gv::view();
        if (!isCut) {
            gv::view(renderable1, cam);
            gv::view(pos2, gv::no_grid, gv::print_mode);
        }
        else {           
            if (showOctree)
                gv::view(octreeRenderable, tg::color3::blue, "gv::lines(pos)");
            if (tooglePolygons != 6)
                gv::view(renderable1, cam, gv::print_mode);
            if (tooglePolygons != 5)
                gv::view(renderable2, cam, gv::print_mode);
        }       

        ImGui::End();
    });
}