#include <nexus/app.hh>
#include <intersection_cut.hh>
#include <obj_config.hh>
#include <octree.hh>
#include <face_component_finder.hh>
#include <polymesh/algorithms/deduplicate.hh>

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
    ObjConfig conf = ObjCollection::map.at("complex_1");
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

    planeMesh1->mesh().compactify();
    auto faceMask1 = planeMesh1->mesh().faces().make_attribute_with_default(false);
    auto faceMask2 = planeMesh2->mesh().faces().make_attribute_with_default(false);

    auto test1 = planeMesh1->noDuplicatedVerticesInFaces(faceMask1);
    auto test2 = planeMesh2->noDuplicatedVerticesInFaces(faceMask2);

    auto view = gv::view(planeMesh2->positions(), components2.getColorAssignment());
    gv::view(gv::lines(planeMesh2->positions()).line_width_world(1000000), gv::masked(iCut.getIntersectionEdgesMarkerB()), tg::color3::color(0.0));
    gv::view(gv::lines(planeMesh2->positions()).line_width_world(100000), tg::color3::color(0.0));
}