#pragma once
#include <nexus/app.hh>
#include <intersection_cut.hh>
#include <octree.hh>
//#include <plane_polygon.hh>
#include <face_component_finder.hh>
#include <polymesh/algorithms/deduplicate.hh>
#include <component_categorization.hh>

class CSGObject {
public:
    SharedPlaneMesh mesh1;
    SharedPlaneMesh mesh2;
    std::shared_ptr<ComponentCategorization> components;
};

SharedOctree generateOctree(SharedPlaneMesh m1, SharedPlaneMesh m2, const AABB& aabb) {
    auto octree = std::make_shared<Octree>(&(*m1), &(*m2), aabb);

    //Set max polygons in Cell
    octree->setOption(Octree::Options::MAX_OBJ_IN_CELL, 22);

    //Fill Octree with faces
    for (auto f : m1->allFaces()) {
        octree->insert_polygon(m1->id(), f);
    }

    for (auto f : m2->allFaces()) {
        octree->insert_polygon(m1->id(), f);
    }  
    return octree;
}


void getCSGobject(const pm::vertex_attribute<tg::pos3>& pos1, const pm::vertex_attribute<tg::pos3> pos2, int scale = 10e6) {
    CSGObject csgObj;
    const pm::Mesh& m1 = pos1.mesh();
    const pm::Mesh& m2 = pos1.mesh();
    csgObj.mesh1 = std::make_shared<PlaneMesh>(m1, pos1, scale);
    csgObj.mesh2 = std::make_shared<PlaneMesh>(m2, pos2, scale);

    //Todo: dynamic octree cell size. Fits for nearly all meshes in thingi10K
    auto octree = generateOctree(csgObj.mesh1, csgObj.mesh2, AABB(pos_t({ -100, -100, -100 }) * scale, pos_t({ 100, 100, 100 }) * scale ));

    //Cut Mesh
    IntersectionCut iCut;
    {
        ct::scope s("Cut Mesh");
        iCut = octree->cutPolygons();
    }
    
    //Categorization
    int countComponents = 0;
    std::shared_ptr<ComponentCategorization> components;
    {
        ct::scope s("Categorization");
        std::shared_ptr<FaceComponentFinder> components1 = std::make_shared<FaceComponentFinder>(*csgObj.mesh1, iCut.getIntersectionEdgesMarkerA());
        std::shared_ptr<FaceComponentFinder> components2 = std::make_shared<FaceComponentFinder>(*csgObj.mesh2, iCut.getIntersectionEdgesMarkerB());
        components = std::make_shared<ComponentCategorization>(octree, components1, components2, iCut);
        countComponents = components1->countComponents() + components2->countComponents();
    }

    //Render results
    //components->renderFinalResult(iCut, 1000); //10000);
}