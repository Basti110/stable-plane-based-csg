#pragma once
#include <nexus/app.hh>
#include <intersection_cut.hh>
#include <octree.hh>
//#include <plane_polygon.hh>
#include <face_component_finder.hh>
#include <polymesh/algorithms/deduplicate.hh>
#include <component_categorization.hh>

namespace csg {
bool copyFacesFromMeshOut(pm::vertex_attribute<tg::pos3>& pos, pm::Mesh& mesh, const VertexAttribute& copyFrom, pm::face_attribute<bool>& copyFaces, int scale = 10e6);
bool copyFacesFromMeshIn(pm::vertex_attribute<tg::pos3>& pos, pm::Mesh& mesh, const VertexAttribute& copyFrom, pm::face_attribute<bool>& copyFaces, int scale = 10e6);

class CSGObject {
public:
    SharedPlaneMesh mesh1;
    SharedPlaneMesh mesh2;
    SharedOctree octree;
    std::shared_ptr<ComponentCategorization> components;
    int scale = 0;
};

enum class Operation : int8_t {
    INTERSECTION = 0,
    UNION = 1,
    M1_MINUS_M2 = 2,
    M2_MINUS_M1 = 3,
};

pm::vertex_attribute<tg::pos3> applyCSG(CSGObject& csgObject, Operation csg, pm::Mesh& mesh) {
    auto pos = mesh.vertices().make_attribute<tg::pos3>();
    int8_t insideCode = (int8_t)ComponentCategorization::InOutState::INSIDE;
    int8_t outsideCode = (int8_t)ComponentCategorization::InOutState::OUTSIDE;
    int8_t coplanarSameCode = (int8_t)ComponentCategorization::InOutState::COPLANAR_SAME;
    int8_t coplanarOppositeCode = (int8_t)ComponentCategorization::InOutState::COPLANAR_OPPOSITE;
    csgObject.mesh1->checkAndComputePositions();
    csgObject.mesh2->checkAndComputePositions();
    auto test = csgObject.mesh1->faces().size();

    //pm::Mesh resultMesh;
    //pm::vertex_attribute<tg::pos3> resultPos(resultMesh);
    if (csg == Operation::INTERSECTION) {
        auto maskInA = csgObject.components->getStateMaskA(insideCode, coplanarSameCode);
        auto maskInB = csgObject.components->getStateMaskB(insideCode, -2);
        copyFacesFromMeshIn(pos, mesh, csgObject.mesh1->positions(), maskInA);
        copyFacesFromMeshIn(pos, mesh, csgObject.mesh2->positions(), maskInB);
    }
    else if (csg == Operation::UNION) {
        auto maskOutA = csgObject.components->getStateMaskA(outsideCode, coplanarSameCode);
        auto maskOutB = csgObject.components->getStateMaskB(outsideCode, -2);
        copyFacesFromMeshOut(pos, mesh, csgObject.mesh1->positions(), maskOutA);
        copyFacesFromMeshOut(pos, mesh, csgObject.mesh2->positions(), maskOutB);
    }
    else if (csg == Operation::M1_MINUS_M2) {
        auto maskInA = csgObject.components->getStateMaskA(insideCode, coplanarOppositeCode);
        auto maskOutB = csgObject.components->getStateMaskB(outsideCode, -2);
        copyFacesFromMeshIn(pos, mesh, csgObject.mesh1->positions(), maskInA);
        copyFacesFromMeshOut(pos, mesh, csgObject.mesh2->positions(), maskOutB);
    }
    else if (csg == Operation::M2_MINUS_M1) {
        auto maskOutA = csgObject.components->getStateMaskA(outsideCode, coplanarOppositeCode);
        auto maskInB = csgObject.components->getStateMaskB(insideCode, -2);
        copyFacesFromMeshOut(pos, mesh, csgObject.mesh1->positions(), maskOutA);
        copyFacesFromMeshIn(pos, mesh, csgObject.mesh2->positions(), maskInB);
    }
    //copyFacesFromMeshIn(resultPos, resultMesh, planeMeshA.positions(), colorAtrMaskAOut);
    //copyFacesFromMeshOut(resultPos, resultMesh, planeMeshB.positions(), colorAtrMaskBIn);
    //std::cout << "### Vertices: " << resultMesh.vertices().size() << std::endl;
    return pos;
}

SharedOctree generateOctree(SharedPlaneMesh m1, SharedPlaneMesh m2, const AABB& aabb) {
    auto octree = std::make_shared<Octree>(&(*m1), &(*m2), aabb);

    //Set max polygons in Cell
    octree->setOption(Octree::Options::MAX_OBJ_IN_CELL, 22);

    //Fill Octree with faces
    for (auto f : m1->allFaces()) {
        octree->insert_polygon(m1->id(), f);
    }

    for (auto f : m2->allFaces()) {
        octree->insert_polygon(m2->id(), f);
    }
    return octree;
}

void viewMesh(CSGObject& object, bool showOctree = false) {
    object.mesh1->checkAndComputePositions();
    object.mesh2->checkAndComputePositions();
    int sizeV = object.mesh1->positions().count();
    int sizeF = object.mesh1->faces().count();
    auto view = gv::view(object.mesh1->positions(), gv::print_mode, tg::color3::color(0.5));
    //gv::view(gv::lines(mPlaneMeshA->positions()).line_width_world((double)mScaleOctree / 20), tg::color3::color(0.0));

    gv::view(object.mesh2->positions(), tg::color3::color(0.98));
    //gv::view(gv::lines(mPlaneMeshB->positions()).line_width_world((double)mScaleOctree / 20), tg::color3::color(0.0));
    std::vector<AABB> boxes;
    object.octree->insertAABB(boxes);

    std::vector<tg::aabb3> octreeBoxes;
    octreeBoxes.reserve(boxes.size());

    for (auto box : boxes) {
        octreeBoxes.push_back(tg::aabb3(tg::pos3(box.min), tg::pos3(box.max)));
    }

    if (showOctree) {
        gv::view(gv::lines(octreeBoxes).line_width_world(1000000), tg::color3::blue, "gv::lines(pos)");
    }
}


CSGObject computeCSG(const pm::vertex_attribute<tg::pos3>& pos1, const pm::vertex_attribute<tg::pos3> pos2, int scale = 1e6, bool render = false) {
    CSGObject csgObj;
    csgObj.scale = scale;
    const pm::Mesh& m1 = pos1.mesh();
    const pm::Mesh& m2 = pos2.mesh();
    csgObj.mesh1 = std::make_shared<PlaneMesh>(m1, pos1, scale);
    csgObj.mesh2 = std::make_shared<PlaneMesh>(m2, pos2, scale);

    //Todo: dynamic octree cell size. Fits for nearly all meshes in thingi10K
    csgObj.octree = generateOctree(csgObj.mesh1, csgObj.mesh2, AABB(pos_t({ -100, -100, -100 }) * scale, pos_t({ 100, 100, 100 }) * scale));
    //viewMesh(csgObj, true);
    //Cut Mesh
    IntersectionCut iCut;
    {
        ct::scope s("Cut Mesh");
        iCut = csgObj.octree->cutPolygons();
    }

    //Categorization
    int countComponents = 0;
    {
        ct::scope s("Categorization");
        std::shared_ptr<FaceComponentFinder> components1 = std::make_shared<FaceComponentFinder>(*csgObj.mesh1, iCut.getIntersectionEdgesMarkerA());
        std::shared_ptr<FaceComponentFinder> components2 = std::make_shared<FaceComponentFinder>(*csgObj.mesh2, iCut.getIntersectionEdgesMarkerB());
        csgObj.components = std::make_shared<ComponentCategorization>(csgObj.octree, components1, components2, iCut);
        countComponents = components1->countComponents() + components2->countComponents();
    }
    if (render)
        csgObj.components->renderFinalResult(3000, &iCut);
    return csgObj;
}

bool copyFacesFromMeshOut(pm::vertex_attribute<tg::pos3>& pos, pm::Mesh& mesh, const VertexAttribute& copyFrom, pm::face_attribute<bool>& copyFaces, int scale) {
    const auto& oldMesh = copyFrom.mesh();
    pm::Mesh& newMesh = mesh;
    for (auto face : oldMesh.faces()) {
        if (!copyFaces[face])
            continue;

        auto he = face.any_halfedge();
        auto heTmp = he.next();
        auto pFrom = newMesh.vertices().add();
        auto oldFrom = copyFrom[heTmp.vertex_from()];
        pos[pFrom] = tg::pos3((double)oldFrom.x / scale, (double)oldFrom.y / scale, (double)oldFrom.z / scale);
        std::vector<pm::halfedge_handle> halfEdges;

        while (he != heTmp) {
            pm::vertex_handle pTo = newMesh.vertices().add();
            auto oldTo = copyFrom[heTmp.vertex_to()];
            pos[pTo] = tg::pos3((double)oldTo.x / scale, (double)oldTo.y / scale, (double)oldTo.z / scale);
            auto newHe = newMesh.halfedges().add_or_get(pFrom, pTo);
            halfEdges.push_back(newHe);
            pFrom = pTo;
            heTmp = heTmp.next();
        }
        auto pTo = halfEdges[0].vertex_from();
        halfEdges.push_back(newMesh.halfedges().add_or_get(pFrom, pTo));
        newMesh.faces().add(halfEdges);
    }
    //pm::triangulate_naive(newMesh);
    //pm::deduplicate(newMesh, pos);       
    //newMesh.compactify();
    return true;
}

//Flip faces
bool copyFacesFromMeshIn(pm::vertex_attribute<tg::pos3>& pos, pm::Mesh& mesh, const VertexAttribute& copyFrom, pm::face_attribute<bool>& copyFaces, int scale) {
    const auto& oldMesh = copyFrom.mesh();
    pm::Mesh& newMesh = mesh;
    for (auto face : oldMesh.faces()) {
        if (!copyFaces[face])
            continue;

        auto he = face.any_halfedge();
        auto heTmp = he.prev();
        auto pFrom = newMesh.vertices().add();
        auto oldTo = copyFrom[heTmp.vertex_to()];
        pos[pFrom] = tg::pos3((double)oldTo.x / scale, (double)oldTo.y / scale, (double)oldTo.z / scale);
        std::vector<pm::halfedge_handle> halfEdges;

        while (he != heTmp) {
            pm::vertex_handle pTo = newMesh.vertices().add();
            auto oldfrom = copyFrom[heTmp.vertex_from()];
            pos[pTo] = tg::pos3((double)oldfrom.x / scale, (double)oldfrom.y / scale, (double)oldfrom.z / scale);
            auto newHe = newMesh.halfedges().add_or_get(pFrom, pTo);
            halfEdges.push_back(newHe);
            pFrom = pTo;
            heTmp = heTmp.prev();
        }
        auto pTo = halfEdges[0].vertex_from();
        halfEdges.push_back(newMesh.halfedges().add_or_get(pFrom, pTo));
        newMesh.faces().add(halfEdges);
    }
    //pm::triangulate_naive(newMesh);
    //pm::deduplicate(newMesh, pos);
    //newMesh.compactify();
    return true;
}
}