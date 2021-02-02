#pragma once
#include <string>
#include <typed-geometry/tg.hh>
#include <algorithm>
#include "plane_polygon.hh"
#include "octree.hh"
#include <unordered_map>
#include <unordered_set>
#include <glow-extras/viewer/view.hh>
#include <glow-extras/viewer/experimental.hh>
#include <glow-extras/timing/CpuTimer.hh>
#include <polymesh/algorithms/deduplicate.hh>
#include <clean-core/array.hh>
#include <babel-serializer/file.hh>
#include <babel-serializer/geometry/stl.hh>

//data class
class ObjConfig {
public:
    ObjConfig() {}; // = delete;
    ObjConfig(scalar_t pScale, scalar_t pScaleOctree, AABB pOctreeBox, std::string pPathObj1, tg::mat4 pTranslation1, tg::mat4 pRotation1) : 
        mScale(pScale),
        mScaleOctree(pScaleOctree),
        mPathObj1(pPathObj1),
        mTranslation1(pTranslation1),
        mRotation1(pRotation1),
        mNumObjects(1),
        mPos1(*mMeshA),
        mPos2(*mMeshB)
    {
        mOctreeBox = AABB(pOctreeBox.min * pScaleOctree, pOctreeBox.max * pScaleOctree);
        mTransformation1 = mTranslation1 * mRotation1;
    }

    ObjConfig(scalar_t pScale, scalar_t pScaleOctree, AABB pOctreeBox,
        std::string pPathObj1, tg::mat4 pTranslation1, tg::mat4 pRotation1,
        std::string pPathObj2, tg::mat4 pTranslation2, tg::mat4 pRotation2) :
        mScale(pScale),
        mScaleOctree(pScaleOctree),
        mPathObj1(pPathObj1),
        mTranslation1(pTranslation1),
        mRotation1(pRotation1),
        mPathObj2(pPathObj2),
        mTranslation2(pTranslation2),
        mRotation2(pRotation2),
        mNumObjects(2),
        mPos1(*mMeshA),
        mPos2(*mMeshB)
    {
        mOctreeBox = AABB(pOctreeBox.min * pScaleOctree, pOctreeBox.max * pScaleOctree);
        mTransformation1 = mTranslation1 * mRotation1;
        mTransformation2 = mTranslation2 * mRotation2;
    }

    ObjConfig(scalar_t pScaleOctree, AABB pOctreeBox, const pm::vertex_attribute<tg::dpos3> pos1, const pm::vertex_attribute<tg::dpos3> pos2) :
        mScaleOctree(pScaleOctree),
        mNumObjects(2),
        mPos1(*mMeshA),
        mPos2(*mMeshB)
    {
        mOctreeBox = AABB(pOctreeBox.min * pScaleOctree, pOctreeBox.max * pScaleOctree);
        const auto& meshA = pos1.mesh();
        const auto& meshB = pos2.mesh();
        mPlaneMeshA = std::make_shared<PlaneMesh>(meshA, pos1, 1);
        mPlaneMeshB = std::make_shared<PlaneMesh>(meshB, pos2, 1);
    }

    void transformation(pm::vertex_attribute<tg::pos3>& pos, tg::mat4& mat) {
        pos.apply([&mat](tg::pos3& pos) {
            auto pos4 = tg::vec4(pos, 1);
            pos4 = mat * pos4;
            pos = tg::pos3(pos4);
            }
        );
    }

    SharedOctree getOctree() {
        fillOctreeIfNotFilled();
        return mOctree;
    }

    std::vector<tg::aabb3> getOctreeBoxes() {
        fillOctreeIfNotFilled();

        std::vector<AABB> boxes;       
        mOctree->insertAABB(boxes);

        std::vector<tg::aabb3> returnBoxes;
        returnBoxes.reserve(boxes.size());

        for (auto box : boxes) {
            returnBoxes.push_back(tg::aabb3(tg::pos3(box.min), tg::pos3(box.max)));
        }
        return returnBoxes;
    }

    SharedPlaneMesh getMeshA() {
        loadMeshIfNotLoaded();
        return mPlaneMeshA;
    }

    SharedPlaneMesh getMeshB() {
        TG_ASSERT(mNumObjects == 2);
        loadMeshIfNotLoaded();
        return mPlaneMeshB;
    }

    void viewMesh(bool showOctree = false) {
        loadMeshIfNotLoaded();
        if (showOctree)
            fillOctreeIfNotFilled();
        mPlaneMeshA->checkAndComputePositions();
        mPlaneMeshB->checkAndComputePositions();
        int sizeV = mPlaneMeshA->positions().count();
        int sizeF = mPlaneMeshA->faces().count();
        auto view = gv::view(mPlaneMeshA->positions(), gv::print_mode, tg::color3::color(0.5));
        //gv::view(gv::lines(mPlaneMeshA->positions()).line_width_world((double)mScaleOctree / 20), tg::color3::color(0.0));
        if (mNumObjects == 2) {
            gv::view(mPlaneMeshB->positions(), tg::color3::color(0.98));
            //gv::view(gv::lines(mPlaneMeshB->positions()).line_width_world((double)mScaleOctree / 20), tg::color3::color(0.0));
        }
        if (showOctree) {
            auto boxes = getOctreeBoxes();
            gv::view(gv::lines(boxes).line_width_world(mScaleOctree / 20), tg::color3::blue, "gv::lines(pos)");
        }
    }

    static int meshHasBoundaries(SharedPlaneMesh mesh) {
        int invalidCount = 0;
        for (auto& he : mesh->mesh().all_halfedges()) {
            if (he.is_invalid()) {
                invalidCount++;
                continue;
            }

            if (he.face().is_invalid()) {
                invalidCount++;
                continue;
            }
        }
        return invalidCount;
    }

    static void meshHasBoundaries(SharedPlaneMesh mesh, std::vector<pm::halfedge_handle>& edgeVector) {
        for (auto& he : mesh->mesh().all_halfedges()) {
            if (he.is_invalid()) {
                edgeVector.push_back(he);
                continue;
            }

            if (he.face().is_invalid()) {
                edgeVector.push_back(he);
                continue;
            }
        }
    }

    void viewMeshWithBoundaries(SharedPlaneMesh mesh) {
        std::vector<pm::halfedge_handle> faceVector;
        meshHasBoundaries(mesh, faceVector);

        pm::halfedge_attribute<bool> boundaryEdges = mesh->mesh().halfedges().map([&](pm::halfedge_handle& he) {
            return std::find(faceVector.begin(), faceVector.end(), he) != faceVector.end();
        }, false);

        auto const boundaryLines = gv::make_renderable(gv::lines(mesh->positions()).line_width_world(300000));
        auto view = gv::view(mesh->positions());
        gv::view(gv::lines(mesh->positions()).line_width_world(100000));
        gv::view(boundaryLines, gv::masked(boundaryEdges), tg::color3::color(0.0));
    }

    double initMeshTime() {
        return mInitMeshTime;
    }

    void setMeshRepairBefore(bool v) {
        mRepairBefore = v;
    }

    void setMaxObjInCell(int v) {
        mMaxObjInCell = v;
    }

    bool loadMesh() {
        return loadMeshIfNotLoaded();
    }

    bool meshIsValid() {
        if (!loadMeshIfNotLoaded())
            return false;

        for (auto h : mMeshA->halfedges()) {
            if (h.is_invalid())
                return false;
            if (h.face().is_invalid())
                return false;
        }

        for (auto h : mMeshB->halfedges()) {
            if (h.is_invalid())
                return false;
            if (h.face().is_invalid())
                return false;
        }
        return true;
    }

    void setMaxCellSize(int v) {
        mMaxCellSize = v;
    }

    void setMoveToCenter(bool v) {
        mMoveToCenter = v;
    }

    void reset() {
        mPlaneMeshA = std::make_shared<PlaneMesh>(*mMeshA, mPos1, mScale);
        mPlaneMeshB = std::make_shared<PlaneMesh>(*mMeshB, mPos2, mScale);
        mOctree = SharedOctree(nullptr);
        //fillOctreeIfNotFilled();
    }

    void adjustScale() {
        auto posRef = mPos1.first();
        auto scaleO = false;
        auto scaleM = true;
        for (auto pos : mPos1) {
            if (std::abs(pos.x) > 100 || std::abs(pos.y) > 100 || std::abs(pos.z) > 100)
                scaleO = true;

            if (scaleM && tg::distance(posRef, pos) > 1)
                scaleM = false;
        }
        if (scaleM)
            mScale *= 10;
        if (scaleO)
            mScaleOctree *= 10;
        std::cout << "*10" << std::endl;
    }


    static void loadBabel(std::string filepath, std::shared_ptr<pm::Mesh> mesh, pm::vertex_attribute<tg::pos3>& pos) {
        //auto c_name = cc::string_view(filepath.c_str());
        //auto t = babel::file::read_all_bytes(c_name);
        auto res = babel::stl::read(babel::file::read_all_bytes(filepath));
        mesh->clear();
        pos = mesh->vertices().make_attribute<tg::pos3>();
        for (auto const t : res.triangles)
        {
            auto const v0 = mesh->vertices().add();
            auto const v1 = mesh->vertices().add();
            auto const v2 = mesh->vertices().add();
            pos[v0] = t.v0;
            pos[v1] = t.v1;
            pos[v2] = t.v2;
            mesh->faces().add(v0, v1, v2);
        }
        //gv::view(pos);
    }

    void moveToCenter(const pm::vertex_attribute<tg::pos3>& pos, tg::mat4& transformation) {
        auto aabb = tg::aabb_of(pos);
        auto distance = aabb.max - aabb.min;
        auto halfDistance = distance / 2;
        auto center = aabb.min + halfDistance;
        auto trans = tg::translation(-center);
        transformation = transformation * trans;
    }
       
private:
    void fillOctreeIfNotFilled() {  
        loadMeshIfNotLoaded();
        if (!mOctree) {
            //glow::timing::CpuTimer timer;
            TRACE("[ObjConfig] Build Octree");
            if (mNumObjects == 2) {
                mOctree = std::make_shared<Octree>(&(*mPlaneMeshA), &(*mPlaneMeshB), mOctreeBox);
            }
            else {
                mOctree = std::make_shared<Octree>(&(*mPlaneMeshA), mOctreeBox);
                mOctree->setOption(Octree::Options::SPLIT_ONE_MESH);
            }               
            mOctree->setOption(Octree::Options::MAX_OBJ_IN_CELL, mMaxObjInCell);
            for (auto f : mPlaneMeshA->allFaces()) {
                mOctree->insert_polygon(mPlaneMeshA->id(), f);
            }

            if (mNumObjects == 2) {
                for (auto f : mPlaneMeshB->allFaces()) {
                    mOctree->insert_polygon(mPlaneMeshB->id(), f);
                }
            }
            //std::cout << "Created Octree in " << timer.elapsedMilliseconds() << "ms" << std::endl;
            //std::cout << "  -- Smallest Cell Lenght: " << (double)mOctree->smallestCellLen() << std::endl;
        }       
    }

    bool loadMeshIfNotLoaded() {
        if (mPlaneMeshA && mPlaneMeshB)
            return true;

        long long meshInitTime = 0;       
        glow::timing::CpuTimer timer;
        if (!mPlaneMeshA) {
            //pm::vertex_attribute<tg::pos3> pos1(*mMeshA);
            {
                TRACE("[ObjConfig] PM Load Mesh 1");
                //pm::load(mPathObj1, *mMeshA, mPos1);
                if (!pm::load(mPathObj1, *mMeshA, mPos1))
                    //return false;
                    loadBabel(mPathObj1, mMeshA, mPos1);
            }   
            if (mRepairBefore) {
                pm::deduplicate(*mMeshA, mPos1);
                mMeshA->compactify();
                adjustScale();
            }
            if (mMoveToCenter) 
                moveToCenter(mPos1, mTransformation1);           
            transformation(mPos1, mTransformation1);
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            mPlaneMeshA = std::make_shared<PlaneMesh>(*mMeshA, mPos1, mScale);
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            meshInitTime += std::chrono::duration_cast<std::chrono::microseconds> (end - begin).count();
        }

        if (!mPlaneMeshB && mNumObjects == 2) {
            //pm::vertex_attribute<tg::pos3> pos2(*mMeshB);
            {
                TRACE("[ObjConfig] PM Load Mesh 2");
                //pm::load(mPathObj2, *mMeshB, mPos2);
                if (!pm::load(mPathObj2, *mMeshB, mPos2))
                    loadBabel(mPathObj2, mMeshB, mPos2);
            }   
            if (mRepairBefore) {
                pm::deduplicate(*mMeshB, mPos2);
                mMeshB->compactify();
            }
            if (mMoveToCenter)
                moveToCenter(mPos2, mTransformation2);
            transformation(mPos2, mTransformation2);
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            mPlaneMeshB = std::make_shared<PlaneMesh>(*mMeshB, mPos2, mScale);
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            meshInitTime += std::chrono::duration_cast<std::chrono::microseconds> (end - begin).count();
            //TG_ASSERT(mPlaneMeshB->allFacesAreValid());
        }      
        mInitMeshTime = meshInitTime / (double)1000;
        return true;
        //std::cout << "Load Meshes in " << timer.elapsedMilliseconds() << "ms" << std::endl;
        //std::cout << " -- PM load in " << pmLoadTime / 1000.0 << "ms" << std::endl;
    }




private:
    bool mRepairBefore = false;
    bool mMoveToCenter = false;
    int mNumObjects = 1;
    double mInitMeshTime = 0;
    int mMaxCellSize = 15;
    std::shared_ptr<pm::Mesh> mMeshA = std::make_shared<pm::Mesh>();
    std::shared_ptr<pm::Mesh> mMeshB = std::make_shared<pm::Mesh>();
    pm::vertex_attribute<tg::pos3> mPos1;
    pm::vertex_attribute<tg::pos3> mPos2;
    SharedPlaneMesh mPlaneMeshA;
    SharedPlaneMesh mPlaneMeshB;
    SharedOctree mOctree;
    int mMaxObjInCell = 22;
    scalar_t mScale = 1;
    scalar_t mScaleOctree = 1;    

    AABB mOctreeBox;

    std::string mPathObj1;
    std::string mPathObj2;

    tg::mat4 mTranslation1 = tg::mat4::identity;
    tg::mat4 mTranslation2 = tg::mat4::identity;

    tg::mat4 mRotation1 = tg::mat4::identity;
    tg::mat4 mRotation2 = tg::mat4::identity;

    tg::mat4 mTransformation1 = tg::mat4::identity;
    tg::mat4 mTransformation2 = tg::mat4::identity;
};

class ObjCollection {
public:
    static inline tg::mat4 ident = tg::mat4::identity;
    static inline std::unordered_map<std::string, ObjConfig> map = {};
    static inline void init() {
        map = {
            { "fox_mesh_1",
                ObjConfig(1e6, 1e6, AABB({ -60, -60, -40 }, { 60, 60, 80 }),
                "../data/mesh/fox.obj", tg::translation(tg::vec{ 0.f, -50.f, 15.f }), tg::rotation_x(tg::angle::from_degree(-90))) },
            { "fox_mesh_2",
                ObjConfig(1e6, 1e6, AABB({ -60, -40, -40 }, { 60, 80, 80 }),
                //"../data/mesh/fox.obj", tg::translation(tg::vec{ 0.f, -50.f, 15.f }), tg::rotation_x(tg::angle::from_degree(-90)),
                "../data/mesh/fox.obj", tg::mat4::identity, tg::rotation_x(tg::angle::from_degree(-90)),
                "../data/mesh/fox.obj", tg::translation(tg::vec{ -5.0f, 10.f, 5.f }), tg::rotation_x(tg::angle::from_degree(-90))) },
            { "bunny_mesh_1",
                ObjConfig(1e7, 1e7, AABB({ -60, -60, -50 }, { 60, 60, 70 }),
                "../data/mesh/bunny.obj", tg::translation(tg::vec{ .0f, .5f, .5f }), tg::rotation_y(tg::angle::from_degree(-90))) },
            { "bunny_mesh_2",
                ObjConfig(1e6, 1e6, AABB({ -60, -60, -50 }, { 60, 60, 70 }),
                "../data/mesh/bunny.obj", tg::translation(tg::vec{ -.0f, -.1f, .04f }), tg::rotation_y(tg::angle::from_degree(-90)),
                "../data/mesh/bunny.obj", tg::translation(tg::vec{ 0.0f, -.08f, 0.0f }), tg::mat4::identity) },
            { "gyroid_mesh_1",
                ObjConfig(1e6, 1e6, AABB({ -60, -60, -50 }, { 60, 60, 70 }),
                "../data/mesh/soma_gyroid_Z_2.obj", tg::translation(tg::vec{ -.0f, -.1f, .04f }), tg::rotation_y(tg::angle::from_degree(-90))) },
            { "octree_easy",
                ObjConfig(1e8, 1e8, AABB({ -2, -2, -2 }, { 2, 2, 2 }),
                "../data/mesh/triangle.obj", tg::translation(tg::vec{-.5f, -.5f, -.5f }), tg::mat4::identity,
                "../data/mesh/cube2.obj", tg::translation(tg::vec{-.5f, -.5f, -.5f }), tg::mat4::identity) },
            { "octree_hard",
                ObjConfig(3e7, 1e8, AABB({ -2, -2, -2 }, { 2, 2, 2 }),
                "../data/mesh/triangle.obj", tg::translation(tg::vec{-.5f, -.5f, -.5f }), tg::mat4::identity,
                "../data/mesh/cube3.obj", tg::translation(tg::vec{-.5f, -.5f, -.5f }), tg::mat4::identity) },
            { "raycast",
                ObjConfig(2e7, 1e8, AABB({ -2, -2, -2 }, { 2, 2, 2 }),
                "../data/mesh/raycast4.obj", tg::translation(tg::vec{-.5f, -.5f, -.5f }), tg::mat4::identity,
                "../data/mesh/raycast2.obj", tg::translation(tg::vec{-.5f, -.5f, -.5f }), tg::mat4::identity) },
            { "complex_1",
                ObjConfig(1e7, 1e7, AABB({ -60, -60, -50 }, { 60, 60, 70 }),
                "../data/mesh/bunny.obj", tg::translation(tg::vec{-.5f, -.5f, -.5f }), tg::mat4::identity,
                "../data/mesh/soma_gyroid_Z.obj", tg::translation(tg::vec{-25.0f, -.5f, 10.0f }), tg::scaling(1.3f, 1.3f, 1.3f))},
            { "Armadillo", //
            ObjConfig(1e7, 2e7, AABB({ -60, -60, -50 }, { 60, 60, 70 }),
            //ObjConfig(1e6, 1e7, AABB({ -15, -15, -10 }, { 15, 15, 20 }),
                "../data/mesh/Armadillo_2.obj", tg::mat4::identity, tg::mat4::identity,
                "../data/mesh/Armadillo_2.obj", tg::translation(tg::vec{ -20.0f, 0.0f, 20.0f }), tg::rotation_y(tg::angle::from_degree(-90)))},
                //"../data/mesh/Armadillo.obj", tg::translation(tg::vec{ -0.556207f, -0.512985f, 6.94738f }), tg::mat4::identity)},
                //"../data/mesh/Armadillo_2.obj", tg::translation(tg::vec{ -0.296180f, 6.20101f, 5.74443f }), tg::mat4::identity) },
                //-556207:-512985:6.94738e+06:1 //-296180:6.20101e+06:5.74443e+06:1
            { "Buddha", //
                ObjConfig(1e8, 1e7, AABB({ -60, -60, -50 }, { 60, 60, 70 }),
                "../data/mesh/Buddha.obj", tg::translation(tg::vec{ 0.0f, -10.5f, 0.0f }), tg::mat4::identity,
                "../data/mesh/Buddha.obj", tg::translation(tg::vec{ -1.0f, -10.0f, 2.0f }), tg::rotation_y(tg::angle::from_degree(-90))) },
            { "cubes", 
                ObjConfig(1e7, 1e7, AABB({ -15, -15, -10 }, { 15, 15, 20 }),
                "../data/mesh/cubes1.obj", tg::mat4::identity, tg::mat4::identity, //-264829:-5.87149e+06:-4.86755e+06:1
                //"../data/mesh/cubes2.obj", tg::translation(tg::vec{ .410618f, -1.10753f, -.371363f }), tg::mat4::identity)},
                //"../data/mesh/cubes2.obj", tg::translation(tg::vec{ .81621f, -1.08294f, .545297f }), tg::mat4::identity)},
                //"../data/mesh/cubes2.obj", tg::translation(tg::vec{ 0.89226f,-1.08672f, 0.582565f}), tg::mat4::identity) },
                //"../data/mesh/cubes2.obj", tg::translation(tg::vec{ -.0631121f, .0887926f, .0665039f }), tg::mat4::identity) },
                "../data/mesh/cubes2.obj", tg::translation(tg::vec{ -0.0264829f,-0.587149f, -0.486755f}), tg::mat4::identity) },
            { "Buddha_90", //
                ObjConfig(1e8, 1e7, AABB({ -60, -60, -50 }, { 60, 60, 70 }),
                "../data/mesh/Buddha.obj", tg::translation(tg::vec{ 5.0f, -10.5f, 0.0f }), tg::mat4::identity,
                "../data/mesh/Buddha.obj", tg::translation(tg::vec{ 20.0f, 5.0f, 0.0f }), tg::rotation_z(tg::angle::from_degree(90))) },
            { "Buddha_10", //
                ObjConfig(1e8, 1e7, AABB({ -60, -60, -50 }, { 60, 60, 70 }),
                "../data/mesh/Buddha.obj", tg::translation(tg::vec{ 5.0f, -10.5f, 0.0f }), tg::mat4::identity,
                "../data/mesh/Buddha.obj", tg::translation(tg::vec{ 8.0f, -10.0f, 0.0f }), tg::rotation_z(tg::angle::from_degree(10))) },
            { "Lucy_10", //
                ObjConfig(1e6, 1e7, AABB({ -60, -60, -50 }, { 60, 60, 70 }),
                "../data/mesh/Lucy.obj", tg::translation(tg::vec{ 100.0f, 300.0f, -100.0f }), tg::mat4::identity,
                "../data/mesh/Lucy.obj", tg::translation(tg::vec{ 100.0f, 300.0f, -100.0f }), tg::rotation_z(tg::angle::from_degree(10))) },
            { "Lucy_90", //
                ObjConfig(1e6, 1e7, AABB({ -60, -60, -50 }, { 60, 60, 70 }),
                "../data/mesh/Lucy.obj", tg::translation(tg::vec{ 500.0f, 300.0f, -100.0f }), tg::mat4::identity,
                "../data/mesh/Lucy.obj", tg::translation(tg::vec{ 400.0f, 300.0f, -100.0f }), tg::rotation_z(tg::angle::from_degree(90))) },
            { "Lucy_1", //
                ObjConfig(1e6, 1e7, AABB({ -60, -60, -50 }, { 60, 60, 70 }),
                "../data/mesh/Lucy.obj", tg::translation(tg::vec{ 100.0f, 300.0f, -100.0f }), tg::mat4::identity,
                "../data/mesh/Lucy.obj", tg::translation(tg::vec{ 100.0f, 300.0f, -100.0f }), tg::rotation_z(tg::angle::from_degree(1))) },
            { "co-planar-cubes1", //
                ObjConfig(1e6, 1e6, AABB({ -5, -5, -5 }, { 5, 5, 5 }),
                "../data/mesh/co-planar-cube.obj", tg::mat4::identity, tg::mat4::identity,
                "../data/mesh/unit_cube.obj", tg::mat4::identity, tg::mat4::identity )},
            { "co-planar-cubes2", //
                ObjConfig(1e6, 1e6, AABB({ -5, -5, -5 }, { 5, 5, 5 }),
                "../data/mesh/co-planar-cube.obj", tg::mat4::identity, tg::mat4::identity,
                "../data/mesh/unit_cube.obj", tg::translation(tg::vec{ 0.0f, 1.0f, 0.0f }), tg::mat4::identity)},
        };
    }
};
