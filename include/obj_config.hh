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
        mNumObjects(1)
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
        mNumObjects(2)
    {
        mOctreeBox = AABB(pOctreeBox.min * pScaleOctree, pOctreeBox.max * pScaleOctree);
        mTransformation1 = mTranslation1 * mRotation1;
        mTransformation2 = mTranslation2 * mRotation2;
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
        mPlaneMeshA->checkAndComputePositions();
        mPlaneMeshB->checkAndComputePositions();
        int sizeV = mPlaneMeshA->positions().count();
        int sizeF = mPlaneMeshA->faces().count();
        auto view = gv::view(mPlaneMeshA->positions());
        gv::view(gv::lines(mPlaneMeshA->positions()).line_width_world((double)mScaleOctree / 20), tg::color3::color(0.0));
        if (mNumObjects == 2) {
            gv::view(mPlaneMeshB->positions());
            gv::view(gv::lines(mPlaneMeshB->positions()).line_width_world((double)mScaleOctree / 20), tg::color3::color(0.0));
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

    void loadMeshIfNotLoaded() {
        if (mPlaneMeshA && mPlaneMeshB)
            return;

        long long meshInitTime = 0;       
        glow::timing::CpuTimer timer;
        if (!mPlaneMeshA) {
            pm::vertex_attribute<tg::pos3> pos1(*mMeshA);
            
            {
                TRACE("[ObjConfig] PM Load Mesh 1");
                pm::load(mPathObj1, *mMeshA, pos1);
            }          
            transformation(pos1, mTransformation1);
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            mPlaneMeshA = std::make_shared<PlaneMesh>(*mMeshA, pos1, mScale);
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            meshInitTime += std::chrono::duration_cast<std::chrono::microseconds> (end - begin).count();
        }

        if (!mPlaneMeshB && mNumObjects == 2) {
            pm::vertex_attribute<tg::pos3> pos2(*mMeshB);
            {
                TRACE("[ObjConfig] PM Load Mesh 2");
                pm::load(mPathObj2, *mMeshB, pos2);
            }           
            transformation(pos2, mTransformation2);
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            mPlaneMeshB = std::make_shared<PlaneMesh>(*mMeshB, pos2, mScale);
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            meshInitTime += std::chrono::duration_cast<std::chrono::microseconds> (end - begin).count();
            //TG_ASSERT(mPlaneMeshB->allFacesAreValid());
        }      
        mInitMeshTime = meshInitTime / (double)1000;
        //std::cout << "Load Meshes in " << timer.elapsedMilliseconds() << "ms" << std::endl;
        //std::cout << " -- PM load in " << pmLoadTime / 1000.0 << "ms" << std::endl;
    }



private:
    int mNumObjects = 1;
    double mInitMeshTime = 0;
    std::shared_ptr<pm::Mesh> mMeshA = std::make_shared<pm::Mesh>();
    std::shared_ptr<pm::Mesh> mMeshB = std::make_shared<pm::Mesh>();
    SharedPlaneMesh mPlaneMeshA;
    SharedPlaneMesh mPlaneMeshB;
    SharedOctree mOctree;
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
                ObjConfig(1e6, 1e6, AABB({ -60, -60, -40 }, { 60, 60, 80 }),
                "../data/mesh/fox.obj", tg::translation(tg::vec{ 0.f, -50.f, 15.f }), tg::rotation_x(tg::angle::from_degree(-90)),
                "../data/mesh/fox.obj", tg::mat4::identity, tg::mat4::identity) },
            { "bunny_mesh_1",
                ObjConfig(1e7, 1e7, AABB({ -60, -60, -50 }, { 60, 60, 70 }),
                "../data/mesh/bunny.obj", tg::translation(tg::vec{ -.0f, -.1f, .04f }), tg::rotation_y(tg::angle::from_degree(-90))) },
            { "bunny_mesh_2",
                ObjConfig(1e6, 1e6, AABB({ -1, -1, -1 }, { 1, 1, 1 }),
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
                "../data/mesh/raycast1.obj", tg::translation(tg::vec{-.5f, -.5f, -.5f }), tg::mat4::identity,
                "../data/mesh/raycast2.obj", tg::translation(tg::vec{-.5f, -.5f, -.5f }), tg::mat4::identity) },
            { "complex_1",
                ObjConfig(1e7, 1e7, AABB({ -60, -60, -50 }, { 60, 60, 70 }),
                "../data/mesh/bunny.obj", tg::translation(tg::vec{-.5f, -.5f, -.5f }), tg::mat4::identity,
                "../data/mesh/soma_gyroid_Z.obj", tg::translation(tg::vec{-25.0f, -.5f, 10.0f }), tg::scaling(1.3f, 1.3f, 1.3f))},
            { "Armadillo", //
                ObjConfig(1e7, 1e7, AABB({ -60, -60, -50 }, { 60, 60, 70 }),
                "../data/mesh/Armadillo.obj", tg::mat4::identity, tg::mat4::identity,
                "../data/mesh/Armadillo.obj", tg::translation(tg::vec{ -20.0f, 0.0f, 20.0f }), tg::rotation_y(tg::angle::from_degree(-90)))},
            { "Buddha", //
                ObjConfig(1e8, 1e7, AABB({ -60, -60, -50 }, { 60, 60, 70 }),
                "../data/mesh/Buddha.obj", tg::translation(tg::vec{ 0.0f, -10.5f, 0.0f }), tg::mat4::identity,
                "../data/mesh/Buddha.obj", tg::translation(tg::vec{ -1.0f, -10.0f, 2.0f }), tg::rotation_y(tg::angle::from_degree(-90)))},
        };
    }
};
