#pragma once
#include <string>
#include <typed-geometry/tg.hh>
#include "plane_polygon.hh"
#include "octree.hh"
#include <unordered_map>

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
       

private:
    void fillOctreeIfNotFilled() {  
        loadMeshIfNotLoaded();
        if (!mOctree) {
            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            mOctree = std::make_shared<Octree>(&(*mPlaneMeshA), &(*mPlaneMeshB), mOctreeBox);
            if (mNumObjects == 1)
                mOctree->setOption(Octree::Options::SPLIT_ONE_MESH);
            
            for (auto f : mPlaneMeshA->allFaces()) {
                mOctree->insert_polygon(mPlaneMeshA->id(), f);
            }

            if (mNumObjects == 2) {
                for (auto f : mPlaneMeshB->allFaces()) {
                    mOctree->insert_polygon(mPlaneMeshB->id(), f);
                }
            }

            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            auto seconds = std::chrono::duration_cast<std::chrono::milliseconds> (end - begin).count();
            std::cout << "Created Octree in " << seconds << "ms" << std::endl;
        }       
    }

    void loadMeshIfNotLoaded() {
        if (!mPlaneMeshA) {
            pm::vertex_attribute<tg::pos3> pos1(*mMeshA);
            pm::load(mPathObj1, *mMeshA, pos1);
            transformation(pos1, mTransformation1);
            mPlaneMeshA = std::make_shared<PlaneMesh>(*mMeshA, pos1, mScale);
        }

        if (!mPlaneMeshB && mNumObjects == 2) {
            pm::vertex_attribute<tg::pos3> pos2(*mMeshB);
            pm::load(mPathObj2, *mMeshB, pos2);
            transformation(pos2, mTransformation2);
            mPlaneMeshB = std::make_shared<PlaneMesh>(*mMeshB, pos2, mScale);
            TG_ASSERT(mPlaneMeshB->allFacesAreValid());
        }
    }

private:
    int mNumObjects = 1;
    std::shared_ptr<pm::Mesh> mMeshA = std::make_shared<pm::Mesh>();
    std::shared_ptr<pm::Mesh> mMeshB = std::make_shared<pm::Mesh>();
    SharedPlaneMesh mPlaneMeshA;
    SharedPlaneMesh mPlaneMeshB;
    SharedOctree mOctree;

    scalar_t mScale;
    scalar_t mScaleOctree;
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
            ObjConfig(1e9, 1e8, AABB({ -1, -1, -1 }, { 1, 1, 1 }),
            "../data/mesh/bun_zipper.obj", tg::translation(tg::vec{ -.0f, -.1f, .04f }), tg::rotation_y(tg::angle::from_degree(-90))) },
            { "bunny_mesh_2",
            ObjConfig(1e9, 1e8, AABB({ -1, -1, -1 }, { 1, 1, 1 }),
            "../data/mesh/bun_zipper.obj", tg::translation(tg::vec{ -.0f, -.1f, .04f }), tg::rotation_y(tg::angle::from_degree(-90)),
            "../data/mesh/bun_zipper.obj", tg::translation(tg::vec{ 0.0f, -.08f, 0.0f }), tg::mat4::identity) },
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
        };
    }
};
