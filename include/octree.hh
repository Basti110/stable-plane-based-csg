#pragma once
#include <typed-geometry/tg.hh>
#include <geometry.hh>
#include <memory>
#include <array>
#include <unordered_map>
//#include <set>
#include "plane_polygon.hh"

class OctreeNode;
class BranchNode;
class LeafNode;
class EmptyNode;
class Octree;
using SharedOctreeNode = std::shared_ptr<OctreeNode>;
using WeakOctreeNode = std::weak_ptr<OctreeNode>;
using SharedLeafNode = std::shared_ptr<LeafNode>;
using SharedBranchNode = std::shared_ptr<BranchNode>;
using SharedEmptyNode = std::shared_ptr<EmptyNode>;
using SharedOctree = std::shared_ptr<Octree>;
using AABB = tg::aabb<3, scalar_t>;

struct PolygonIndex
{
    int mMeshIndex;
    int mPolyIndex;
};

class OctreeNode {
public:
    enum class NodeType {
        EMPTY,
        LEAF,
        BRANCH,
    };

public: 
    //OctreeNode(AABB aabb) : mAABB(aabb) {}
    OctreeNode(const AABB& aabb, Octree* octree) : mAABB(aabb), mOctree(octree) {}
    OctreeNode(const AABB& aabb, Octree* octree, SharedOctreeNode parent) : mParentNode(parent), mAABB(aabb), mOctree(mParentNode->mOctree) {}
    void insertAABB(std::vector<AABB>& insertVec);
    virtual SharedOctreeNode childNode(int idx) = 0;
    //virtual OctreeNode& childNodeRef(int idx) = 0;
    virtual NodeType nodeBase() = 0;
    virtual SharedOctreeNode parent() = 0;
    virtual void markIntersections(pm::face_attribute<tg::color3>& faceColor1, pm::face_attribute<tg::color3>& faceColor2) {}
    bool isInTree();
    bool hasParent();
    bool polygonInAABB(int meshIdx, pm::face_index faceIdx);
    

    Octree* getOctree();

protected: 
    SharedOctreeNode mParentNode;
    AABB mAABB;
    Octree* const mOctree;
};

class LeafNode : public OctreeNode {
public:
    LeafNode(const AABB& aabb, Octree* octree);
    LeafNode(const AABB& aabb, Octree* octree, SharedOctreeNode parent);
    //static SharedLeafNode create() { return std::make_shared<LeafNode>(); };
    //static SharedLeafNode create(SharedOctreeNode parent) { return std::make_shared<LeafNode>(parent); };

    NodeType nodeBase() override { return NodeType::LEAF; }
    SharedOctreeNode childNode(int idx) override;
    //OctreeNode& childNodeRef(int idx) override;
    SharedOctreeNode parent() override;
    
    void insertPolygon(int meshIdx, pm::face_index faceIdx);
    int childCount();
    bool mustSplitIfFull();
    int maxValues() { return mMaxValues; }
    SharedBranchNode split();
    void markIntersections(pm::face_attribute<tg::color3>& faceColor1, pm::face_attribute<tg::color3>& faceColor2);

    void splitAccordingToIntersection() {
        std::vector<pm::face_index> Triangles = mFacesMeshA;
        //std::set<pm::face_index> checkedTriangles;
        for (auto t : Triangles) {
            splitAccordingToIntersection(t);
        }
    }

    void splitAccordingToIntersection(pm::face_index triangle);

    std::tuple<pm::face_index, pm::face_index> split(pm::face_index t1, pm::face_index t2) {
        std::tuple<pm::face_index, pm::face_index> split;
        return split;
    }

private: 
    std::vector<uint32_t> mValueIndices;
    std::vector<pm::face_index> mFacesMeshA;
    std::vector<pm::face_index> mFacesMeshB;
    int mMaxValues = 15;
};

class BranchNode : public OctreeNode, public std::enable_shared_from_this<BranchNode>{
public:
    BranchNode(const AABB& aabb, Octree* octree);
    BranchNode(const AABB& aabb, Octree* octree, SharedOctreeNode parent);
    //static SharedBranchNode create() { return std::make_shared<BranchNode>(); };
    //static SharedBranchNode create(SharedOctreeNode parent) { return std::make_shared<BranchNode>(parent); };

    NodeType nodeBase() override { return NodeType::BRANCH; }
    SharedOctreeNode childNode(int idx) override;
    //OctreeNode& childNodeRef(int idx) override;
    SharedOctreeNode parent() override;

    void pushDown(int meshIdx, pm::face_index faceIdx);
    void initLeafNodes();
    //void initEmptyNodes();

    void markIntersections(pm::face_attribute<tg::color3>& faceColor1, pm::face_attribute<tg::color3>& faceColor2) override {
        for (auto child : mChildNodes)
            child->markIntersections(faceColor1, faceColor2);
    }
    
private: 
    
    std::array<SharedOctreeNode, 8> mChildNodes;
    void insertInLeaf(int child, int meshIdx, pm::face_index faceIdx);
};

class EmptyNode : public OctreeNode {
public:
    EmptyNode(Octree* octree);
    EmptyNode(Octree* octree, SharedOctreeNode parent);
    //static SharedEmptyNode create() { return std::make_shared<EmptyNode>(); };
    //static SharedEmptyNode create(SharedOctreeNode parent) { return std::make_shared<EmptyNode>(parent); };

    NodeType nodeBase() override { return NodeType::EMPTY; }
    SharedOctreeNode childNode(int idx) override;
    //OctreeNode& childNodeRef(int idx) override;
    SharedOctreeNode parent() override;
};


class Octree : public std::enable_shared_from_this<Octree> {

public:
    enum class Options {
        SPLIT_ONE_MESH = 1,
        SPLIT_TWO_MESHES = 2
    };
    Octree(PlaneMesh* a, PlaneMesh* b, const AABB& aabb) {
        mMeshA = a;
        mMeshB = b;
        mRoot = std::make_shared<BranchNode>(aabb, this);
        //mRoot->setOctree(shared_from_this());
        mRoot->initLeafNodes();
    }

    void insert_polygon(int meshIdx, pm::face_handle faceIdx) {
        mRoot->pushDown(meshIdx, faceIdx);
    }

    void insertAABB(std::vector<AABB>& insertVec) {
        mRoot->insertAABB(insertVec);
    }

    void setOption(Options option) {
        if (uint32_t(option) && uint32_t(Options::SPLIT_ONE_MESH))
            mSplitOnlyOneMesh = true;
        else if(uint32_t(option) && uint32_t(Options::SPLIT_TWO_MESHES))
            mSplitOnlyOneMesh = false;
    }

    int markIntersections(pm::face_attribute<tg::color3>& faceColor1, pm::face_attribute<tg::color3>& faceColor2) {
        intersectionCounterTMP = 0;
        mRoot->markIntersections(faceColor1, faceColor2);
        int count = intersectionCounterTMP;
        intersectionCounterTMP = 0;
        return count;
    }
    //PlaneMesh& meshA() { return mMeshA; }
    //PlaneMesh& meshB() { return mMeshB; }

private: 
    bool mSplitOnlyOneMesh = false;
    friend class OctreeNode;
    friend class BranchNode;
    friend class LeafNode;
    SharedBranchNode mRoot;
    PlaneMesh* mMeshA;
    PlaneMesh* mMeshB;
    int intersectionCounterTMP = 0;
};