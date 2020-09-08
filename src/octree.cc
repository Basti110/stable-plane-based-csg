#include "..\include\octree.hh"
//std
#include <functional>

//Glow
#include <glow/common/log.hh>

//custom
#include <utils.hh>
#include <aabb.hh>


//#############################################################################
//#                             OctreeNode                                    #
//#############################################################################

void OctreeNode::insertAABB(std::vector<AABB>& insertVec)
{
    insertVec.push_back(mAABB);
    if (nodeBase() == NodeType::BRANCH) {
        for (int i = 0; i < 8; ++i)
            childNode(i)->insertAABB(insertVec);
    }
    return; 
}

bool OctreeNode::isInTree()
{
    if (mOctree)
        return true;
    return false;
}

bool OctreeNode::hasParent()
{
    if (!mParentNode.expired())
        return true;
    return false;
}

//TODO
bool OctreeNode::polygonInAABB(int meshIdx, pm::face_index faceIdx)
{
    TG_ASSERT(mOctree);
    PlaneMesh* mesh = meshIdx == mOctree->mMeshA->id() ? mOctree->mMeshA : mOctree->mMeshB;
    auto intersection = ob::intersection_type<geometry128>(*mesh, faceIdx.of(mesh->mesh()), mAABB);
    if (intersection != ob::intersection_result::non_intersecting)
        return true;
    return false;
}

void OctreeNode::setParent(SharedBranchNode node)
{
    mParentNode = node;
    mIsValid = true;
}


//#############################################################################
//#                             LeafNode                                      #
//#############################################################################


LeafNode::LeafNode(const AABB& aabb, Octree* octree) : OctreeNode(aabb, octree)
{
}

LeafNode::LeafNode(const AABB& aabb, SharedBranchNode parent) : OctreeNode(aabb, parent)
{
}

SharedOctreeNode LeafNode::childNode(int idx)
{
    return std::dynamic_pointer_cast<OctreeNode>(std::make_shared<EmptyNode>(mOctree));
}

SharedOctreeNode LeafNode::parent()
{
    if (!mParentNode.expired())
        return std::dynamic_pointer_cast<OctreeNode>(mParentNode.lock());
    return std::dynamic_pointer_cast<OctreeNode>(std::make_shared<EmptyNode>(mOctree));
}

void LeafNode::insertPolygon(int meshIdx, pm::face_index faceIdx)
{
    pm::face_index index = faceIdx;
    if (mOctree->mMeshA->id() == meshIdx) {
        mFacesMeshA.push_back(index);
        mOctree->mFaceMeshAToNode[index].push_back(shared_from_this());
    }
    else if (mOctree->mMeshB->id() == meshIdx) {
        mFacesMeshB.push_back(index);
        mOctree->mFaceMeshBToNode[index].push_back(shared_from_this());
    }
    else {
        LOG_ERROR() << meshIdx << " is no valid Mesh ID";
    }
}

int LeafNode::childCount()
{
    return mFacesMeshA.size() + mFacesMeshB.size();
}

bool LeafNode::mustSplitIfFull()
{
    if(mOctree->mSplitOnlyOneMesh)
        return true;

    return mFacesMeshA.size() > 0 && mFacesMeshB.size() > 0;
}

SharedBranchNode LeafNode::split()
{
    TG_ASSERT(mOctree);
    auto branchNode = std::make_shared<BranchNode>(mAABB, mOctree);
    branchNode->initLeafNodes();
    for (auto& face : mFacesMeshA)
        branchNode->pushDown(mOctree->mMeshA->id(), face);
    for (auto& face : mFacesMeshB)
        branchNode->pushDown(mOctree->mMeshB->id(), face);
    return branchNode;
}

void LeafNode::cutPolygons(IntersectionCut& lookup) {
    if (mFacesMeshA.size() <= 0 || mFacesMeshB.size() <= 0)
        return;
    
    lookup.cutPolygons(mFacesMeshA, mFacesMeshB);
}

void LeafNode::markIntersections(pm::face_attribute<tg::color3>& faceColor1, pm::face_attribute<tg::color3>& faceColor2) {
    if (mFacesMeshA.size() <= 0 || mFacesMeshB.size() <= 0)
        return;

    for (pm::face_index face1Index : mFacesMeshA) {
        auto face1 = face1Index.of(mOctree->mMeshA->mesh());
        for (pm::face_index face2Index : mFacesMeshB) {
            auto face2 = face2Index.of(mOctree->mMeshB->mesh());
            mOctree->intersectionCounterTMP++;
            if (IsectOb(*(mOctree->mMeshA), *(mOctree->mMeshB)).intersect<geometry128>(face1, face2)->intersectionState != TrianlgeIntersection::IntersectionState::NON_INTERSECTING) {
                faceColor1[face1] = tg::color3::green;
                faceColor2[face2] = tg::color3::red;             
            }
        }
    }
}

NearestFace LeafNode::getNearestFace(tg::vec3 ray, pos_t origin) {
    NearestFace currentNearest = { -1, pm::face_index(), -1 };
    
    for (pm::face_index& face : mFacesMeshA) {
        PlaneMesh* planeMesh = mOctree->mMeshA;
        auto distance = planeMesh->rayHitPolygon(face, ray, origin);
        TG_ASSERT(distance >= -1);

        if (distance == -1)
            continue;

        if (currentNearest.distance == -1 || distance < currentNearest.distance)
            currentNearest = { planeMesh->id(), face, distance };
    }

    for (pm::face_index& face : mFacesMeshB) {
        PlaneMesh* planeMesh = mOctree->mMeshB;
        auto distance = planeMesh->rayHitPolygon(face, ray, origin);

        if (distance == -1)
            continue;

        if (currentNearest.distance == -1 || distance < currentNearest.distance)
            currentNearest = { planeMesh->id(), face, distance };
    }
    return currentNearest;
}

/*void LeafNode::splitAccordingToIntersection(pm::face_index triangle)
{
    for (pm::face_index t2 : mFacesMeshB) {
        pm::Mesh& meshA = mOctree->mMeshA->mesh();
        pm::Mesh& meshB = mOctree->mMeshB->mesh();
        if (ob::intersect<geometry128>(*(mOctree->mMeshA), triangle, *(mOctree->mMeshB), t2)) {
            auto splits = split(triangle, t2);
            splitAccordingToIntersection(std::get<0>(splits));
            splitAccordingToIntersection(std::get<1>(splits));
            break;
        }
    }
}*/


//#############################################################################
//#                             BranchNode                                    #
//#############################################################################


BranchNode::BranchNode(const AABB& aabb, Octree* octree) : OctreeNode(aabb, octree)
{
}

BranchNode::BranchNode(const AABB& aabb, SharedBranchNode parent) : OctreeNode(aabb, parent)
{
}

void BranchNode::initLeafNodes()
{
    int aabbLen = mAABB.max.x - mAABB.min.x;
    TG_ASSERT(mAABB.max.y - mAABB.min.y == aabbLen);
    TG_ASSERT(mAABB.max.z - mAABB.min.z == aabbLen);
    TG_ASSERT(aabbLen % 2 == 0);

    auto thisPtr = shared_from_this();
    for (int i = 0; i < 8; ++i) {
        TG_ASSERT(!mChildNodes[i] && "Pointer must be empty");
        AABB aabb;
        aabb.min.x = mAABB.min.x + ((uint8_t(i) >> 0) & 1) * (aabbLen / 2);
        aabb.min.y = mAABB.min.y + ((uint8_t(i) >> 1) & 1) * (aabbLen / 2);
        aabb.min.z = mAABB.min.z + ((uint8_t(i) >> 2) & 1) * (aabbLen / 2);
        aabb.max = aabb.min + (aabbLen / 2);
        mChildNodes[i] = std::dynamic_pointer_cast<OctreeNode>(std::make_shared<LeafNode>(aabb, thisPtr));
    }
    return;
}

SharedOctreeNode BranchNode::childNode(int idx)
{
    return mChildNodes[idx];
}

SharedOctreeNode BranchNode::parent()
{
    if (!mParentNode.expired())
        return std::dynamic_pointer_cast<OctreeNode>(mParentNode.lock());
    return std::dynamic_pointer_cast<OctreeNode>(std::make_shared<EmptyNode>(mOctree));
}

void BranchNode::pushDown(int meshIdx, pm::face_index faceIdx)
{
    for (int i = 0; i < 8; i++) {
        
        SharedOctreeNode childNode = mChildNodes[i];
        
        if (childNode->polygonInAABB(meshIdx, faceIdx)) {
            if (childNode->nodeBase() == NodeType::LEAF) {
                insertInLeaf(i, meshIdx, faceIdx);
            }            
            else if(childNode->nodeBase() == NodeType::BRANCH) {
                auto branchChild = std::dynamic_pointer_cast<BranchNode>(childNode);
                branchChild->pushDown(meshIdx, faceIdx);
            }
            else {
                //ERROR
            }               
        }
    }
    return;
}

void BranchNode::insertInLeaf(int child, int meshIdx, pm::face_index faceIdx)
{
    TG_ASSERT(mChildNodes[child]->nodeBase() == NodeType::LEAF);
    auto leafChild = std::dynamic_pointer_cast<LeafNode>(mChildNodes[child]);
    leafChild->insertPolygon(meshIdx, faceIdx);

    int test = leafChild->childCount();
    if (leafChild->childCount() > leafChild->maxValues()) {
        if (leafChild->mustSplitIfFull()) {
            auto new_child = leafChild->split();
            mChildNodes[child] = std::dynamic_pointer_cast<OctreeNode>(new_child);
        }           
    }
    return;
}


//#############################################################################
//#                             EmptyNode                                     #
//#############################################################################


EmptyNode::EmptyNode(Octree* octree) : OctreeNode(AABB({ 0,0,0 }, { 0,0,0 }), octree)
{

}

SharedOctreeNode EmptyNode::childNode(int idx)
{
    return SharedOctreeNode();
}

SharedOctreeNode EmptyNode::parent()
{
    return SharedOctreeNode();
}
