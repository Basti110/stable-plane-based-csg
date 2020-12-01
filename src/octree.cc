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
int8_t OctreeNode::polygonInAABB(int meshIdx, pm::face_index faceIdx)
{
    //TRACE("[Octree] AABB Test");
    TG_ASSERT(mOctree);
    PlaneMesh* mesh = meshIdx == mOctree->mMeshA->id() ? mOctree->mMeshA : mOctree->mMeshB;
    auto intersection = ob::intersection_type<geometry128>(*mesh, faceIdx.of(mesh->mesh()), mAABB);
    return (int8_t)intersection;
}

int8_t OctreeNode::polygonInAABBNew(int meshIdx, pm::face_index faceIdx)
{
    int8_t childBits = 0xFF;
    PlaneMesh* planeMesh = meshIdx == mOctree->mMeshA->id() ? mOctree->mMeshA : mOctree->mMeshB;
    pm::face_handle face = faceIdx.of(planeMesh->mesh());
    auto h = face.any_halfedge();
    auto center = vec_t(mAABB.max) + vec_t(mAABB.min); 
    auto const p0 = pos_t(scalar_t(planeMesh->posInt(h.vertex_from()).x) << 1, //
        scalar_t(planeMesh->posInt(h.vertex_from()).y) << 1, //
        scalar_t(planeMesh->posInt(h.vertex_from()).z) << 1)
        - center;

    auto const p1 = pos_t(scalar_t(planeMesh->posInt(h.vertex_to()).x) << 1, //
        scalar_t(planeMesh->posInt(h.vertex_to()).y) << 1, //
        scalar_t(planeMesh->posInt(h.vertex_to()).z) << 1)
        - center;

    auto const p2 = pos_t(scalar_t(planeMesh->posInt(h.next().vertex_to()).x) << 1, //
        scalar_t(planeMesh->posInt(h.next().vertex_to()).y) << 1, //
        scalar_t(planeMesh->posInt(h.next().vertex_to()).z) << 1)
        - center;

    auto signToPlane = [=](int index) -> int8_t {
        int signCount = 0;
        auto t1 = p0[index];
        auto t2 = p1[index];
        auto t3 = p2[index];
        signCount += p0[index] > 0 ? 1 : p0[index] < 0 ? -1 : 0;
        signCount += p1[index] > 0 ? 1 : p1[index] < 0 ? -1 : 0;
        signCount += p2[index] > 0 ? 1 : p2[index] < 0 ? -1 : 0;
        if (signCount == 3)
            return 1;
        else if (signCount * -1 == 3)
            return -1;
        return 0;
    };

    
    //Plane xPlane = Plane{ 1, 0, 0, Plane::distance_t(-dVec.x) };
    auto sign = signToPlane(0);
    if (sign == 1)
        childBits &= 0b10101010;
    else if(sign == -1)
        childBits &= 0b01010101;
    //Plane yPlane = Plane{ 0, 1, 0, Plane::distance_t(-dVec.y) };
    sign = signToPlane(1);
    if (sign == 1)
        childBits &= 0b11001100;
    else if (sign == -1)
        childBits &= 0b00110011;
    //Plane zPlane = Plane{ 0, 0, 1, Plane::distance_t(-dVec.z) };
    sign = signToPlane(2);
    if (sign == 1)
        childBits &= 0b011110000;
    else if (sign == -1)
        childBits &= 0b00001111;
    
    TG_ASSERT(childBits != 0);
    if (childBits == 0xFF)
        int i = 5;
    return childBits;
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

SharedBranchNode LeafNode::parent()
{
    if (!mParentNode.expired())
        return mParentNode.lock();
    return SharedBranchNode();
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
    auto branchNode = std::make_shared<BranchNode>(mAABB, mParentNode.lock());
    branchNode->setChildIndex(mChildIndex);
    branchNode->initLeafNodes();
    for (auto& face : mFacesMeshA)
        branchNode->pushDown(mOctree->mMeshA->id(), face);
    for (auto& face : mFacesMeshB)
        branchNode->pushDown(mOctree->mMeshB->id(), face);
    mFacesMeshA.clear();
    mFacesMeshB.clear();
    mIsValid = false;
    mOctree->increaseSplitcount();
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

int LeafNode::countIntersections(SharedIntersectionList intersectionList)
{
    if (mFacesMeshA.size() <= 0 || mFacesMeshB.size() <= 0)
        return 0;
    int intersectionCount = 0;
    for (pm::face_index face1Index : mFacesMeshA) {
        auto face1 = face1Index.of(mOctree->mMeshA->mesh());
        for (pm::face_index face2Index : mFacesMeshB) {
            auto face2 = face2Index.of(mOctree->mMeshB->mesh());
            if (intersectionList) {
                if (intersectionList->count(face1Index.value) == 0) {
                    intersectionList->operator[](face1Index.value) = std::unordered_set<int>();
                }
                else {
                    if (intersectionList->operator[](face1Index.value).count(face2Index.value) != 0)
                        continue;
                }
            }
            if (IsectOb(*(mOctree->mMeshA), *(mOctree->mMeshB)).intersect<geometry128>(face1, face2)->intersectionState != TrianlgeIntersection::IntersectionState::NON_INTERSECTING) {
                intersectionCount++;
                if(intersectionList)
                    intersectionList->operator[](face1Index.value).insert(face2Index.value);
            }               
        }
    }
    return intersectionCount;
}

void LeafNode::repairCell(const IntersectionCut& cut)
{
    std::vector<pm::face_index> faces1;
    std::vector<pm::face_index> faces2;
    faces1.reserve(mFacesMeshA.size());
    faces2.reserve(mFacesMeshB.size());


    //TODO: Cann remove first IF
    for (pm::face_index& faceIndex : mFacesMeshA) {
        mOctree->fillFacesFromLookupInVec(faces1, faceIndex.of(mOctree->mMeshA->mesh()), cut.getLookUpA(), true);
    }

    for (pm::face_index& faceIndex : mFacesMeshB) {
        mOctree->fillFacesFromLookupInVec(faces2, faceIndex.of(mOctree->mMeshB->mesh()), cut.getLookUpB(), false);
    }
    mFacesMeshA = faces1;
    mFacesMeshB = faces2;   
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
    scalar_t aabbLen = mAABB.max.x - mAABB.min.x;
    TG_ASSERT(mAABB.max.y - mAABB.min.y == aabbLen);
    TG_ASSERT(mAABB.max.z - mAABB.min.z == aabbLen);
    TG_ASSERT(aabbLen % 2 == 0);
    mOctree->setSmallestCellLen(aabbLen / 2);

    auto thisPtr = shared_from_this();
    for (int i = 0; i < 8; ++i) {
        TG_ASSERT(!mChildNodes[i] && "Pointer must be empty");
        AABB aabb;
        aabb.min.x = mAABB.min.x + ((uint8_t(i) >> 0) & 1) * (aabbLen / 2);
        aabb.min.y = mAABB.min.y + ((uint8_t(i) >> 1) & 1) * (aabbLen / 2);
        aabb.min.z = mAABB.min.z + ((uint8_t(i) >> 2) & 1) * (aabbLen / 2);
        aabb.max = aabb.min + (aabbLen / 2);
        auto leaf = std::make_shared<LeafNode>(aabb, thisPtr);
        leaf->setChildIndex(i);
        mChildNodes[i] = std::dynamic_pointer_cast<OctreeNode>(leaf);
    }
    return;
}

SharedOctreeNode BranchNode::childNode(int idx)
{
    return mChildNodes[idx];
}

SharedBranchNode BranchNode::parent()
{
    if (!mParentNode.expired())
        return mParentNode.lock();
    return SharedBranchNode();
}

void BranchNode::pushDown(int meshIdx, pm::face_index faceIdx)
{
    //Old
    /*for (int i = 0; i < 8; i++) {
        
        SharedOctreeNode childNode = mChildNodes[i];
        auto iResult = ob::intersection_result(childNode->polygonInAABB(meshIdx, faceIdx));

        if (iResult == ob::intersection_result::non_intersecting)
            mOctree->mNonIntersecting++;
        if (iResult == ob::intersection_result::proper_contains)
            mOctree->mProperContains++;
        if (iResult == ob::intersection_result::proper_intersection)
            mOctree->mProperIntersection++;
        if (iResult == ob::intersection_result::touching)
            mOctree->mTouching++;
        
        if (iResult != ob::intersection_result::non_intersecting) {
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
            if (iResult == ob::intersection_result::proper_contains)
                return;
        }
    }*/
    int8_t childCells = polygonInAABBNew(meshIdx, faceIdx);
    for (int8_t i = 0; i < 8; ++i) {
        if ((childCells >> i) & 1 == 1) {            
            SharedOctreeNode childNode = mChildNodes[i];
            auto iResult = ob::intersection_result(childNode->polygonInAABB(meshIdx, faceIdx));
            if (iResult == ob::intersection_result::non_intersecting)
                mOctree->mNonIntersecting++;
            if (iResult == ob::intersection_result::proper_contains)
                mOctree->mProperContains++;
            if (iResult == ob::intersection_result::proper_intersection)
                mOctree->mProperIntersection++;
            if (iResult == ob::intersection_result::touching)
                mOctree->mTouching++;
            if (iResult == ob::intersection_result::non_intersecting)
                continue;

            if (childNode->nodeBase() == NodeType::LEAF) {
                insertInLeaf(i, meshIdx, faceIdx);
            }
            else if (childNode->nodeBase() == NodeType::BRANCH) {
                auto branchChild = std::dynamic_pointer_cast<BranchNode>(childNode);
                branchChild->pushDown(meshIdx, faceIdx);
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

SharedBranchNode EmptyNode::parent()
{
    return SharedBranchNode();
}
