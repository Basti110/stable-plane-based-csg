#pragma once
#include <plane_polygon.hh>
#include <nexus/test.hh>
#include <octree.hh>

class DebugRayInfo {
public:
    std::vector<SubDet> rayPath;
    pos_t octreeVerex;
    std::vector<tg::pos3> intersections;
    std::vector<bool> intersectionToInside;

    //Debug direct approach
    std::vector<AABB> rayBoxesDirect;
    pos_t rayStartDirect;
    pos_t rayEndDirect;

    //Cell approach
    std::vector<pos_t> nexPointsCell;
};
using SharedDebugRayInfo = std::shared_ptr<DebugRayInfo>;

struct RayCastInfo {
    int intersections;
    pos_t currentPos;
    SharedOctreeNode currentNode;
};

class RayCast {
public:
    RayCast() = delete;
    RayCast(PlaneMesh& mainMesh, PlaneMesh& otherMesh, SharedOctree octree, SharedDebugRayInfo rayInfo = SharedDebugRayInfo())
        : mMainMesh(mainMesh), mOtherMesh(otherMesh), mOctree(octree) {
        mRayInfo = rayInfo;

        if (rayInfo)
            mHasDebug = true;

        if (mOctree->getPlaneMeshA().id() == mainMesh.id())
            mMainIsMeshA = true;
        else 
            mMainIsMeshA = false;
    }

    int countIntersectionsToOutside(const pm::vertex_handle& origin) {
        auto rayCastInfo = castRayToNextCornerPoint(origin);
        auto curPos = rayCastInfo.currentPos;
        auto node = rayCastInfo.currentNode;

        if (mHasDebug)
            mRayInfo->nexPointsCell.push_back(curPos);

        if (!node->hasParent())
            return -1;

        int8_t nearestPosIndex = -1;

        auto len = mOctree->getRootNode()->AABBLen();
        auto nearestLen = ob::mul<geometry128::bits_position * 2 + 3>(len, len);
        for (int8_t i = 0; i < 7; ++i) {
            auto pos = mOctree->getRootNode()->getPosFromIndex(i);
            auto dis = ob::distancePow2<geometry128>(curPos, pos);
            if (dis < nearestLen) {
                nearestPosIndex = i;
                nearestLen = dis;
            }
        }
        TG_ASSERT(nearestPosIndex != -1);
        return castToParentRecursive(node->parent(), curPos, nearestPosIndex, node->childIndex());
    }

    RayCastInfo castRayToNextCornerPoint(const pm::vertex_handle& origin) {
        SharedLeafNode node = getRightCell(origin);
        //const std::vector<pm::face_index>& FacesMeshA = node->getFacesMeshA();
        //const std::vector<pm::face_index>& FacesMeshB = node->getFacesMeshB();

        const OctreeNodePlanes np = node->getPlanes();

        PlaneRay planeRay = mMainMesh.getRayPlanes(origin);
        const Plane& basePlane = mMainMesh.getAnyFace(origin);
        TG_ASSERT(!(basePlane == planeRay.plane1) && !(basePlane == planeRay.plane1));

        PlanePolygon nodeSides[6] = {
        { np.xyFront, {np.xzTop, np.yzLeft, np.xzBottom, np.yzRight}},
        { np.xyBack, {np.xzTop, np.yzLeft, np.xzBottom, np.yzRight}},
        { np.xzTop, {np.xyFront, np.yzRight, np.xyBack, np.yzLeft}},
        { np.xzBottom, {np.xyFront, np.yzRight, np.xyBack, np.yzLeft}},
        { np.yzRight, {np.xzTop, np.xyFront, np.xzBottom, np.xyBack}},
        { np.yzLeft, {np.xzTop, np.xyFront, np.xzBottom, np.xyBack}} };

        for (int i = 0; i < 6; ++i) {
            auto side = nodeSides[i];
            if (IntersectionObject::isIntersecting(planeRay, side)) {
                PlaneRay nextRay = { planeRay.plane1, side.basePlane };

                for (int j = 0; j < 4; j++) {
                    auto& sidePlane = side.edgePlanes[j];
                    auto& nextSidePlane = side.edgePlanes[(j + 1) % 4];
                    PlaneInterval interval = { nextSidePlane, side.edgePlanes[(j + 3) % 4] };
                    if (intersectInInterval(nextRay, sidePlane, interval)) {
                        const Plane& thirdPlane = nextSidePlane;

                        pos_t point = getPointFromPlaneIndex(i, j, node);
                        if (mHasDebug) {
                            fillRayInfo({ basePlane , planeRay.plane1, planeRay.plane2 }, side.basePlane, sidePlane, thirdPlane);
                            mRayInfo->octreeVerex = point;
                        }
                        int intersectionCount = castToNextPlanes({ basePlane , planeRay.plane1, planeRay.plane2 }, node, side.basePlane, sidePlane, thirdPlane);
                        return { intersectionCount, point,  std::dynamic_pointer_cast<OctreeNode>(node) };
                    }
                }
            }
        }
        return { -1, {0, 0, 0} };
    }

    SharedLeafNode getRightCell(const pm::vertex_handle& origin) {
        SharedOctreeNode node = mOctree->getRootNode();
        while (node->nodeBase() != OctreeNode::NodeType::LEAF) {
            int8_t child = 0;
            auto aabb = node->aabb();
            auto dVec = aabb.max - ((aabb.max - aabb.min) / 2);
            Plane xPlane = Plane{ 1, 0, 0, Plane::distance_t(-dVec.x) };
            Plane yPlane = Plane{ 0, 1, 0, Plane::distance_t(-dVec.y) };
            Plane zPlane = Plane{ 0, 0, 1, Plane::distance_t(-dVec.z) };
            if (mMainMesh.getSign(origin, xPlane) > 0)
                child |= 0x1;
            if (mMainMesh.getSign(origin, yPlane) > 0)
                child |= 0x2;
            if (mMainMesh.getSign(origin, zPlane) > 0)
                child |= 0x4;
            TG_ASSERT(node->nodeBase() == OctreeNode::NodeType::BRANCH);
            auto branchNode = std::dynamic_pointer_cast<BranchNode>(node);
            node = branchNode->childNode(child);
        }
        return std::dynamic_pointer_cast<LeafNode>(node);
    }

    bool intersectInInterval(PlaneRay planeRay, const Plane& plane, PlaneInterval& interval) {
        SubDet subDet = mMainMesh.pos(planeRay.plane1, planeRay.plane2, plane);

        if (!subDet.is_valid())
            return false;

        int8_t sign = ob::classify_vertex(subDet, interval.plane1);
        if (ob::classify_vertex(subDet, interval.plane1) >= 1)
            return false;

        sign = ob::classify_vertex(subDet, interval.plane2);
        if (ob::classify_vertex(subDet, interval.plane2) >= 1)
            return false;

        return true;
    }

    int castToNextPlanes(const PlanePoint& origin, SharedLeafNode node, const Plane& first, const Plane& second, const Plane& third) {
        //std::vector<int8_t> singsMeshA(node->getFacesMeshA().size());
        auto& facesList = mMainIsMeshA ? node->getFacesMeshB() : node->getFacesMeshA();
        std::vector<int8_t> singsMesh(facesList.size());
        size_t intersectionCount = 0;

        SubDet det = mMainMesh.pos(origin.basePlane, origin.edgePlane1, origin.edgePlane2);
        /*for (int i = 0; i < singsMeshA.size(); ++i) {
            const Plane& facePlane = mMainMesh.face(node->mFacesMeshA[i]);
            auto sign = ob::classify_vertex(det, facePlane);
            singsMeshA[i] = sign == 0 ? 2 : sign;
        }*/

        for (int i = 0; i < singsMesh.size(); ++i) {
            const Plane& facePlane = mOtherMesh.face(facesList[i]);
            auto sign = ob::classify_vertex(det, facePlane);
            singsMesh[i] = sign == 0 ? 2 : sign;
        }

        Plane edgePlane = origin.edgePlane1; // !ob::are_parallel(origin.edgePlane1, second) ? origin.edgePlane1 : origin.edgePlane2;

        det = mMainMesh.pos(first, origin.edgePlane1, origin.edgePlane2);
        intersectionCount += intersectionToNextPoint(origin.edgePlane1, origin.edgePlane2, det, node, singsMesh);

        det = mMainMesh.pos(first, second, edgePlane);
        TG_ASSERT(det.is_valid());
        intersectionCount += intersectionToNextPoint(first, edgePlane, det, node, singsMesh);

        det = mMainMesh.pos(first, second, third);
        intersectionCount += intersectionToNextPoint(first, second, det, node, singsMesh);

        return intersectionCount;
    }

    size_t intersectionToNextPoint(const Plane& p1, const Plane& p2, SubDet& subDet, SharedLeafNode node, std::vector<int8_t>& singsMesh) {
        size_t intersectionCount = 0;
        auto& facesList = mMainIsMeshA ? node->getFacesMeshB() : node->getFacesMeshA();
        //Mesh A
        for (int i = 0; i < singsMesh.size(); ++i) {
            auto face = facesList[i].of(mOtherMesh.mesh());
            if (face.is_removed())
                continue;
            const Plane& facePlane = mOtherMesh.face(facesList[i]);
            auto sign = ob::classify_vertex(subDet, facePlane);
            if (singsMesh[i] == 2)
                singsMesh[i] = sign;

            if (sign == singsMesh[i] || sign == 0)
                continue;

            singsMesh[i] = sign;
            auto subDetFacePlane = mOtherMesh.pos(p1, p2, facePlane);
            
            if (checkIfPointInPolygon(face, mOtherMesh, subDetFacePlane)) {
                intersectionCount++;
            }

        }
        /*//Mesh B
        for (int i = 0; i < node->mFacesMeshB.size(); ++i) {
            auto face = node->mFacesMeshB[i].of(mMeshB->mesh());
            if (face.is_removed())
                continue;
            const Plane& facePlane = mMeshB->face(node->mFacesMeshB[i]);
            auto sign = ob::classify_vertex(subDet, facePlane);
            if (singsMeshB[i] == 2)
                singsMeshB[i] == sign;

            if (sign == singsMeshB[i] || sign == 0)
                continue;

            singsMeshB[i] = sign;
            auto subDetFacePlane = mMeshA->pos(p1, p2, facePlane);
            if (checkIfPointInPolygon(face, mMeshB, subDetFacePlane)) {
                intersectionCount++;
            }
        }*/
        return intersectionCount;
    }

    bool checkIfPointInPolygon(pm::face_handle face, PlaneMesh& mesh, SubDet& subDet) {
        for (auto halfEdge : face.halfedges()) {
            TG_ASSERT(!ob::are_parallel(mesh.edge(halfEdge.edge()), mesh.face(face)));
            int8_t sign = ob::classify_vertex(subDet, mesh.edge(halfEdge.edge()));
            if (sign * mesh.halfedge(halfEdge) >= 1)
                return false;
        }
        return true;
    }

    void fillRayInfo(const PlanePoint& origin, const Plane& first, const Plane& second, const Plane& third) {
        TG_ASSERT(!ob::are_parallel(origin.basePlane, origin.edgePlane1));
        TG_ASSERT(!ob::are_parallel(origin.basePlane, origin.edgePlane2));
        TG_ASSERT(!ob::are_parallel(origin.edgePlane1, origin.edgePlane2));
        TG_ASSERT(!ob::are_parallel(origin.edgePlane1, first));
        TG_ASSERT(!ob::are_parallel(origin.edgePlane2, first));
        TG_ASSERT(!ob::are_parallel(first, second));
        TG_ASSERT(!ob::are_parallel(first, third));
        TG_ASSERT(!ob::are_parallel(second, third));
        int counter = 0;
        Plane edgePlane = origin.edgePlane1;

        SubDet subDet = mMainMesh.pos(origin.basePlane, origin.edgePlane1, origin.edgePlane2);
        mRayInfo->rayPath.push_back(subDet);
           
        subDet = mMainMesh.pos(origin.edgePlane1, origin.edgePlane2, first);
        mRayInfo->rayPath.push_back(subDet);

        subDet = mMainMesh.pos(edgePlane, first, second);
        mRayInfo->rayPath.push_back(subDet);

        subDet = mMainMesh.pos(first, second, third);
        mRayInfo->rayPath.push_back(subDet);

    }

    int castToParentRecursive(SharedBranchNode node, pos_t p, int8_t parenIndex, uint8_t childIdx) {
        pos_t pos = p;

        /*rayInfo->nexPointsCell.push_back(node->aabb().max);
        node->getAllBoundingBoxes(tg::vec3(ray), pos, rayInfo->rayBoxesDirect, exludeChild);
        if (node->hasParent())
            castToParentRecursive(node->parent(), node->aabb().max, node->childIndex(), rayInfo);*/
        pos_t parenPos = node->getPosFromIndex(parenIndex);

        int intersectionCount = 0;
        auto childIndex = node->childIndex();
        if (pos.x != parenPos.x) {


            Plane xyPlane = Plane::from_pos_normal(pos, { 0, 0, -1 });
            Plane xzPlane = Plane::from_pos_normal(pos, { 0, 1, 0 });
            pos_t newPos = pos_t{ parenPos.x, pos.y, pos.z };
            std::vector<SharedOctreeNode> nodes{ node->childNode(childIdx), node->childNode(mOctree->getXNeightboor(childIdx)) };
            vec_t ray = newPos - pos;
            intersectionCount += intersectionToNextPointThroughNodes(nodes, tg::vec3(ray), PlaneRay{ xyPlane , xzPlane }, pos, newPos);
            if (mRayInfo) {
                mRayInfo->nexPointsCell.push_back(newPos);
                nodes[0]->getAllBoundingBoxes(tg::normalize(tg::vec3(ray)), pos, mRayInfo->rayBoxesDirect);
                nodes[1]->getAllBoundingBoxes(tg::normalize(tg::vec3(ray)), pos, mRayInfo->rayBoxesDirect);
                //rayInfo->rayBoxesDirect.push_back(nodes[0]->aabb());
                //rayInfo->rayBoxesDirect.push_back(nodes[1]->aabb());
            }

            if (pos.x < parenPos.x)
                childIdx |= 0x1;
            else
                childIdx &= 0x6;

            pos = newPos;
        }
        if (pos.y != parenPos.y) {
            Plane yxPlane = Plane::from_pos_normal(pos, { 0, 0, -1 });
            Plane yzPlane = Plane::from_pos_normal(pos, { 1, 0, 0 });
            pos_t newPos = pos_t{ pos.x, parenPos.y, pos.z };
            std::vector<SharedOctreeNode> nodes{ node->childNode(childIdx), node->childNode(mOctree->getYNeightboor(childIdx)) };
            vec_t ray = newPos - pos;
            intersectionCount += intersectionToNextPointThroughNodes(nodes, tg::vec3(ray), PlaneRay{ yxPlane , yzPlane }, pos, newPos);
            if (mRayInfo) {
                mRayInfo->nexPointsCell.push_back(newPos);
                nodes[0]->getAllBoundingBoxes(tg::normalize(tg::vec3(ray)), pos, mRayInfo->rayBoxesDirect);
                nodes[1]->getAllBoundingBoxes(tg::normalize(tg::vec3(ray)), pos, mRayInfo->rayBoxesDirect);
                //rayInfo->rayBoxesDirect.push_back(nodes[0]->aabb());
                //rayInfo->rayBoxesDirect.push_back(nodes[1]->aabb());
            }
            if (pos.y < parenPos.y)
                childIdx |= 0x2;
            else
                childIdx &= 0x5;
            pos = newPos;
        }
        if (pos.z != parenPos.z) {
            Plane zxPlane = Plane::from_pos_normal(pos, { 1, 0, 0 });
            Plane zyPlane = Plane::from_pos_normal(pos, { 0, 1, 0 });
            pos_t newPos = pos_t{ pos.x, pos.y, parenPos.z };
            std::vector<SharedOctreeNode> nodes{ node->childNode(childIdx), node->childNode(mOctree->getZNeightboor(childIdx)) };
            vec_t ray = newPos - pos;
            intersectionCount += intersectionToNextPointThroughNodes(nodes, tg::vec3(ray), PlaneRay{ zxPlane , zyPlane }, pos, newPos);
            if (mRayInfo) {
                mRayInfo->nexPointsCell.push_back(newPos);
                nodes[0]->getAllBoundingBoxes(tg::normalize(tg::vec3(ray)), pos, mRayInfo->rayBoxesDirect);
                nodes[1]->getAllBoundingBoxes(tg::normalize(tg::vec3(ray)), pos, mRayInfo->rayBoxesDirect);
                //rayInfo->rayBoxesDirect.push_back(nodes[0]->aabb());
                //rayInfo->rayBoxesDirect.push_back(nodes[1]->aabb());
            }
            if (pos.y < parenPos.y)
                childIdx |= 0x4;
            else
                childIdx &= 0x3;
            pos = newPos;
        }
        if (node->hasParent())
            intersectionCount += castToParentRecursive(node->parent(), pos, parenIndex, node->childIndex());
        return intersectionCount;
    }

    size_t intersectionToNextPointThroughNodes(std::vector<SharedOctreeNode> nodes, tg::vec3 ray, PlaneRay& planeRay, pos_t startPoint, pos_t endPoint) {
        std::set<pm::face_index> facesMesh;
        //std::set<pm::face_index> facesMeshB;

        for (auto node : nodes) {
            if(mMainIsMeshA)
                node->getAllFacesB(ray, startPoint, facesMesh);
            else
                node->getAllFacesA(ray, startPoint, facesMesh);
        }

        std::vector<int8_t> singsMesh(facesMesh.size());
        //std::vector<int8_t> singsMeshB(facesMeshB.size());

        auto setIt = facesMesh.begin();
        for (int i = 0; i < facesMesh.size(); ++i) {
            const Plane& facePlane = mOtherMesh.face(*setIt);
            auto sign = ob::sign_of(ob::signed_distance(facePlane, startPoint));
            singsMesh[i] = sign == 0 ? 2 : sign;
            ++setIt;
        }
        /*setIt = facesMeshB.begin();
        for (int i = 0; i < facesMeshB.size(); ++i) {
            const Plane& facePlane = mMeshB->face(*setIt);
            auto sign = ob::sign_of(ob::signed_distance(facePlane, startPoint));
            singsMeshB[i] = sign == 0 ? 2 : sign;
            ++setIt;
        }*/

        size_t intersectionCount = 0;
        intersectionCount += faceIntersections(mOtherMesh, facesMesh, planeRay, singsMesh, endPoint);
        //intersectionCount += faceIntersections(mMeshB, facesMeshB, planeRay, singsMeshB, endPoint);
        return intersectionCount;
    }

    size_t faceIntersections(PlaneMesh& mesh, std::set<pm::face_index>& faces, PlaneRay& planeRay, std::vector<int8_t>& startSigns, pos_t endPoint) {
        size_t intersectionCount = 0;
        auto setIt = faces.begin();
        for (int i = 0; i < faces.size(); ++i) {
            auto face = (*setIt).of(mesh.mesh());
            if (face.is_removed())
                continue;

            const Plane& facePlane = mesh.face(*setIt);
            auto distance = ob::signed_distance<geometry128>(facePlane, endPoint);
            int8_t sign = distance >= 0 ? (distance == 0 ? 0 : 1) : -1;
            if (startSigns[i] == 2)
                startSigns[i] = sign;

            if (sign == startSigns[i] || sign == 0) {
                ++setIt;
                continue;
            }

            startSigns[i] = sign;
            auto subDetFacePlane = mesh.pos(planeRay.plane1, planeRay.plane2, facePlane);
            if (checkIfPointInPolygon(face, mOtherMesh, subDetFacePlane))
                intersectionCount++;
            ++setIt;
        }
        return intersectionCount;
    }

    static pos_t getPointFromPlaneIndex(int i1, int i2, SharedLeafNode node) {
        if (i1 == 0) {
            if (i2 == 0)
                return node->getPosFromVertexIndex(4);
            else if (i2 == 1)
                return node->getPosFromVertexIndex(8);
            else if (i2 == 2)
                return node->getPosFromVertexIndex(5);
            else if (i2 == 3)
                return node->getPosFromVertexIndex(1);
        }
        else if (i1 == 1) {
            if (i2 == 0)
                return node->getPosFromVertexIndex(3);
            else if (i2 == 1)
                return node->getPosFromVertexIndex(7);
            else if (i2 == 2)
                return node->getPosFromVertexIndex(6);
            else if (i2 == 3)
                return node->getPosFromVertexIndex(2);
        }
        else if (i1 == 2) {
            if (i2 == 0)
                return node->getPosFromVertexIndex(1);
            else if (i2 == 1)
                return node->getPosFromVertexIndex(2);
            else if (i2 == 2)
                return node->getPosFromVertexIndex(3);
            else if (i2 == 3)
                return node->getPosFromVertexIndex(4);
        }
        else if (i1 == 3) {
            if (i2 == 0)
                return node->getPosFromVertexIndex(5);
            else if (i2 == 1)
                return node->getPosFromVertexIndex(6);
            else if (i2 == 2)
                return node->getPosFromVertexIndex(7);
            else if (i2 == 3)
                return node->getPosFromVertexIndex(8);
        }
        else if (i1 == 4) {
            if (i2 == 0)
                return node->getPosFromVertexIndex(4);
            else if (i2 == 1)
                return node->getPosFromVertexIndex(8);
            else if (i2 == 2)
                return node->getPosFromVertexIndex(7);
            else if (i2 == 3)
                return node->getPosFromVertexIndex(3);
        }
        else if (i1 == 5) {
            if (i2 == 0)
                return node->getPosFromVertexIndex(1);
            else if (i2 == 1)
                return node->getPosFromVertexIndex(5);
            else if (i2 == 2)
                return node->getPosFromVertexIndex(6);
            else if (i2 == 3)
                return node->getPosFromVertexIndex(2);
        }
        return { 0, 0,0 };
    }

private: 
    bool mMainIsMeshA;
    bool mHasDebug = false;
    PlaneMesh& mMainMesh;
    PlaneMesh& mOtherMesh;
    SharedOctree mOctree;
    SharedDebugRayInfo mRayInfo;
};