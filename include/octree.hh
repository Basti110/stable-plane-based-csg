#pragma once
#include <typed-geometry/tg.hh>
#include <geometry.hh>
#include <memory>
#include <array>
#include <unordered_map>
#include <imgui/imgui.h>
//#include <set>
#include "plane_polygon.hh"
//intersection and cut
#include <intersection_cut.hh>
#include <face_marker.hh>
#include <glow-extras/viewer/view.hh>
#include <glow-extras/viewer/experimental.hh>

class OctreeNode;
class BranchNode;
class LeafNode;
class EmptyNode;
class Octree;
using SharedOctreeNode = std::shared_ptr<OctreeNode>;
using WeakOctreeNode = std::weak_ptr<OctreeNode>;
using SharedLeafNode = std::shared_ptr<LeafNode>;
using SharedBranchNode = std::shared_ptr<BranchNode>;
using WeakBranchNode = std::weak_ptr<BranchNode>;
using SharedEmptyNode = std::shared_ptr<EmptyNode>;
using SharedOctree = std::shared_ptr<Octree>;
using AABB = tg::aabb<3, scalar_t>;

struct PolygonIndex
{
    int mMeshIndex;
    int mPolyIndex;
};

struct NearestFace
{
    int meshID;
    pm::face_index faceIndex;
    double distance;
};

struct BoxAndDistance {
    AABB box = AABB();
    scalar_t dinstance = -1;
};

struct RayCastInfo {
    int intersections;
    pos_t currentPos;
    SharedOctreeNode currentNode;
};

class OctreeNodePlanes {
public:
    OctreeNodePlanes() {};
    OctreeNodePlanes(Plane& xyFront, Plane& xyBack, Plane& xzTop, Plane& xzBottom, Plane& yzRight, Plane& yzLeft) :
        planes{ xyFront, xyBack, xzTop, xzBottom, yzRight, yzLeft }
    {}
    OctreeNodePlanes(const OctreeNodePlanes& other) {
        planes[0] = other.planes[0];
        planes[1] = other.planes[1];
        planes[2] = other.planes[2];
        planes[3] = other.planes[3];
        planes[4] = other.planes[4];
        planes[5] = other.planes[5];
    }

    Plane planes[6];
    const Plane& xyFront = planes[0];
    const Plane& xyBack = planes[1];
    const Plane& xzTop = planes[2];
    const Plane& xzBottom = planes[3];
    const Plane& yzRight = planes[4];
    const Plane& yzLeft = planes[5];
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
    OctreeNode(const AABB& aabb, SharedBranchNode parent) : mAABB(aabb), mIsValid(true) {
        mParentNode = parent;
        mOctree = std::dynamic_pointer_cast<OctreeNode>(parent)->octree();
    }

    void insertAABB(std::vector<AABB>& insertVec);
    virtual SharedOctreeNode childNode(int idx) = 0;
    //virtual OctreeNode& childNodeRef(int idx) = 0;
    virtual NodeType nodeBase() = 0;
    virtual SharedBranchNode parent() { return mParentNode.lock(); }
    virtual void cutPolygons(IntersectionCut& lookup) {};
    virtual void markIntersections(pm::face_attribute<tg::color3>& faceColor1, pm::face_attribute<tg::color3>& faceColor2) {}
    virtual BoxAndDistance getNearestBoundingBox(tg::vec3 ray, pos_t origin) { return { AABB(), -1 }; }
    virtual NearestFace getNearestFace(tg::vec3 ray, pos_t origin) { return { -1, pm::face_index(), -1 }; }
    virtual void getAllBoundingBoxes(tg::vec3 ray, pos_t origin, std::vector<AABB>& boxes, uint8_t exludeIndex = 255) { }
    virtual void getAllFaces(tg::vec3 ray, pos_t origin, std::vector<pm::face_index>& indices, uint8_t exludeIndex = 255) { }
    bool isInTree();
    bool hasParent();
    bool polygonInAABB(int meshIdx, pm::face_index faceIdx);
    bool isValid() { return mIsValid; }
    void setParent(SharedBranchNode node);
    uint8_t childIndex() { return mChildIndex; }
    Octree* octree() { return mOctree; }
    AABB aabb() { return mAABB; }


    OctreeNodePlanes getPlanes() {
        TG_ASSERT(mAABB != AABB());
        OctreeNodePlanes planes;

        planes.planes[0] = Plane::from_pos_normal(mAABB.max, { 0, 0, 1 });
        planes.planes[1] = Plane::from_pos_normal(mAABB.min, { 0, 0, -1 });
        planes.planes[2] = Plane::from_pos_normal(mAABB.max, { 0, 1, 0 });
        planes.planes[3] = Plane::from_pos_normal(mAABB.min, { 0, -1, 0 });
        planes.planes[4] = Plane::from_pos_normal(mAABB.max, { 1, 0, 0 });
        planes.planes[5] = Plane::from_pos_normal(mAABB.min, { -1, 0, 0 });
        return planes;
    }

    pos_t getPosFromVertexIndex(size_t i) {
        scalar_t len = mAABB.max.x - mAABB.min.y;
        if (i == 1)
            return mAABB.max;
        else if (i == 2)
            return mAABB.max + vec_t{ 0, 0, -len };
        else if (i == 3)
            return mAABB.max + vec_t{ 0, -len, -len };
        else if (i == 4)
            return mAABB.max + vec_t{ 0, -len, 0 };
        else if (i == 5)
            return mAABB.min + vec_t{ 0, len, len };
        else if (i == 6)
            return mAABB.min + vec_t{ 0, len, 0 };
        else if (i == 7)
            return mAABB.min;
        else if (i == 8)
            return mAABB.min + vec_t{ 0, 0, len };
    }
     
    virtual bool isNotLeafOrContainsData() {
        return true;
    }

    bool intersect(tg::vec3 ray, pos_t origin)
    {
        double tmin = (mAABB.min.x - origin.x) / ray.x;
        double tmax = (mAABB.max.x - origin.x) / ray.x;

        if (tmin > tmax) std::swap(tmin, tmax);

        double tymin = (mAABB.min.y - origin.y) / ray.y;
        double tymax = (mAABB.max.y - origin.y) / ray.y;

        if (tymin > tymax) std::swap(tymin, tymax);

        if ((tmin > tymax) || (tymin > tmax))
            return false;

        if (tymin > tmin)
            tmin = tymin;

        if (tymax < tmax)
            tmax = tymax;

        double tzmin = (mAABB.min.z - origin.z) / ray.z;
        double tzmax = (mAABB.max.z - origin.z) / ray.z;

        if (tzmin > tzmax) std::swap(tzmin, tzmax);

        if ((tmin > tzmax) || (tzmin > tmax))
            return false;

        if (tzmin > tmin)
            tmin = tzmin;

        if (tzmax < tmax)
            tmax = tzmax;

        return true;
    }

    scalar_t distance(pos_t point) {
        pos_t origin = mAABB.min + (mAABB.max - mAABB.min) / 2;
        vec_t distance = point - origin;
        return tg::length(distance);
    }


    

    Octree* getOctree();

protected: 
    bool mIsValid = false;
    WeakBranchNode mParentNode;
    AABB mAABB;
    Octree* mOctree;
    uint8_t mChildIndex = 255;
};

class LeafNode : public OctreeNode, public std::enable_shared_from_this<LeafNode> {
public:
    LeafNode(const AABB& aabb, Octree* octree);
    LeafNode(const AABB& aabb, SharedBranchNode parent);
    //static SharedLeafNode create() { return std::make_shared<LeafNode>(); };
    //static SharedLeafNode create(SharedOctreeNode parent) { return std::make_shared<LeafNode>(parent); };

    NodeType nodeBase() override { return NodeType::LEAF; }
    SharedOctreeNode childNode(int idx) override;
    //OctreeNode& childNodeRef(int idx) override;
    SharedBranchNode parent() override;
    void setChildIndex(uint8_t i) { mChildIndex = i; }
    void insertPolygon(int meshIdx, pm::face_index faceIdx);
    int childCount();
    bool mustSplitIfFull();
    int maxValues() { return mMaxValues; }
    
    SharedBranchNode split();
    void markIntersections(pm::face_attribute<tg::color3>& faceColor1, pm::face_attribute<tg::color3>& faceColor2) override;
    void cutPolygons(IntersectionCut& lookup) override;
    BoxAndDistance getNearestBoundingBox(tg::vec3 ray, pos_t origin) override {
        return { mAABB, distance(origin) };
    }

    bool isNotLeafOrContainsData() override {
        return (mFacesMeshA.size() > 0 || mFacesMeshB.size() > 0);
    }

    void getAllBoundingBoxes(tg::vec3 ray, pos_t origin, std::vector<AABB>& boxes, uint8_t exludeIndex = 255) override {
        if (mFacesMeshA.size() > 0 || mFacesMeshB.size() > 0)
            boxes.push_back(mAABB);
    }

    void getAllFaces(tg::vec3 ray, pos_t origin, std::vector<pm::face_index>& indices, uint8_t exludeIndex = 255) override {
        if (mFacesMeshA.size() > 0)
            indices.insert(indices.end(), mFacesMeshA.begin(), mFacesMeshA.end());
            
        if (mFacesMeshB.size() > 0)
            indices.insert(indices.end(), mFacesMeshB.begin(), mFacesMeshB.end());
    }

    /*void splitAccordingToIntersection() {
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
    }*/
    NearestFace getNearestFace(tg::vec3 ray, pos_t origin) override;

private: 
    friend class Octree;
    std::vector<uint32_t> mValueIndices;
    std::vector<pm::face_index> mFacesMeshA;
    std::vector<pm::face_index> mFacesMeshB;
    int mMaxValues = 15;
};

class BranchNode : public OctreeNode, public std::enable_shared_from_this<BranchNode>{
public:
    BranchNode(const AABB& aabb, Octree* octree);
    BranchNode(const AABB& aabb, SharedBranchNode parent);
    //static SharedBranchNode create() { return std::make_shared<BranchNode>(); };
    //static SharedBranchNode create(SharedOctreeNode parent) { return std::make_shared<BranchNode>(parent); };

    NodeType nodeBase() override { return NodeType::BRANCH; }
    SharedOctreeNode childNode(int idx) override;
    //OctreeNode& childNodeRef(int idx) override;
    SharedBranchNode parent() override;

    void pushDown(int meshIdx, pm::face_index faceIdx);
    void initLeafNodes();
    //void initEmptyNodes();

    void markIntersections(pm::face_attribute<tg::color3>& faceColor1, pm::face_attribute<tg::color3>& faceColor2) override {
        for (auto child : mChildNodes)
            child->markIntersections(faceColor1, faceColor2);
    }

    void cutPolygons(IntersectionCut& lookup) override {
        for (auto child : mChildNodes)
            child->cutPolygons(lookup);
    }

    BoxAndDistance getNearestBoundingBox(tg::vec3 ray, pos_t origin) override {
        BoxAndDistance currentBox = { AABB(), -1 };
        //SharedOctreeNode nearestNode = std::dynamic_pointer_cast<OctreeNode>(std::make_shared<EmptyNode>(mOctree));
        for (auto child : mChildNodes) {
            if (!child->isNotLeafOrContainsData())
                continue;

            if (!child->intersect(ray, origin))
                continue;

            auto distance = child->distance(origin);
            if (currentBox.dinstance == -1 || distance < currentBox.dinstance) {
                    auto box = child->getNearestBoundingBox(ray, origin);
                    if (currentBox.dinstance == -1 || (box.dinstance < currentBox.dinstance && box.dinstance != -1))
                        currentBox = box;
            }            
        }
        return currentBox;
    }

    void getAllBoundingBoxes(tg::vec3 ray, pos_t origin, std::vector<AABB>& boxes, uint8_t exludeIndex = 255) override {
        for (auto child : mChildNodes) {
            if (!child->intersect(ray, origin) || child->childIndex() == exludeIndex)
                continue;

            child->getAllBoundingBoxes(ray, origin, boxes, exludeIndex);
        }
    }

    NearestFace getNearestFace(tg::vec3 ray, pos_t origin) override {
        NearestFace currentNearest = { -1, pm::face_index(), -1 };
        for (auto child : mChildNodes) {
            if (!child->intersect(ray, origin))
                continue;

            auto nearestFace = child->getNearestFace(ray, origin);
            if (nearestFace.distance == -1)
                continue;

            if (currentNearest.distance == -1 || nearestFace.distance < currentNearest.distance)
                currentNearest = nearestFace;
        }
        return currentNearest;
    }

    void getAllFaces(tg::vec3 ray, pos_t origin, std::vector<pm::face_index>& indices, uint8_t exludeIndex = 255) override {
        for (auto child : mChildNodes) {
            if (!child->intersect(ray, origin) || child->childIndex() == exludeIndex)
                continue;

            child->getAllFaces(ray, origin, indices, exludeIndex);
        }
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
    SharedBranchNode parent() override;
};

class DebugRayInfo {
public:
    std::vector<SubDet> rayPath;
    pos_t octreeVerex;

    //Debug direct approach
    std::vector<AABB> rayBoxesDirect;
    pos_t rayStartDirect;
    pos_t rayEndDirect;

    //Cell approach
    std::vector<pos_t> nexPointsCell;
};
using SharedDebugRayInfo = std::shared_ptr<DebugRayInfo>;


class Octree : public std::enable_shared_from_this<Octree> {

public:
    enum class Options {
        SPLIT_ONE_MESH = 1,
        SPLIT_TWO_MESHES = 2
    };

    scalar_t getNextPower(scalar_t value) {
        scalar_t power = 1;
        while (power < value)
            power *= 2;
        return power;
    }

    AABB getAABBWithNextPowerLen(const AABB& aabb) {
        scalar_t len = aabb.max.x - aabb.min.x;
        scalar_t powerLen = getNextPower(len);
        pos_t max = { aabb.min.x + powerLen, aabb.min.y + powerLen , aabb.min.z + powerLen };
        return AABB(aabb.min, max);
    }

    BoxAndDistance getNearestBoundingBox(tg::vec3 ray, pos_t origin) {
        return mRoot->getNearestBoundingBox(ray, origin);
    }

    void getAllBoundingBoxes(tg::vec3 ray, pos_t origin, std::vector<AABB>& boxes) {
        return mRoot->getAllBoundingBoxes(ray, origin, boxes);
    }

    NearestFace getNearestFace(tg::vec3 ray, pos_t origin) {
        return mRoot->getNearestFace(ray, origin);
    }

    Octree(PlaneMesh* a, PlaneMesh* b, const AABB& aabb) {
        mMeshA = a;
        mMeshB = b;
        AABB powerOf2AABB = getAABBWithNextPowerLen(aabb);
        mRoot = std::make_shared<BranchNode>(powerOf2AABB, this);
        //mRoot->setOctree(shared_from_this());
        mRoot->initLeafNodes();
        mFaceMeshAToNode = a->mesh().faces().make_attribute<std::vector<SharedLeafNode>>();
        mFaceMeshBToNode = b->mesh().faces().make_attribute<std::vector<SharedLeafNode>>();
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

    IntersectionCut cutPolygons() {
        IntersectionCut faceLookUp(mMeshA, mMeshB);
        //Todo
        bool test1 = mMeshA->allFacesAreValidAndNotRemoved();
        bool test2 = mMeshB->allFacesAreValidAndNotRemoved();
        mRoot->cutPolygons(faceLookUp);
        faceLookUp.printTimes();
        return faceLookUp;
    }
    //PlaneMesh& meshA() { return mMeshA; }
    //PlaneMesh& meshB() { return mMeshB; }

    std::string getBoolString(bool b) {
        if (b)
            return "True";
        return "False";
    }
    
    void startDebugView() {
        gv::SharedCameraController cam = gv::CameraController::create();

        /*std::vector<AABB> boxes1;
        std::vector<tg::aabb3> boxes2;
        octree->insertAABB(boxes1);

        for (auto box : boxes1) {
            boxes2.push_back(tg::aabb3(tg::pos3(box.min), tg::pos3(box.max)));
        }*/
        //auto const boxesRender = gv::make_renderable(gv::lines(boxes2).line_width_world(100000));

        auto const mesh1Render = gv::make_renderable(mMeshA->positions());
        auto const mesh2Render = gv::make_renderable(mMeshB->positions());
        
        NearestFace currentNearestFace;
        FaceMarker2 marker(mMeshA, mMeshB);

        gv::interactive([&](auto) {
            auto const mousePos = gv::experimental::interactive_get_mouse_position();
            auto const windowSize = gv::experimental::interactive_get_window_size();

            tg::mat4 projectionMatrix = cam->computeProjMatrix();
            tg::mat4 viewMatrix = cam->computeViewMatrix();

            float x = (2.0f * mousePos.x) / windowSize.width - 1.0f;
            float y = 1.0f - (2.0f * mousePos.y) / windowSize.height;
            tg::vec4 rayClip = tg::vec4(x, y, -1.0, 1.0);
            tg::vec4 rayEye = tg::inverse(projectionMatrix) * rayClip;
            rayEye = tg::vec4(rayEye.x, rayEye.y, -1, 0);
            tg::vec4 rayWorld4 = tg::inverse(viewMatrix) * rayEye;
            tg::vec3 rayWorld = tg::vec3(rayWorld4);
            rayWorld = tg::normalize(rayWorld);

            auto camPos = cam->getPosition();
            auto nearestFace = getNearestFace(rayWorld, pos_t(camPos));
 
            auto face = nearestFace.faceIndex;
            bool faceHaveValidEdges = false;
            bool faceVerticesOnPlane = false;
            bool faceNoDuplicatedVertices = false;
            bool faceHalfEdgesAreValid = false;
            double maxVertexDistance = 0;

            std::vector<pos_t> vertices;
            marker.colorFace(face, nearestFace.meshID);
            if (face.is_valid()) {
                if (nearestFace.meshID == mMeshA->id()) {
                    vertices = mMeshA->getVerticesOfFace(face);
                    //std::cout << "lel 1" << std::endl;
                    faceNoDuplicatedVertices = !mMeshA->duplicatedVerticesInFace(face);
                    faceHaveValidEdges = mMeshA->faceHaveValidEdges(face);
                    faceVerticesOnPlane = mMeshA->allVerticesInFacePlane(face);
                    maxVertexDistance = mMeshA->getGreatestDistanceToBasePlaneFromVertices(face);
                    faceHalfEdgesAreValid = mMeshA->faceHasValidHalfEdges(face);
                }                 
                else {
                    vertices = mMeshB->getVerticesOfFace(face);
                    //std::cout << "lel 2" << std::endl;
                    faceNoDuplicatedVertices = !mMeshB->duplicatedVerticesInFace(face);
                    faceHaveValidEdges = mMeshB->faceHaveValidEdges(face);
                    faceVerticesOnPlane = mMeshB->allVerticesInFacePlane(face);
                    maxVertexDistance = mMeshB->getGreatestDistanceToBasePlaneFromVertices(face);
                    faceHalfEdgesAreValid = mMeshB->faceHasValidHalfEdges(face);
                }                   
            }

            {
                auto view = gv::view(mMeshA->positions(), cam, marker.faceColorsA());
                gv::view(mMeshB->positions(), cam, marker.faceColorsB());
                if(vertices.size() > 0)
                    gv::view(gv::points(vertices).point_size_world(30000));

                if (currentNearestFace.faceIndex != nearestFace.faceIndex) {
                    currentNearestFace = nearestFace;
                    gv::view_clear_accumulation();
                }

            }

            marker.uncolorFace(face, nearestFace.meshID);

            ImGui::Begin("Move");
            if (ImGui::Button("make screenshot"))
                gv::make_screenshot("screenshot.png", 1920, 1080);

            if (ImGui::Button("close viewer"))
                gv::close_viewer();

            if (ImGui::Button("save marked polygons"))
                marker.saveMarkedFacesInFile();

            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
                marker.markFace(face, nearestFace.meshID);

            if (ImGui::IsMouseClicked(ImGuiMouseButton_Right))
                marker.unmarkFace(face, nearestFace.meshID);

            ImGui::Text("Mouse Pos: %f:%f:%f", rayWorld.x, rayWorld.y, rayWorld.z);

            ImGui::Text("Face have valid edges: %s", getBoolString(faceHaveValidEdges));
            ImGui::Text("All vertices on plane: %s", getBoolString(faceVerticesOnPlane));
            ImGui::Text("No duplicated vertices: %s", getBoolString(faceNoDuplicatedVertices));
            ImGui::Text("Halfedges are valid: %s", getBoolString(faceHalfEdgesAreValid));
            ImGui::Text("Max vertex distance: %f", maxVertexDistance);

            for (int i = 0; i < vertices.size(); ++i) {
                ImGui::Text("Pos %i: %f:%f:%f", i, double(vertices[i].x), double(vertices[i].y), double(vertices[i].z));
            }

            ImGui::End();
        });
    }

    SharedLeafNode findLeafNodeMeshA(const pm::face_handle& face) {
        if(mFaceMeshAToNode[face].size() > 0)
            return mFaceMeshAToNode[face][0];
        return SharedLeafNode();
    }

    SharedLeafNode findLeafNodeMeshB(const pm::face_handle& face) {
        if (mFaceMeshBToNode[face].size() > 0)
            return mFaceMeshBToNode[face][0];
        return SharedLeafNode();
    }

    bool checkIfPointInPolygon(pm::face_handle face, PlaneMesh* mesh, SubDet& subDet) {
        for (auto halfEdge : face.halfedges()) {
            TG_ASSERT(!ob::are_parallel(mesh->edge(halfEdge.edge()), mesh->face(face)));
            int8_t sign = ob::classify_vertex(subDet, mesh->edge(halfEdge.edge()));
            if (sign * mesh->halfedge(halfEdge) >= 1)
                return false;
        }
        return true;
    }

    size_t intersectionToNextPoint(const Plane& p1, const Plane& p2, SubDet& subDet, SharedLeafNode node, std::vector<int8_t>& singsMeshA, std::vector<int8_t>& singsMeshB) {
        size_t intersectionCount = 0;
        //Mesh A
        for (int i = 0; i < node->mFacesMeshA.size(); ++i) {
            const Plane& facePlane = mMeshA->face(node->mFacesMeshA[i]);
            auto sign = ob::classify_vertex(subDet, facePlane);
            if (singsMeshA[i] == 2)
                singsMeshA[i] = sign;

            if (sign == singsMeshA[i] || sign == 0)
                continue;

            singsMeshA[i] = sign;
            auto subDetFacePlane = mMeshA->pos(p1, p2, facePlane);
            if (checkIfPointInPolygon(node->mFacesMeshA[i].of(mMeshA->mesh()), mMeshA, subDetFacePlane))
                intersectionCount++;
        }

        //Mesh B
        for (int i = 0; i < node->mFacesMeshB.size(); ++i) {
            const Plane& facePlane = mMeshB->face(node->mFacesMeshB[i]);
            auto sign = ob::classify_vertex(subDet, facePlane);
            if (singsMeshB[i] == 2)
                singsMeshB[i] == sign;

            if (sign == singsMeshB[i] || sign == 0)
                continue;

            singsMeshB[i] = sign;
            auto subDetFacePlane = mMeshA->pos(p1, p2, facePlane);
            if (checkIfPointInPolygon(node->mFacesMeshB[i].of(mMeshB->mesh()), mMeshB, subDetFacePlane))
                intersectionCount++;
        }
        return intersectionCount;
    }

    int castToNextPlanes(const PlanePoint& origin, SharedLeafNode node, const Plane& first, const Plane& second, const Plane& third) {
        std::vector<int8_t> singsMeshA(node->mFacesMeshA.size());
        std::vector<int8_t> singsMeshB(node->mFacesMeshB.size());
        size_t intersectionCount = 0;

        SubDet det = mMeshA->pos(origin.basePlane, origin.edgePlane1, origin.edgePlane2);
        for (int i = 0; i < node->mFacesMeshA.size(); ++i) {
            const Plane& facePlane = mMeshA->face(node->mFacesMeshA[i]);
            auto sign = ob::classify_vertex(det, facePlane);
            singsMeshA[i] = sign == 0 ? 2 : sign;
        }

        for (int i = 0; i < node->mFacesMeshB.size(); ++i) {
            const Plane& facePlane = mMeshB->face(node->mFacesMeshB[i]);
            auto sign = ob::classify_vertex(det, facePlane);
            singsMeshB[i] = sign == 0 ? 2 : sign;
        }

        det = mMeshA->pos(first, origin.edgePlane1, origin.edgePlane2);
        intersectionCount += intersectionToNextPoint(origin.edgePlane1, origin.edgePlane2, det, node, singsMeshA, singsMeshB);

        det = mMeshA->pos(first, second, origin.edgePlane1);
        intersectionCount += intersectionToNextPoint(first, origin.edgePlane1, det, node, singsMeshA, singsMeshB);

        det = mMeshA->pos(first, second, third);
        intersectionCount += intersectionToNextPoint(first, second, det, node, singsMeshA, singsMeshB);

        return intersectionCount;
    }

    void fillRayInfo(const PlanePoint& origin, const Plane& first, const Plane& second, const Plane& third, SharedDebugRayInfo rayInfo) {
        SubDet subDet = mMeshA->pos(origin.basePlane, origin.edgePlane1, origin.edgePlane2);
        rayInfo->rayPath.push_back(subDet);
        subDet = mMeshA->pos(origin.edgePlane1, origin.edgePlane2, first);
        rayInfo->rayPath.push_back(subDet);
        subDet = mMeshA->pos(origin.edgePlane1, first, second);
        rayInfo->rayPath.push_back(subDet);
        subDet = mMeshA->pos(first, second, third);
        rayInfo->rayPath.push_back(subDet);
    }

    bool intersectInInterval(PlaneRay planeRay, const Plane& plane, PlaneInterval& interval) {
        SubDet subDet = mMeshA->pos(planeRay.plane1, planeRay.plane2, plane);

        int8_t sign = ob::classify_vertex(subDet, interval.plane1);
        if (ob::classify_vertex(subDet, interval.plane1) >= 1)
            return false;

        sign = ob::classify_vertex(subDet, interval.plane2);
        if (ob::classify_vertex(subDet, interval.plane2) >= 1)
            return false;
             
        return true;
    }

    bool subDetInCell(const SubDet& subDet, const OctreeNodePlanes& planes) {
        for (auto plane : planes.planes) {
            if (ob::classify_vertex(subDet, plane) > 0)
                return false;
        }
        return true;
    }

    SharedLeafNode getRightCell(const pm::vertex_handle& origin, const PlaneMesh& planeMesh) {
        auto& faceToNode = planeMesh.id() == mMeshA->id() ? mFaceMeshAToNode : mFaceMeshBToNode;
        auto subDet = mMeshA->pos(origin);
        auto test = origin.any_incoming_halfedge().face();
        for (auto node : faceToNode[test]) {
            if (!node->isValid())
                continue;

            auto planes = node->getPlanes();
            if(subDetInCell(subDet, planes))
                return node;
        }
        return SharedLeafNode();
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
    }

    RayCastInfo castRayToNextCornerPoint(const pm::vertex_handle& origin, const PlaneMesh& planeMesh, SharedDebugRayInfo rayInfo = SharedDebugRayInfo()) {
        SharedLeafNode node = getRightCell(origin, planeMesh);
        std::vector<pm::face_index>& FacesMeshA = node->mFacesMeshA;
        std::vector<pm::face_index>& FacesMeshB = node->mFacesMeshB;
        
        const OctreeNodePlanes np = node->getPlanes();

        PlaneRay planeRay = planeMesh.getRayPlanes(origin);
        const Plane& basePlane = planeMesh.getAnyFace(origin);
        TG_ASSERT(!(basePlane == planeRay.plane1) && !(basePlane == planeRay.plane1));

        PlanePolygon nodeSides[6] = {
        { np.xyFront, {np.xzTop, np.yzLeft, np.xzBottom, np.yzRight}},
        { np.xyBack, {np.xzTop, np.yzLeft, np.xzBottom, np.yzRight}},
        { np.xzTop, {np.xyFront, np.yzRight, np.xyBack, np.yzLeft}},
        { np.xzBottom, {np.xyFront, np.yzRight, np.xyBack, np.yzLeft}},
        { np.yzRight, {np.xzTop, np.xyFront, np.xzBottom, np.xyBack}},
        { np.yzLeft, {np.xzTop, np.xyFront, np.xzBottom, np.xyBack}}};

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
                        if (rayInfo) {
                            fillRayInfo({ basePlane , planeRay.plane1, planeRay.plane2 }, side.basePlane, sidePlane, thirdPlane, rayInfo);
                            rayInfo->octreeVerex = point;
                        }                            
                        int intersectionCount = castToNextPlanes({ basePlane , planeRay.plane1, planeRay.plane2 }, node, side.basePlane, sidePlane, thirdPlane);                       
                        return { intersectionCount, point,  std::dynamic_pointer_cast<OctreeNode>(node) };
                    }
                }
            }
        }
        return { -1, {0, 0, 0} };
    }

    int countIntersectionsToOutside(const pm::vertex_handle& origin, const PlaneMesh& planeMesh, SharedDebugRayInfo rayInfo = SharedDebugRayInfo()) {
        auto rayCastInfo = castRayToNextCornerPoint(origin, planeMesh, rayInfo);
        auto curPos = rayCastInfo.currentPos;
        vec_t ray = mRoot->aabb().max - curPos;
        /*auto f = tg::gcd(tg::gcd(tg::abs(ray.x), tg::abs(ray.y)), tg::abs(ray.z));
        if (f > 1)
            ray /= f;*/
        auto ray2 = tg::normalize(tg::vec3(ray));

        std::vector<AABB> boxes;
        if (rayInfo) {
            mRoot->getAllBoundingBoxes(ray2, curPos, rayInfo->rayBoxesDirect);
            rayInfo->rayStartDirect = curPos;
            rayInfo->rayEndDirect = mRoot->aabb().max;
        }
        return 0;
    }

    int castToParentRecursive(SharedBranchNode node, pos_t pos, uint8_t exludeChild, SharedDebugRayInfo rayInfo) {
        vec_t ray = node->aabb().max - pos;
        rayInfo->nexPointsCell.push_back(node->aabb().max);
        node->getAllBoundingBoxes(tg::vec3(ray), pos, rayInfo->rayBoxesDirect, exludeChild);
        if (node->hasParent())
            castToParentRecursive(node->parent(), node->aabb().max, node->childIndex(), rayInfo);
        return -1;
    }

    int countIntersectionsToOutside2(const pm::vertex_handle& origin, const PlaneMesh& planeMesh, SharedDebugRayInfo rayInfo = SharedDebugRayInfo()) {
        auto rayCastInfo = castRayToNextCornerPoint(origin, planeMesh, rayInfo);
        auto curPos = rayCastInfo.currentPos;
        auto node = rayCastInfo.currentNode;
        rayInfo->nexPointsCell.push_back(curPos);
        if (node->hasParent())
            castToParentRecursive(node->parent(), curPos, node->childIndex(), rayInfo);
        return -1;
    }

    void planeToTriangles(const Plane& plane, int sideLen, std::vector<tg::triangle3>& insertVec) {
        tg::vec3 upNormal = tg::normalize(tg::vec3(plane.to_dplane().normal));
        tg::vec3 upNormalFace = upNormal != tg::vec3(0, 0, -1) && upNormal != tg::vec3(0, 0, 1) ? tg::vec3(0, 0, -1) : tg::vec3(0, 1, 0);
        tg::dplane3 dplane = plane.to_dplane();
        tg::vec3 normal = tg::vec3(dplane.normal);
        auto ortho_vec1 = tg::normalize(tg::cross(normal, upNormalFace));
        auto ortho_vec2 = tg::normalize(tg::cross(normal, ortho_vec1));
        auto plane_pos = normal * dplane.dis;
        auto pos_a = tg::pos3(plane_pos + 0.5 * sideLen * ortho_vec1 + 0.5 * sideLen * ortho_vec2);
        auto pos_b = tg::pos3(plane_pos + 0.5 * sideLen * ortho_vec1 - 0.5 * sideLen * ortho_vec2);
        auto pos_c = tg::pos3(plane_pos - 0.5 * sideLen * ortho_vec1 - 0.5 * sideLen * ortho_vec2);
        auto pos_d = tg::pos3(plane_pos - 0.5 * sideLen * ortho_vec1 + 0.5 * sideLen * ortho_vec2);
        insertVec.push_back({ pos_a , pos_b, pos_c });
        insertVec.push_back({ pos_c , pos_d, pos_a });
    }

private:    
    bool mSplitOnlyOneMesh = false;
    friend class OctreeNode;
    friend class BranchNode;
    friend class LeafNode;
    SharedBranchNode mRoot;
    PlaneMesh* mMeshA;
    PlaneMesh* mMeshB;
    int intersectionCounterTMP = 0;
    pm::face_attribute<std::vector<SharedLeafNode>> mFaceMeshAToNode;
    pm::face_attribute<std::vector<SharedLeafNode>> mFaceMeshBToNode;
};