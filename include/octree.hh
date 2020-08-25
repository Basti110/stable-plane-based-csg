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

struct OctreeNodePlanes {
    Plane xyFront;
    Plane xyBack;
    Plane xzTop;
    Plane xzBottom;
    Plane yzRight;
    Plane yzLeft;
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
    virtual void cutPolygons(IntersectionCut& lookup) {};
    virtual void markIntersections(pm::face_attribute<tg::color3>& faceColor1, pm::face_attribute<tg::color3>& faceColor2) {}
    virtual BoxAndDistance getNearestBoundingBox(tg::vec3 ray, pos_t origin) { return { AABB(), -1 }; }
    virtual NearestFace getNearestFace(tg::vec3 ray, pos_t origin) { return { -1, pm::face_index(), -1 }; }
    virtual void getAllBoundingBoxes(tg::vec3 ray, pos_t origin, std::vector<AABB>& boxes) { }
    bool isInTree();
    bool hasParent();
    bool polygonInAABB(int meshIdx, pm::face_index faceIdx);

    OctreeNodePlanes getPlanes() {
        TG_ASSERT(mAABB != AABB());
        OctreeNodePlanes planes;

        Plane xyFront;
        Plane xyBack;
        Plane xzTop;
        Plane xzBottom;
        Plane yzRight;
        Plane yzLeft;

        planes.xyFront = Plane::from_pos_normal(mAABB.max, { 0, 0, 1 });
        planes.xyBack = Plane::from_pos_normal(mAABB.min, { 0, 0, -1 });
        planes.xzTop = Plane::from_pos_normal(mAABB.max, { 0, 1, 0 });
        planes.xzBottom = Plane::from_pos_normal(mAABB.min, { 0, -1, 0 });
        planes.yzRight = Plane::from_pos_normal(mAABB.max, { 1, 0, 0 });
        planes.yzLeft = Plane::from_pos_normal(mAABB.min, { -1, 0, 0 });
        return planes;
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
    SharedOctreeNode mParentNode;
    AABB mAABB;
    Octree* const mOctree;
};

class LeafNode : public OctreeNode, public std::enable_shared_from_this<LeafNode> {
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
    void markIntersections(pm::face_attribute<tg::color3>& faceColor1, pm::face_attribute<tg::color3>& faceColor2) override;
    void cutPolygons(IntersectionCut& lookup) override;
    BoxAndDistance getNearestBoundingBox(tg::vec3 ray, pos_t origin) override {
        return { mAABB, distance(origin) };
    }

    bool isNotLeafOrContainsData() override {
        return (mFacesMeshA.size() > 0 || mFacesMeshB.size() > 0);
    }

    void getAllBoundingBoxes(tg::vec3 ray, pos_t origin, std::vector<AABB>& boxes) { 
        if (mFacesMeshA.size() > 0 || mFacesMeshB.size() > 0)
            boxes.push_back(mAABB);
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

    void getAllBoundingBoxes(tg::vec3 ray, pos_t origin, std::vector<AABB>& boxes) {
        for (auto child : mChildNodes) {
            if (!child->intersect(ray, origin))
                continue;

            child->getAllBoundingBoxes(ray, origin, boxes);
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
        mFaceMeshAToNode = a->mesh().faces().make_attribute<SharedLeafNode>();
        mFaceMeshBToNode = b->mesh().faces().make_attribute<SharedLeafNode>();
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
        return mFaceMeshAToNode[face];
    }

    SharedLeafNode findLeafNodeMeshB(const pm::face_handle& face) {
        return mFaceMeshBToNode[face];
    }

    void CastRayToNextCornerPoint(const pm::face_handle& face, PlaneMesh& planeMesh) {
        SharedLeafNode node = planeMesh.id() == mMeshA->id() ? findLeafNodeMeshA(face) : findLeafNodeMeshB(face);
        std::vector<pm::face_index>& FacesMeshA = node->mFacesMeshA;
        std::vector<pm::face_index>& FacesMeshB = node->mFacesMeshB;
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
    pm::face_attribute<SharedLeafNode> mFaceMeshAToNode;
    pm::face_attribute<SharedLeafNode> mFaceMeshBToNode;
};