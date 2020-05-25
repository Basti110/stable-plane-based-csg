#pragma once
#include <vector>
#include <iostream>
#include <typed-geometry/tg.hh>
#include <geometry.hh>
#include <polymesh/pm.hh>

using geometry128 = ob::geometry<48, 49>;//ob::geometry<32, 21>;
using plane128 = geometry128::plane_t;
using Plane = geometry128::plane_t;
using scalar_t = geometry128::pos_scalar_t;
//using type_t = geometry128::pos_scalar_t;
using pos_t = geometry128::pos_t;
using dir_t = tg::dir<3, scalar_t>;
using SubDet = ob::subdeterminants<geometry128>;
using VertexAttribute = pm::vertex_attribute<geometry128::pos_t>;
//using SharedPlanePolygon = std::shared_ptr<PlanePolygon>;

struct PlanePolygon {
    pm::face_handle face;
    VertexAttribute& positions;
    pm::face_attribute<Plane>& facePlanes;
    pm::edge_attribute<Plane>& edgePlanes;
};

class PlaneMesh {
public:
    PlaneMesh() : mPositions(mMesh), mEdges(mMesh), mFaces(mMesh) {
        mID = instances;
        instances++;
    }

    ///Only Triangle TODO: Polygons
    PlaneMesh(const pm::Mesh& m, const pm::vertex_attribute<pos_t>& pos) : mPositions(mMesh), mEdges(mMesh), mFaces(mMesh) {
        mID = instances;
        instances++;

        mMesh.copy_from(m);
        mPositions.copy_from(pos);
        init_mesh();
    }

    PlaneMesh(const pm::Mesh& m, const pm::vertex_attribute<tg::pos3>& pos, int scale) : mPositions(mMesh), mEdges(mMesh), mFaces(mMesh) {
        mID = instances;
        instances++;

        mMesh.copy_from(m);
        auto vertices1 = mMesh.all_vertices();
        auto vertices2 = m.all_vertices();
        TG_ASSERT(vertices1.size() == vertices2.size());
        for (int i = 0; i < vertices1.size(); ++i) {
            auto dpos = pos[vertices2[i]] * scale;
            mPositions[vertices1[i]] = pos_t(dpos);
        }
        init_mesh();

    }

    pm::all_face_collection allFaces() {
        return mMesh.all_faces();
    }

    //void only test
    pm::face_handle insertPolygon(pos_t x, pos_t y, pos_t z) {
        auto normalFace = tg::cross(y - x, y - z);
        
        const auto vh0 = mMesh.vertices().add();
        const auto vh1 = mMesh.vertices().add();
        const auto vh2 = mMesh.vertices().add();
        //pm::load("test", mMesh, mPositions);

        mPositions[vh0] = x;
        mPositions[vh1] = y;
        mPositions[vh2] = z;

        pm::face_handle face = mMesh.faces().add(vh0, vh1, vh2);
        mFaces[face] = Plane::from_points(x, y, z);
        
        pm::face_edge_ring edgeRing = face.edges();        
        TG_ASSERT(edgeRing.count() == 3);
        
        for (auto it = edgeRing.begin(); it != edgeRing.end(); ++it) {
            pm::edge_handle edgeHandle = it.handle.edge();
            auto from = mPositions[it.handle.vertex_from()];
            auto to = mPositions[it.handle.vertex_to()];
            auto normalEdge = tg::cross(to - from, normalFace);
            mEdges[edgeHandle] = Plane::from_pos_normal(from, normalEdge);
        }
        return face;
    }

    void planesTriangles(int sideLen, std::vector<tg::triangle3>& insertVec) {
        auto faces = mMesh.faces();
        for (auto face : faces) {
            tg::dplane3 dplane = mFaces[face].to_dplane();
            tg::vec3 upNormal = tg::normalize(tg::vec3(dplane.normal));
            tg::vec3 upNormalFace = upNormal != tg::vec3(0, 0, -1) ? tg::vec3(0, 0, -1) : tg::vec3(0, 1, 0);
            auto half_edges = face.halfedges();
            planeTriangle(mFaces[face], 15, upNormalFace, insertVec);
            for (auto he : half_edges) {
                planeTriangle(mEdges[he.edge()], sideLen, upNormal, insertVec);
            }
        }
    }

    void planeTriangle(Plane& plane, int sideLen, tg::vec3& upNormal, std::vector<tg::triangle3>& insertVec) {
        tg::dplane3 dplane = plane.to_dplane();
        tg::vec3 normal = tg::vec3(dplane.normal);
        auto ortho_vec1 = tg::normalize(tg::cross(normal, upNormal));
        auto ortho_vec2 = tg::normalize(tg::cross(normal, ortho_vec1));
        auto plane_pos = normal * dplane.dis;
        auto pos_a = tg::pos3(plane_pos + 0.5 * sideLen * ortho_vec1 + 0.5 * sideLen * ortho_vec2);
        auto pos_b = tg::pos3(plane_pos + 0.5 * sideLen * ortho_vec1 - 0.5 * sideLen * ortho_vec2);
        auto pos_c = tg::pos3(plane_pos - 0.5 * sideLen * ortho_vec1 - 0.5 * sideLen * ortho_vec2);
        auto pos_d = tg::pos3(plane_pos - 0.5 * sideLen * ortho_vec1 + 0.5 * sideLen * ortho_vec2);
        insertVec.push_back({ pos_a , pos_b, pos_c });
        insertVec.push_back({ pos_c , pos_d, pos_a });
    }

    PlanePolygon planePolygon(const pm::face_index& face) {
        return {face.of(mMesh), mPositions, mFaces, mEdges };
    }

    PlanePolygon planePolygon(const pm::face_handle& face) {
        return { face, mPositions, mFaces, mEdges };
    }

    uint8_t signDistance(const pm::face_handle& polygon, const pm::vertex_handle& point) const {
        return 0;
    }

    pm::vertex_attribute<geometry128::pos_t>& positions() {
        return mPositions;
    }

    pm::face_attribute<Plane>& faces() {
        return mFaces;
    }

    pm::edge_attribute<Plane>& edges() {
        return mEdges;
    }

    pm::Mesh& mesh() {
        return mMesh;
    }

    Plane& getPlane(const pm::face_handle& face) {
        return mFaces[face];
    }

    const Plane& getPlane(const pm::face_handle& face) const {
        return mFaces[face];
    }

    const pm::Mesh& mesh() const {
        return mMesh;
    }

    int id() {
        return mID;
    }

private: 
    void init_mesh() {
        for (auto f : mMesh.faces()) {
            auto h = f.any_halfedge();

            const auto vh0 = h.vertex_from();
            const auto vh1 = h.vertex_to();
            const auto vh2 = h.next().vertex_to();

            auto pos1 = mPositions[vh0];
            auto pos2 = mPositions[vh1];
            auto pos3 = mPositions[vh2];

            auto normalFace = tg::cross(pos2 - pos1, pos2 - pos3);
            mFaces[f] = Plane::from_points(pos1, pos2, pos3);
            dir_t normal_dir = dir_t(tg::normalize(tg::f64vec3(normalFace) * 100));

            pm::face_edge_ring edgeRing = f.edges();
            TG_ASSERT(edgeRing.count() == 3);

            for (auto it = edgeRing.begin(); it != edgeRing.end(); ++it) {
                pm::edge_handle edgeHandle = it.handle.edge();
                auto from = mPositions[it.handle.vertex_from()];
                auto to = mPositions[it.handle.vertex_to()];
                auto random_pos = from + normal_dir;
                mEdges[edgeHandle] = Plane::from_points(from, to, random_pos);
            }
        }
    }

private: 
    pm::Mesh mMesh;
    VertexAttribute mPositions;
    pm::face_attribute<Plane> mFaces;
    pm::edge_attribute<Plane> mEdges;
    bool useHalfedges;
    int mID;
    static int instances;
};











































/*class PlaneMesh {
    using geometry128 = ob::geometry<32, 21>;
    using plane128 = geometry128::plane_t;
    using Plane = geometry128::plane_t;
    using SubDet = ob::subdeterminants<geometry128>;
    using VertexAttribute = pm::vertex_attribute<tg::pos3>;
    using SharedPlanePolygon = std::shared_ptr<PlanePolygon>;
    
public:
    PlaneMesh() {
        mID = instances;
        instances++;
    }
    int id() { return mID; }
    void create_polygon(tg::ipos3 x, tg::ipos3 y, tg::ipos3 z);
private: 
    static int instances;
    int mID;
    std::vector<SharedPlanePolygon> mPolygons;
    std::vector<Plane> mPlanes;
    std::vector<int> mFacePlaneIdx;
};

class PlanePolygon {
    using geometry128 = ob::geometry<32, 21>;
    using plane128 = geometry128::plane_t;
    using Plane = geometry128::plane_t;
    using SubDet = ob::subdeterminants<geometry128>;
    using VertexAttribute = pm::vertex_attribute<tg::pos3>;

public:
    PlanePolygon(tg::ipos3 x, tg::ipos3 y, tg::ipos3 z) {
        auto normal = tg::cross(y - x, y - z);
        auto normal_a = tg::cross(y - x, normal);
        auto normal_b = tg::cross(z - y, normal);
        auto normal_c = tg::cross(x - z, normal);

        mSubDeterminants.resize(3);
        mPlanes.resize(3);

        mPlaneFace = Plane::from_points(x, y, z);
        mPlanes[0] = Plane::from_pos_normal(x, normal_a);
        mPlanes[1] = Plane::from_pos_normal(y, normal_b);
        mPlanes[2] = Plane::from_pos_normal(z, normal_c);

        mSubDeterminants[0] = { ob::i128(0), ob::i128(0), ob::i128(0), ob::i128(0) };
        mSubDeterminants[1] = { ob::i128(0), ob::i128(0), ob::i128(0), ob::i128(0) };
        mSubDeterminants[2] = { ob::i128(0), ob::i128(0), ob::i128(0), ob::i128(0) };
    }

    PlanePolygon(std::vector<tg::ipos3>& positions) {
        //TODO ERRORS
        if (positions.size() < 3)
            std::cout << "ERROR: Need min. 3 values" << std::endl;

        auto normal = tg::cross(positions[1] - positions[0], positions[1] - positions[2]);

        mSubDeterminants.resize(positions.size());
        mPlanes.resize(positions.size());

        mPlaneFace = Plane::from_points(positions[0], positions[1], positions[2]);
        for (int i = 0; i < positions.size(); i++) {
            int idx = (i + 1) % positions.size();
            auto plane_normal = tg::cross(positions[idx] - positions[i], normal);
            mPlanes[i] = Plane::from_pos_normal(positions[i], plane_normal);
            mSubDeterminants[i] = { ob::i128(0), ob::i128(0), ob::i128(0), ob::i128(0) };
        }
    }

    void compute_subdetermiants() {
        for (int i = 0; i < mPlanes.size(); i++) {
            int idx = (i + 1) % mPlanes.size();
            ob::compute_subdeterminants(mPlaneFace, mPlanes[i], mPlanes[idx], mSubDeterminants[i]);
        }
        mSubDetPreComputed = true;
    }

    void getPolygone(VertexAttribute& pos, pm::Mesh& m) {
        
        if (!mSubDetPreComputed) {
            compute_subdetermiants();
        }

        std::vector<pm::vertex_handle> vertices;
        vertices.resize(mSubDeterminants.size());
        for (int i = 0; i < mSubDeterminants.size(); i++) {
            vertices[i] = m.vertices().add();
            auto test = tg::pos3(ob::to_position(mSubDeterminants[i]));
            pos[vertices[i]] = tg::pos3(ob::to_position(mSubDeterminants[i]));
        }

        m.faces().add(vertices);
    }

    std::vector<tg::triangle3> get_planes_triangles(int side_len) {
        std::vector<tg::triangle3> triangles;
        triangles.reserve(8);
        tg::dplane3 dplane = mPlaneFace.to_dplane();
        tg::vec3 up_normal = tg::normalize(tg::vec3(dplane.normal));
        tg::vec3 up_normal_face = up_normal != tg::vec3(0, 0, -1) ? tg::vec3(0, 0, -1) : tg::vec3(0, 1, 0);
        
        get_plane_triangle(mPlaneFace, side_len, up_normal_face, triangles);
        for (Plane& plane : mPlanes) {
            get_plane_triangle(plane, side_len, up_normal, triangles);
        }
        return triangles;
    }

    void get_plane_triangle(Plane& plane, int side_len, tg::vec3& up_normal, std::vector<tg::triangle3>& insert_vec) {
        tg::dplane3 dplane = plane.to_dplane();
        tg::vec3 normal = tg::vec3(dplane.normal);
        auto ortho_vec1 = tg::normalize(tg::cross(normal, up_normal));
        auto ortho_vec2 = tg::normalize(tg::cross(normal, ortho_vec1));
        auto plane_pos = normal * dplane.dis;
        auto pos_a = tg::pos3(plane_pos + 0.5 * side_len * ortho_vec1 + 0.5 * side_len * ortho_vec2);
        auto pos_b = tg::pos3(plane_pos + 0.5 * side_len * ortho_vec1 - 0.5 * side_len * ortho_vec2);
        auto pos_c = tg::pos3(plane_pos - 0.5 * side_len * ortho_vec1 - 0.5 * side_len * ortho_vec2);
        auto pos_d = tg::pos3(plane_pos - 0.5 * side_len * ortho_vec1 + 0.5 * side_len * ortho_vec2);
        insert_vec.push_back({ pos_a , pos_b, pos_c });
        insert_vec.push_back({ pos_c , pos_d, pos_a });
    }

private:
    bool mSubDetPreComputed = false;
    Plane mPlaneFace;
    std::vector<Plane> mPlanes;
    std::vector<SubDet> mSubDeterminants;

};*/