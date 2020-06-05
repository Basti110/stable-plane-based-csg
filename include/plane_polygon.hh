#pragma once
#include <vector>
#include <iostream>
#include <typed-geometry/tg.hh>
#include <geometry.hh>
#include <iostream>
#include <polymesh/pm.hh>

using geometry128 = ob::geometry<48, 49>;//ob::geometry<32, 21>;
using plane128 = geometry128::plane_t;
using Plane = geometry128::plane_t;
using scalar_t = geometry128::pos_scalar_t;
//using type_t = geometry128::pos_scalar_t;
using pos_t = geometry128::pos_t;
using vec_t = tg::vec<3, scalar_t>;
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
    PlaneMesh() : mPositions(mMesh), mEdges(mMesh), mFaces(mMesh), mHalfEdges(mMesh){
        mID = instances;
        instances++;
    }

    ///Only Triangle TODO: Polygons
    PlaneMesh(const pm::Mesh& m, const pm::vertex_attribute<pos_t>& pos) : mPositions(mMesh), mEdges(mMesh), mFaces(mMesh), mHalfEdges(mMesh) {
        mID = instances;
        instances++;

        mMesh.copy_from(m);
        mPositions.copy_from(pos);
        init_mesh();
    }

    PlaneMesh(const pm::Mesh& m, const pm::vertex_attribute<tg::pos3>& pos, int scale) : mPositions(mMesh), mEdges(mMesh), mFaces(mMesh), mHalfEdges(mMesh) {
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

    pm::face_handle insertTriangle(tg::triangle<3, scalar_t> triangle) {
        return insertPolygon(triangle.pos0, triangle.pos1, triangle.pos2);
    }

    //void only test
    pm::face_handle insertPolygon(pos_t x, pos_t y, pos_t z) {      
        const auto vh0 = mMesh.vertices().add();
        const auto vh1 = mMesh.vertices().add();
        const auto vh2 = mMesh.vertices().add();
        //pm::load("test", mMesh, mPositions);

        mPositions[vh0] = x;
        mPositions[vh1] = y;
        mPositions[vh2] = z;

        pm::face_handle face = mMesh.faces().add(vh0, vh1, vh2);
        mFaces[face] = Plane::from_points(x, y, z);
        generatePlanes(face);
        return face;
    }

    //void only test
    pm::face_handle insertPolygon(std::vector<pos_t> polygon) {
        TG_ASSERT(polygon.size() >= 3);
        std::vector<pm::vertex_handle> positions(polygon.size());
        for (int i = 0; i < polygon.size(); ++i) {
            positions[i] = mMesh.vertices().add();
            mPositions[positions[i]] = polygon[i];
        }

        pm::face_handle face = mMesh.faces().add(positions);
        mFaces[face] = Plane::from_points(mPositions[positions[0]], mPositions[positions[1]], mPositions[positions[2]]);
        generatePlanes(face);
        return face;
    }

    void generatePlanes(pm::face_handle& face) {
        pm::face_edge_ring edgeRing = face.edges();
        TG_ASSERT(edgeRing.count() >= 3);

        auto randomEdge = face.any_halfedge();
        auto x = mPositions[randomEdge.vertex_from()];
        auto y = mPositions[randomEdge.vertex_to()];
        auto z = mPositions[randomEdge.next().vertex_to()];

        auto dir1 = y - x;
        auto dir2 = z - x;

        auto normalDir = tg::cross(dir1, dir2);
        vec_t approxNormalDir = vec_t(tg::normalize(tg::f64vec3(normalDir)) * 100);




        for (auto it = edgeRing.begin(); it != edgeRing.end(); ++it) {
            pm::edge_handle edgeHandle = it.handle.edge();
            auto from = mPositions[it.handle.vertex_from()];
            auto to = mPositions[it.handle.vertex_to()];
            auto anotherPos = from + approxNormalDir;
            if (mEdges[edgeHandle] == mEdges.get_default_value()) {
                mEdges[edgeHandle] = Plane::from_points(from, to, anotherPos);
                //TG_ASSERT(ob::signed_distance(mEdges[edgeHandle], posInFace) < 0); //TODO 
                TG_ASSERT(ob::signed_distance(mEdges[edgeHandle], mPositions[it.handle.next().vertex_to()]) < 0); //TODO: Remove
                //TG_ASSERT(ob::classify_vertex(pos(it.handle.next().vertex_to()), edge(edgeHandle)) < 0);
                mHalfEdges[it.handle] = 1;
            }
            else {            
                if (useHalfedges) {
                    if (ob::signed_distance(mEdges[edgeHandle], mPositions[it.handle.next().vertex_to()]) < 0)
                        mHalfEdges[it.handle] = 1;
                    else
                        mHalfEdges[it.handle] = -1;
                }
            }
        }
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

    int8_t signDistanceToBasePlane(const pm::face_handle& polygon, const pm::vertex_handle& point) const {
        return ob::classify_vertex(pos(point), mFaces[polygon]);
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

    pm::halfedge_attribute<int8_t>& halfEdges() {
        return mHalfEdges;
    }

    const pm::halfedge_attribute<int8_t>& halfEdges() const {
        return mHalfEdges;
    }

    Plane& face(const pm::face_handle& face) {
        return mFaces[face];
    }

    Plane& edge(const pm::edge_handle& edge) {
        return mEdges[edge];
    }

    int8_t halfedge(const pm::halfedge_handle& edge) {
        return mHalfEdges[edge];
    }

    const Plane& face(const pm::face_handle& face) const {
        return mFaces[face];
    }

    const Plane& edge(const pm::edge_handle& edge) const{
        return mEdges[edge];
    }

    const int8_t halfedge(const pm::halfedge_handle& edge) const {
        return mHalfEdges[edge];
    }

    SubDet pos(const pm::vertex_handle& vertex) const {
        auto he1 = vertex.any_incoming_halfedge();
        auto he2 = he1.next();
        auto face = he1.face();     
        return pos(he1, he2, face);
    }

    SubDet pos(pm::halfedge_handle he1, pm::halfedge_handle he2, pm::face_handle face) const {
        TG_ASSERT(face.is_valid() && he1.is_valid() && he2.is_valid());
        return pos(mFaces[face], mEdges[he1.edge()], mEdges[he2.edge()]);
    }

    SubDet pos(pm::halfedge_handle he, pm::face_handle face1, pm::face_handle face2) const {
        TG_ASSERT(he.is_valid() && face1.is_valid() && face2.is_valid());
        return pos(mFaces[face1], mFaces[face2], mEdges[he.edge()]);
    }

    static SubDet pos(const plane128 &p1, const plane128 &p2, const plane128 &p3) {
        SubDet subDet;
        ob::compute_subdeterminants(p1, p2, p3, subDet);
        return subDet;
    }

    pos_t posInt(const pm::vertex_handle& vertex) const {
        return mPositions[vertex];
    }

    pm::halfedge_handle findEdge(const pm::vertex_handle& from, const pm::vertex_handle& to) const {
        for (pm::halfedge_handle edge : from.outgoing_halfedges()) {
            if (edge.vertex_to() == to) {
                return edge;
            }
        }
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

    Plane& getPlane(const pm::edge_handle edge) {
        return mEdges[edge];
    }

    const Plane& getPlane(const pm::edge_handle& edge) const {
        return mEdges[edge];
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

            //TODO: check if point lie on same lane 
            auto pos1 = mPositions[vh0];
            auto pos2 = mPositions[vh1];
            auto pos3 = mPositions[vh2];

            mFaces[f] = Plane::from_points(pos1, pos2, pos3);
            generatePlanes(f);

            /*auto dir1 = pos2 - pos1;
            auto dir2 = pos3 - pos1;

            //TODO: Assert dir > abs((1, 1, 1))
            pos_t posInFace = pos1 + vec_t((dir1 / 4) + (dir2 / 4));

            auto normalDir = tg::cross(dir1, dir2);            
            vec_t approxNormalDir = vec_t(tg::normalize(tg::f64vec3(normalDir)) * 100); //Note: Not to far, max len resolution

            pm::face_edge_ring edgeRing = f.edges();
            TG_ASSERT(edgeRing.count() >= 3);


            //TODO: extra function
            for (auto it = edgeRing.begin(); it != edgeRing.end(); ++it) {
                pm::edge_handle edgeHandle = it.handle.edge();
                auto from = mPositions[it.handle.vertex_from()];
                auto to = mPositions[it.handle.vertex_to()];
                auto anotherPos = from + approxNormalDir; 
                if (mEdges[edgeHandle] == mEdges.get_default_value()) {
                    mEdges[edgeHandle] = Plane::from_points(from, to, anotherPos);
                    //TG_ASSERT(ob::signed_distance(mEdges[edgeHandle], posInFace) < 0); //TODO 
                    TG_ASSERT(ob::signed_distance(mEdges[edgeHandle], mPositions[it.handle.next().vertex_to()]) < 0);
                    mHalfEdges[it.handle] = 1;
                }                
                else {
                    if (useHalfedges) {
                        if(ob::signed_distance(mEdges[edgeHandle], mPositions[it.handle.next().vertex_to()]) < 0)
                            mHalfEdges[it.handle] = 1;
                        else
                            mHalfEdges[it.handle] = -1;
                    }
                }
            }*/
        }
        std::cout << "normal: " << debugNormalEdges << std::endl;
        std::cout << "inverted: " << debugInvertedEdges << std::endl;
    }

private: 
    int debugNormalEdges = 0;
    int debugInvertedEdges = 0;
    pm::Mesh mMesh;
    VertexAttribute mPositions;
    pm::face_attribute<Plane> mFaces;
    pm::edge_attribute<Plane> mEdges;
    pm::halfedge_attribute<int8_t> mHalfEdges;
    bool useHalfedges = true;
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