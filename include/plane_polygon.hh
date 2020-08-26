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


struct PlanePolygon {
    Plane& basePlane;
    std::vector<Plane> edgePlanes;
};

struct PlaneInterval {
    const Plane& plane1;
    const Plane& plane2;
};

struct PlanePoint {
    const Plane& basePlane;
    const Plane& edgePlane1;
    const Plane& edgePlane2;
};

struct PlaneRay {
    const Plane& plane1;
    const Plane& plane2;
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
                //TG_ASSERT(ob::signed_distance(mEdges[edgeHandle], mPositions[it.handle.next().vertex_to()]) < 0); //TODO: Remove
                //TG_ASSERT(ob::classify_vertex(pos(it.handle.next().vertex_to()), mEdges[edgeHandle]) == -1);
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

        for (auto it = edgeRing.begin(); it != edgeRing.end(); ++it) {
            pm::edge_handle edgeHandle = it.handle.edge();
            //TG_ASSERT(ob::classify_vertex(pos(it.handle.next().vertex_to()), mEdges[edgeHandle]) == -1);
        }
    }

    //#############################################################################
    //#                          Validation on Mesh                               #
    //#############################################################################

    bool allHalfEdgesAreValid() {
        for (auto h : mMesh.all_halfedges()) {
            if (h.is_invalid())
                return false;
        }
        return true;
    }

    bool allFacesAreValid() {
        for (auto f : mMesh.all_faces()) {
            if (f.is_removed())
                continue;

            if (!faceHaveValidEdges(f))
                return false;
        }      
        return true;
    }

    bool noDuplicatedVerticesInFaces(pm::face_attribute<bool>& mask) {  
        bool noDuplicates = true;
        for (pm::face_handle& f : mMesh.faces()) {
            if (duplicatedVerticesInFace(f)) {
                noDuplicates = false;
                mask[f] = true;
            }
        }
        return noDuplicates;
    }

    bool noDuplicatedVerticesInFaces() {
        for (auto f : mMesh.faces()) {
            if (duplicatedVerticesInFace(f))
                return false;         
        }
        return true;
    }

    bool allFacesHaveEdges() {
        for (auto f : mMesh.all_faces()) {
            if (f.is_removed())
                continue;

            auto h = f.any_halfedge();
            pm::halfedge_handle hTmp;
            int count = 0;
            do {
                count++;
                hTmp = h.next();
            } while (hTmp != h && count <= f.halfedges().count());
            if (h != hTmp)
                return false;
        }
        return true;
    }

    bool allFacesAreValidAndNotRemoved() {
        for (auto f : mMesh.all_faces()) {
            if (f.is_invalid() || f.is_removed())
                return false;
        }
        return true;
    }

    bool existsEdgesWithoutFace() {
        for (auto edge : mMesh.edges()) {
            if (!edge.faceA().is_valid() && !edge.faceB().is_valid())
                return true;
        }
        return false;
    }

    //#############################################################################
    //#                          Validation on Face                               #
    //#############################################################################
    bool allFacesHaveValidHalfEdges() {
        for (pm::face_handle& f : mMesh.faces()) {
            if (!faceHasValidHalfEdges(f))
                return false;
        }
        return true;
    }

    bool faceHasValidHalfEdges(pm::face_index f) {
        return faceHasValidHalfEdges(f.of(mMesh));
    }

    bool faceHasValidHalfEdges(pm::face_handle face) {
        for (pm::vertex_handle v : face.vertices()) {
            auto sub = pos(v);
            for (pm::halfedge_handle he : face.halfedges()) {
                auto plane = mEdges[he.edge()];
                int8_t sign = ob::classify_vertex(sub, plane);
                sign *= mHalfEdges[he];
                if (sign > 0)
                    return false;
            }
        }
        return true;
    }

    bool duplicatedVerticesInFace(pm::face_index f) {
        return duplicatedVerticesInFace(f.of(mMesh));
    }

    bool duplicatedVerticesInFace(pm::face_handle f) {
        auto vertices = f.vertices().to_vector([](pm::vertex_handle i) { return i; });
        for (auto i = 0; i < vertices.size(); ++i) {
            for (auto j = i + 1; j < vertices.size(); ++j) {
                if (i == j)
                    continue;

                if (mPositions[vertices[i]] == mPositions[vertices[j]]) {
                    return true;
                }
            }
        }
        return false;
    }

    bool faceHaveValidEdges(pm::face_index f) {
        return faceHaveValidEdges(f.of(mMesh));
    }

    bool faceHaveValidEdges(pm::face_handle f) {
        for (auto h : f.halfedges()) {
            if (h.is_invalid())
                return false;
        }
        return true;
    }

    bool allVerticesInFacePlane(pm::face_index f) {
        return allVerticesInFacePlane(f.of(mMesh));
    }

    bool allVerticesInFacePlane(pm::face_handle f) {
        for (auto v : f.vertices()) {
            auto dis = signDistanceToBasePlane(f, v);
            if (dis != 0)
                return false;
        }
        return true;
    }

    double getGreatestDistanceToBasePlaneFromVertices(pm::face_index f) {
        return getGreatestDistanceToBasePlaneFromVertices(f.of(mMesh));
    }

    double getGreatestDistanceToBasePlaneFromVertices(pm::face_handle f) {
        double dis = 0;
        for (auto v : f.vertices()) {
            double disTmp = double(ob::signed_distance(mFaces[f], mPositions[v]));
            if (disTmp > dis)
                dis = disTmp;
        }
        return dis;
    }

    //#############################################################################

    double rayHitPolygon(pm::face_index face, tg::vec3 rayDir, pos_t rayOrigin) {
        return rayHitPolygon(face.of(mMesh), rayDir, rayOrigin);
    }

    double rayHitPolygon(pm::face_handle face, tg::vec3 rayDir, pos_t rayOrigin) {
        if (face.is_invalid() || face.is_removed())
            return -1;

        auto disOption = ob::intersection_distance<geometry128>(mFaces[face], rayOrigin, rayDir);
        if (disOption) {
            TG_ASSERT(disOption.value() >= -1);
            for (pm::halfedge_handle& halfedge : face.halfedges()) {
                //std::cout << (tg::dvec3(rayDir) * disOption.value()).x << ":" << (tg::dvec3(rayDir) * disOption.value()).y << ":" << (tg::dvec3(rayDir) * disOption.value()).z << std::endl;
                pos_t pos = rayOrigin + vec_t(rayDir * disOption.value());
                auto signedDistance = ob::signed_distance(mEdges[halfedge.edge()], pos);
                int8_t sign = signedDistance > 0 ? 1 : -1;
                TG_ASSERT(mHalfEdges[halfedge] != 0);
                if (sign * mHalfEdges[halfedge] == 1) {
                    return -1;
                }
            }
            return disOption.value();
        }
        return -1;
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

    void checkAndComputePositions() {
        //mMesh.polymesh::Mesh::compactify();
        for (auto vertex : mMesh.vertices()) {
            if (mPositions[vertex] == pos_t(0, 0, 0)) {
                
                auto h0 = vertex.any_incoming_halfedge(); 
                auto h1 = h0.opposite();
                auto face1 = h0.face();
                auto face2 = h1.face();

                TG_ASSERT(face1.is_valid() || face2.is_valid());

                auto hIn = face1.is_valid() ? h0 : h1;
                auto face = face1.is_valid() ? face1 : face2;
                //auto hOut = hIn.next();

                Plane plane1 = mEdges[hIn.edge()];
                Plane plane2;
                int count = vertex.outgoing_halfedges().count();
                for (auto h : vertex.outgoing_halfedges()) {
                    plane2 = mEdges[h];
                    if (!ob::are_parallel(plane1, plane2))
                        break;
                }
                              
             
                Plane plane3 = mFaces[face];
                pos_t pos = pos_t(ob::compute_intersection(plane1, plane2, plane3));
                mPositions[vertex] = pos;
            }
        }
    }

    std::vector<pos_t> getVerticesOfFace(const pm::face_index& face) {
        return getVerticesOfFace(face.of(mMesh));
    }

    std::vector<pos_t> getVerticesOfFace(const pm::face_handle& face) {
        std::vector<pos_t> vertices;
        vertices.reserve(face.vertices().count());
        for (auto vertex : face.vertices())
            vertices.push_back(mPositions[vertex]);
        return vertices;
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

    void setFace(const pm::face_handle& face, Plane& plane) {
        mFaces[face] = plane;
    }

    void setEdge(const pm::edge_handle& edge, Plane& plane) {
        mEdges[edge] = plane;
    }

    void setHalfedge(const pm::halfedge_handle& edge, int8_t sign) {
        mHalfEdges[edge] = sign;
    }

    Plane& face(const pm::face_handle& face) {
        return mFaces[face];
    }

    Plane& edge(const pm::edge_handle& edge) {
        return mEdges[edge];
    }

    Plane& edge(const pm::halfedge_handle& edge) {
        return mEdges[edge.edge()];
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

    const Plane& face(const pm::face_index& face) const {
        return mFaces[face];
    }

    const Plane& edge(const pm::halfedge_handle& edge) const {
        return mEdges[edge.edge()];
    }

    const int8_t halfedge(const pm::halfedge_handle& edge) const {
        return mHalfEdges[edge];
    }

    SubDet pos(const pm::vertex_handle& vertex) const {
        auto he1 = vertex.any_incoming_halfedge();
        auto hEdges = vertex.outgoing_halfedges();
        pm::halfedge_handle he2;
        for (auto h : hEdges) {
            Plane p1 = mEdges[he1.edge()];
            Plane p2 = mEdges[h.edge()];
            if (!(p1 == p2))
                he2 = h;

        }
        //auto he2 = he1.next();
        auto face = he1.face();     
        return pos(he1, he2, face);
    }

    SubDet pos(pm::halfedge_handle he1, pm::halfedge_handle he2, pm::face_handle face) const {
        TG_ASSERT(face.is_valid() && he1.is_valid() && he2.is_valid());
        Plane T1 = mFaces[face];
        Plane T2 = mEdges[he1.edge()];
        Plane T3 = mEdges[he2.edge()];
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
        return pm::halfedge_handle();
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

    PlaneRay getRayPlanes(pm::vertex_handle vertex) const {
        for (pm::halfedge_handle& he1 : vertex.incoming_halfedges()) {
            for (pm::halfedge_handle& he2 : vertex.outgoing_halfedges()) {
                if (!(edge(he1.edge()) == edge(he2.edge())))
                    return PlaneRay{ edge(he1.edge()), edge(he2.edge()) };
            }
        }
    }

    const Plane& getAnyFace(pm::vertex_handle vertex) const {
        return mFaces[vertex.any_face()];
    }

    const Plane& getPlane(const pm::edge_handle& edge) const {
        return mEdges[edge];
    }

    const pm::Mesh& mesh() const {
        return mMesh;
    }

    int id() const {
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

using SharedPlaneMesh = std::shared_ptr<PlaneMesh>;