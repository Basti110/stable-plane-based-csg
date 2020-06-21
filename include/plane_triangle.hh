#pragma once
#include <typed-geometry/tg.hh>
#include <geometry.hh>


class PlaneTriangle {
    using geometry128 = ob::geometry<32, 21>;
    using plane128 = geometry128::plane_t;
    using Plane = geometry128::plane_t;
    using VertexAttribute = pm::vertex_attribute<tg::pos3>;
public:
    PlaneTriangle(tg::ipos3 x, tg::ipos3 y, tg::ipos3 z) {
        auto normal = tg::cross(y - x, y - z);
        auto normal_a = tg::cross(y - x, normal);
        auto normal_b = tg::cross(z - y, normal);
        auto normal_c = tg::cross(x - z, normal);

        mPlaneFace = Plane::from_points(x, y, z);
        mPlaneA = Plane::from_pos_normal(x, normal_a);
        mPlaneB = Plane::from_pos_normal(y, normal_b);
        mPlaneC = Plane::from_pos_normal(z, normal_c);

        mSubDetPointA = { ob::i128(0), ob::i128(0), ob::i128(0), ob::i128(0) };
        mSubDetPointB = { ob::i128(0), ob::i128(0), ob::i128(0), ob::i128(0) };
        mSubDetPointC = { ob::i128(0), ob::i128(0), ob::i128(0), ob::i128(0) };
    }

    void compute_subdetermiants() {
        ob::compute_subdeterminants(mPlaneFace, mPlaneA, mPlaneB, mSubDetPointA);
        ob::compute_subdeterminants(mPlaneFace, mPlaneB, mPlaneC, mSubDetPointB);
        ob::compute_subdeterminants(mPlaneFace, mPlaneC, mPlaneA, mSubDetPointC);
    }

    tg::dtriangle3 getTriangle() {
        if (!mSubDetPointA.is_valid() || !mSubDetPointB.is_valid() || !mSubDetPointC.is_valid()) {
            compute_subdetermiants();
        }
        auto pos_a = ob::to_position(mSubDetPointA);
        auto pos_b = ob::to_position(mSubDetPointB);
        auto pos_c = ob::to_position(mSubDetPointC);
        return { pos_a, pos_b, pos_c };
    }

    std::vector<tg::triangle3> get_planes_triangles(int side_len) {
        std::vector<tg::triangle3> triangles;
        triangles.reserve(8);
        get_plane_triangle(mPlaneFace, side_len, triangles);
        get_plane_triangle(mPlaneA, side_len, triangles);
        get_plane_triangle(mPlaneB, side_len, triangles);
        get_plane_triangle(mPlaneC, side_len, triangles);
        return triangles;
    }

    void get_plane_triangle(Plane& plane, int side_len, std::vector<tg::triangle3>& insert_vec) {
        tg::dplane3 dplane = plane.to_dplane();
        tg::vec3 normal = tg::vec3(dplane.normal);
        auto ortho_vec1 = tg::normalize(tg::cross(normal, { normal.x, normal.y + 1, normal.z + 1 }));
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
    Plane mPlaneFace;
    Plane mPlaneA;
    Plane mPlaneB;
    Plane mPlaneC;
    ob::subdeterminants<geometry128> mSubDetPointA;
    ob::subdeterminants<geometry128> mSubDetPointB;
    ob::subdeterminants<geometry128> mSubDetPointC;

};