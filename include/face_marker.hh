#pragma once
#include <vector>
#include <set>
#include <typed-geometry/tg.hh>
#include <plane_polygon.hh>
#include <iostream>
#include <fstream>

class FaceMarker2 {
public:
    FaceMarker2() = delete;
    FaceMarker2(const PlaneMesh* planeMeshA, const PlaneMesh* planeMeshB, tg::color3 defaultColor = tg::color3::white) :
        mPlaneMeshA(planeMeshA), 
        mPlaneMeshB(planeMeshB),
        mDefaultColor(defaultColor)
    {
        mFaceColorsA = planeMeshA->mesh().faces().make_attribute_with_default(defaultColor);
        mFaceColorsB = planeMeshB->mesh().faces().make_attribute_with_default(defaultColor);
    }

    void insertPositions(std::vector<tg::pos<3, long>> polygon) {
        mPolygonPositions.push_back(polygon);
    }

    void colorFace(pm::face_index face, int planeMeshId, tg::color3 color = tg::color3::red) {
        if (planeMeshId == mPlaneMeshA->id() && face.is_valid())
            mFaceColorsA[face.of(mPlaneMeshA->mesh())] = color;
        else if (face.is_valid())
            mFaceColorsB[face.of(mPlaneMeshB->mesh())] = color;
    }

    void uncolorFace(pm::face_index face, int planeMeshId) {
        if (planeMeshId == mPlaneMeshA->id() && face.is_valid()) {
            if(mMarkedFacesA.count(face) == 0)
                mFaceColorsA[face.of(mPlaneMeshA->mesh())] = mDefaultColor;
            else
                mFaceColorsA[face.of(mPlaneMeshA->mesh())] = mMarkColorA;
        }          
        else if (face.is_valid()) {
            if (mMarkedFacesB.count(face) == 0)
                mFaceColorsB[face.of(mPlaneMeshB->mesh())] = mDefaultColor;
            else
                mFaceColorsB[face.of(mPlaneMeshB->mesh())] = mMarkColorB;
        }         
    }

    void markFace(pm::face_index face, int planeMeshId) {
        if (planeMeshId == mPlaneMeshA->id() && face.is_valid()) {
            mMarkedFacesA.insert(face);
            mFaceColorsA[face.of(mPlaneMeshA->mesh())] = mMarkColorA;
        }          
        else if (face.is_valid()) {
            mMarkedFacesB.insert(face);
            mFaceColorsB[face.of(mPlaneMeshB->mesh())] = mMarkColorB;
        }
            
    }

    void unmarkFace(pm::face_index face, int planeMeshId) {
        if (planeMeshId == mPlaneMeshA->id() && face.is_valid()) {
            mMarkedFacesA.erase(face);
            mFaceColorsA[face.of(mPlaneMeshA->mesh())] = mDefaultColor;
        }          
        else if (face.is_valid()) {
            mMarkedFacesB.erase(face);
            mFaceColorsB[face.of(mPlaneMeshA->mesh())] = mDefaultColor;
        }           
    }

    void setMarkColorA(tg::color3 color) {
        mMarkColorA = color;
    }

    void setMarkColorB(tg::color3 color) {
        mMarkColorB = color;
    }

    pm::face_attribute<tg::color3>& faceColorsA() {
        return mFaceColorsA;
    }

    pm::face_attribute<tg::color3>& faceColorsB() {
        return mFaceColorsB;
    }

    void saveMarkedFacesInFile() {
        saveMarkedFacesInFile(mMarkedFacesA, mPlaneMeshA, mPathA);
        saveMarkedFacesInFile(mMarkedFacesB, mPlaneMeshB, mPathB);
    }

    void saveMarkedFacesInFile(const std::set<pm::face_index>& markedFaces, const PlaneMesh* planeMesh, const std::string& path) {
        std::vector<tg::vec<3, long>> positions;
        std::vector<std::vector<long>> indices(markedFaces.size());
        int i = 0;
        for (auto face : markedFaces) {
            auto faceHandle = face.of(planeMesh->mesh());
            for (auto vertex : faceHandle.vertices()) {
                pos_t pos = planeMesh->posInt(vertex);
                auto x = double(pos.x);
                auto y = double(pos.y);
                auto z = double(pos.z);
                auto posLong = tg::vec<3, long>({ (long)x, (long)y, (long)z });
                positions.push_back(posLong);
                indices[i].push_back(positions.size());
            }
            i++;
        }

        std::ofstream objFileA(path);
        if (objFileA.is_open())
        {
            for (auto pos : positions) {
                objFileA << "v " << pos.x << " " << pos.y << " " << pos.z << std::endl;
            }

            for (auto face : indices) {
                objFileA << "f";
                for (auto index : face) {
                    objFileA << " " << index;
                }
                objFileA << std::endl;
            }
            objFileA.close();
        }
    }

private:
    std::string mPathA = "../data/mesh/meshA.obj";
    std::string mPathB = "../data/mesh/meshB.obj";
    tg::color3 mDefaultColor;
    tg::color3 mMarkColorA = tg::color3::blue;
    tg::color3 mMarkColorB = tg::color3::green;
    std::vector<std::vector<tg::pos<3, long>>> mPolygonPositions;
    pm::face_attribute<tg::color3> mFaceColorsA;
    pm::face_attribute<tg::color3> mFaceColorsB;
    std::set<pm::face_index> mMarkedFacesA;
    std::set<pm::face_index> mMarkedFacesB;
    const PlaneMesh* mPlaneMeshA;
    const PlaneMesh* mPlaneMeshB;
};