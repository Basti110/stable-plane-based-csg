#pragma once
#include <vector>
#include <set>
#include <typed-geometry/tg.hh>
#include <plane_polygon.hh>

class FaceMarker2 {
public:
    FaceMarker2() = delete;
    FaceMarker2(const PlaneMesh* planeMeshA, const PlaneMesh* planeMeshB, tg::color3 defaultColor = tg::color3::white) :
        mPlaneMeshA(planeMeshA), 
        mPlaneMeshB(planeMeshB)
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
            if(mMarkedFacesA.count(face) != 0)
                mFaceColorsA[face.of(mPlaneMeshA->mesh())] = mDefaultColor;
            else
                mFaceColorsA[face.of(mPlaneMeshA->mesh())] = mMarkColorA;
        }          
        else if (face.is_valid()) {
            if (mMarkedFacesB.count(face) != 0)
                mFaceColorsB[face.of(mPlaneMeshA->mesh())] = mDefaultColor;
            else
                mFaceColorsB[face.of(mPlaneMeshA->mesh())] = mMarkColorB;
        }         
    }

    void markFace(pm::face_index face, int planeMeshId) {
        if (planeMeshId == mPlaneMeshA->id() && face.is_valid()) {
            mMarkedFacesA.insert(face);
            mFaceColorsA[face.of(mPlaneMeshA->mesh())] = mMarkColorA;
        }          
        else if (face.is_valid()) {
            mMarkedFacesB.insert(face);
            mFaceColorsA[face.of(mPlaneMeshA->mesh())] = mMarkColorB;
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

    pm::face_attribute<tg::color3> faceColorsA() {
        return mFaceColorsA;
    }

    pm::face_attribute<tg::color3> faceColorsB() {
        return mFaceColorsB;
    }

private:
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