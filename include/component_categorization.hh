#pragma once
#include <plane_polygon.hh>
#include <nexus/test.hh>
#include <octree.hh>
#include <face_component_finder.hh>
#include <aabb.hh>
#include <set>

class ComponentCategorization {
	enum class InOutState : int8_t {
		UNDEFINED = -1,
		OUTSIDE = 0,
		INSIDE = 1,		
	};
public:
	ComponentCategorization(SharedOctree octree, SharedFaceComponents faceComponentsA, SharedFaceComponents faceComponentsB, const IntersectionCut& iCut)
		: mSharedOctree(octree), mFaceComponentsA(faceComponentsA), mFaceComponentsB(faceComponentsB) {
		mComponentIsOutsideA = std::vector<int8_t>(mFaceComponentsA->getNumberOfComponents(), -1);
		mComponentIsOutsideB = std::vector<int8_t>(mFaceComponentsB->getNumberOfComponents(), -1);

		auto iSectA = iCut.getIntersectionEdgesMarkerA();
		mComponentNeighborsA.resize(mFaceComponentsA->getNumberOfComponents());
		for (pm::edge_handle& e : iSectA.mesh().all_edges()) {
			if (iSectA[e]) {
				int componentA = mFaceComponentsA->getComponentOfFace(e.halfedgeA().face());
				int componentB = mFaceComponentsA->getComponentOfFace(e.halfedgeB().face());
				mComponentNeighborsA[componentA].insert(componentB);
				mComponentNeighborsA[componentB].insert(componentA);
			}
		}

		auto iSectB = iCut.getIntersectionEdgesMarkerB();
		mComponentNeighborsB.resize(mFaceComponentsB->getNumberOfComponents());
		for (pm::edge_handle& e : iSectB.mesh().all_edges()) {
			if (iSectB[e]) {
				int componentA = mFaceComponentsB->getComponentOfFace(e.halfedgeA().face());
				int componentB = mFaceComponentsB->getComponentOfFace(e.halfedgeB().face());
				mComponentNeighborsB[componentA].insert(componentB);
				mComponentNeighborsB[componentB].insert(componentA);
			}
		}
		assignInOut(iCut);
	}

	void assignInOut(const IntersectionCut& iCut) {
		pm::vertex_handle vertexA = pm::vertex_handle::invalid;
		auto iSectA = iCut.getIntersectionEdgesMarkerA();
		for (pm::edge_handle& e : iSectA.mesh().all_edges()) {
			if (!iSectA[e]) {
				vertexA = e.halfedgeA().vertex_from();
				break;
			}
		}
		TG_ASSERT(vertexA != pm::vertex_handle::invalid);
		SharedDebugRayInfo rayInfoA = std::make_shared<DebugRayInfo>();
		auto intersections = mSharedOctree->countIntersectionsToOutside2(vertexA, mSharedOctree->getPlaneMeshA(), rayInfoA);
		auto component = mFaceComponentsA->getComponentOfFace(vertexA.any_face());
		mComponentIsOutsideA[component] = intersections % 2;
		propagateComponentStateRecursive(mComponentIsOutsideA, component);

		pm::vertex_handle vertexB = pm::vertex_handle::invalid;
		auto iSectB = iCut.getIntersectionEdgesMarkerB();
		for (pm::edge_handle& e : iSectB.mesh().all_edges()) {
			if (!iSectB[e]) {
				vertexB = e.halfedgeB().vertex_from();
				break;
			}
		}
		TG_ASSERT(vertexB != pm::vertex_handle::invalid);
		SharedDebugRayInfo rayInfoB = std::make_shared<DebugRayInfo>();
		intersections = mSharedOctree->countIntersectionsToOutside2(vertexB, mSharedOctree->getPlaneMeshB(), rayInfoB);
		component = mFaceComponentsB->getComponentOfFace(vertexB.any_face());
		mComponentIsOutsideB[component] = intersections % 2;
		propagateComponentStateRecursive(mComponentIsOutsideB, component);
		
				
		auto colorsA = getColorToStateA();
		auto colorsB = getColorToStateB();



		//##################################################################
		int i = 0;

		#define RAYINFO rayInfoA
		std::vector<tg::dsegment3> lines;
		for (int i = 0; i < (int)RAYINFO->rayPath.size() - 1; ++i) {
			lines.push_back(tg::dsegment3{ ob::to_position(RAYINFO->rayPath[i]), ob::to_position(RAYINFO->rayPath[i + 1]) });
		}
		//lines.push_back(tg::dsegment3{ tg::dpos3(rayInfo->rayStartDirect), tg::dpos3(rayInfo->rayEndDirect) });

		for (int i = 0; i < (int)RAYINFO->nexPointsCell.size() - 1; ++i) {
			lines.push_back(tg::dsegment3{ tg::dpos3(RAYINFO->nexPointsCell[i]), tg::dpos3(RAYINFO->nexPointsCell[i + 1]) });
		}

		std::vector<tg::aabb3> returnBoxes;
		for (auto box : RAYINFO->rayBoxesDirect) {
			returnBoxes.push_back(tg::aabb3(tg::pos3(box.min), tg::pos3(box.max)));
		}

		mSharedOctree->getPlaneMeshA().checkAndComputePositions();
		mSharedOctree->getPlaneMeshB().checkAndComputePositions();

		//auto const octreeCells = gv::lines(boxes).line_width_world(200000);
		auto const rayCells = gv::lines(returnBoxes).line_width_world(250000);
		auto const rayPath = gv::lines(lines).line_width_world(300000);
		auto const positions1 = mSharedOctree->getPlaneMeshA().positions();
		auto const positionLines1 = gv::lines(mSharedOctree->getPlaneMeshA().positions()).line_width_world(100000);
		auto const positions2 = mSharedOctree->getPlaneMeshB().positions();
		auto const positionLines2 = gv::lines(mSharedOctree->getPlaneMeshB().positions()).line_width_world(100000);
		bool tooglePolygons = true;

		if (tooglePolygons) {
			//auto view = gv::view(octreeCells, tg::color3::blue);
			//auto view = gv::view(rayCells, tg::color3::green);
			//gv::view(rayPath, tg::color3::red);
			//auto view = gv::view(positions1, colorsA);
			//gv::view(positionLines1);
			//auto view = gv::view(positions2, colorsB);
			auto view = gv::view(positions2, mFaceComponentsB->getColorAssignment());
			gv::view(positionLines2);
		}
	}

	void propagateComponentStateRecursive(std::vector<int8_t>& componentIsOutside, int component) {
		int8_t state = componentIsOutside[component];
		for (int neighborComponent : mComponentNeighborsA[component]) {
			if (componentIsOutside[neighborComponent] == (int8_t)InOutState::UNDEFINED) {
				componentIsOutside[neighborComponent] = (state + 1) % 2;
				propagateComponentStateRecursive(componentIsOutside, neighborComponent);
			}
		}
	}

	pm::face_attribute<tg::color3> getColorToStateA() {
		const pm::Mesh& mesh = mSharedOctree->getPlaneMeshA().mesh();
		SharedFaceComponents faceComponentsA = mFaceComponentsA;
		std::vector<int8_t>& componentIsOutsideA = mComponentIsOutsideA;
		
		pm::face_attribute<tg::color3> faceColors = mesh.faces().map([&](pm::face_handle& face) {
				int component = faceComponentsA->getComponentOfFace(face);
				return componentIsOutsideA[component] == 1 ? tg::color3::black : tg::color3::white;
			});
		return faceColors;
	}

	pm::face_attribute<tg::color3> getColorToStateB() {
		const pm::Mesh& mesh = mSharedOctree->getPlaneMeshB().mesh();
		SharedFaceComponents faceComponents = mFaceComponentsB;
		std::vector<int8_t>& componentIsOutside = mComponentIsOutsideB;

		pm::face_attribute<tg::color3> faceColors = mesh.faces().map([&](pm::face_handle& face) {
				int component = faceComponents->getComponentOfFace(face);
				return componentIsOutside[component] == 1 ? tg::color3::black : tg::color3::white;
			});
		return faceColors;
	}

private:
	SharedOctree mSharedOctree;
	SharedFaceComponents mFaceComponentsA;
	SharedFaceComponents mFaceComponentsB;
	std::vector<int8_t> mComponentIsOutsideA;
	std::vector<int8_t> mComponentIsOutsideB;
	std::vector<std::unordered_set<int>> mComponentNeighborsA;
	std::vector<std::unordered_set<int>> mComponentNeighborsB;

};