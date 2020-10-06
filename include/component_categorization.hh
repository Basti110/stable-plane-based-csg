#pragma once
#include <plane_polygon.hh>
#include <nexus/test.hh>
#include <octree.hh>
#include <face_component_finder.hh>
#include <aabb.hh>
#include <set>

class ComponentCategorization {
public:
	ComponentCategorization(SharedOctree octree, SharedFaceComponents faceComponentsA, SharedFaceComponents faceComponentsB, const IntersectionCut& iCut)
		: mSharedOctree(octree), mFaceComponentsA(faceComponentsA), mFaceComponentsB(faceComponentsB) {

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
			if (iSectA[e]) {
				int componentA = mFaceComponentsB->getComponentOfFace(e.halfedgeA().face());
				int componentB = mFaceComponentsB->getComponentOfFace(e.halfedgeB().face());
				mComponentNeighborsA[componentA].insert(componentB);
				mComponentNeighborsA[componentB].insert(componentA);
			}
		}
	}

	void assignInOut() {
		
	}

private:
	SharedOctree mSharedOctree;
	SharedFaceComponents mFaceComponentsA;
	SharedFaceComponents mFaceComponentsB;
	std::vector<std::unordered_set<int>> mComponentNeighborsA;
	std::vector<std::unordered_set<int>> mComponentNeighborsB;

};