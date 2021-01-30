#pragma once
#include <glow-extras/timing/CpuTimer.hh>
#include <plane_polygon.hh>
#include <nexus/test.hh>
#include <octree.hh>
#include <face_component_finder.hh>
#include <ray_cast.hh>
#include <aabb.hh>
#include <set>

class ComponentCategorization {
	enum class InOutState : int8_t {
		UNDEFINED = -1,
		OUTSIDE = 0,
		INSIDE = 1,	
        COPLANAR = 2,
	};
public:
	ComponentCategorization(SharedOctree octree, SharedFaceComponents faceComponentsA, SharedFaceComponents faceComponentsB, const IntersectionCut& iCut)
		: mSharedOctree(octree), mFaceComponentsA(faceComponentsA), mFaceComponentsB(faceComponentsB) {
		mComponentIsOutsideA = std::vector<int8_t>(mFaceComponentsA->getNumberOfComponents(), -1);
		mComponentIsOutsideB = std::vector<int8_t>(mFaceComponentsB->getNumberOfComponents(), -1);
        
        octree->repairOctree(iCut);
		auto iSectA = iCut.getIntersectionEdgesMarkerA();
        //
		mComponentNeighborsA.resize(mFaceComponentsA->getNumberOfComponents());
		for (pm::edge_handle& e : iSectA.mesh().edges()) {
			if (iSectA[e]) {
				int componentA = mFaceComponentsA->getComponentOfFace(e.halfedgeA().face());
				int componentB = mFaceComponentsA->getComponentOfFace(e.halfedgeB().face());
				mComponentNeighborsA[componentA].insert(componentB);
				mComponentNeighborsA[componentB].insert(componentA);
			}
		}

		auto iSectB = iCut.getIntersectionEdgesMarkerB();
		mComponentNeighborsB.resize(mFaceComponentsB->getNumberOfComponents());
		for (pm::edge_handle& e : iSectB.mesh().edges()) {
			if (iSectB[e]) {
				int componentA = mFaceComponentsB->getComponentOfFace(e.halfedgeA().face());
				int componentB = mFaceComponentsB->getComponentOfFace(e.halfedgeB().face());
				mComponentNeighborsB[componentA].insert(componentB);
				mComponentNeighborsB[componentB].insert(componentA);
			}
		}

        auto coPlanarFacesA = iCut.getCoplanarFacesMeshA();
        auto coPlanarFacesB = iCut.getCoplanarFacesMeshB();

        for (auto face : coPlanarFacesA) {
            auto faceHandle = face.of(octree->getPlaneMeshA().mesh());
            bool t1 = faceHandle.is_removed();
            auto t2 = faceComponentsA->getFaceToComponent()[face];
            mComponentIsOutsideA[t2] = (int8_t)InOutState::COPLANAR;
        }
            

        for (auto face : coPlanarFacesB)
            mComponentIsOutsideB[faceComponentsB->getFaceToComponent()[face]] = (int8_t)InOutState::COPLANAR;
        
		assignInOut(iCut);
	}

	void assignInOut(const IntersectionCut& iCut) {
        std::vector<SharedDebugRayInfo> rayInfosA;
        std::vector<SharedDebugRayInfo> rayInfosB;
		
        for (int i = 0; i < mFaceComponentsA->getNumberOfComponents(); ++i) {
            pm::vertex_handle vertexA = pm::vertex_handle::invalid;
            if (mComponentIsOutsideA[i] != (int8_t)InOutState::UNDEFINED)
                continue;

            auto iSectA = iCut.getIntersectionEdgesMarkerA();
            auto componentToFace = mFaceComponentsA->getComponentToFace(i);
            for (auto faceIndex : componentToFace) {
                for (pm::edge_handle& e : faceIndex.of(iSectA.mesh()).edges()) {
                    if (!iSectA[e]) {
                        vertexA = e.halfedgeA().vertex_from();
                        break;
                    }
                }
                if (vertexA != pm::vertex_handle::invalid)
                    break;
            }

            TG_ASSERT(vertexA != pm::vertex_handle::invalid);
            SharedDebugRayInfo rayInfoA = std::make_shared<DebugRayInfo>();
            RayCast rayCast(mSharedOctree->getPlaneMeshA(), mSharedOctree->getPlaneMeshB(), mSharedOctree, rayInfoA);
            auto intersections = rayCast.countIntersectionsToOutside(vertexA);
            //auto intersections = mSharedOctree->countIntersectionsToOutside2(vertexA, mSharedOctree->getPlaneMeshA(), rayInfoA);
            rayInfosA.push_back(rayInfoA);
            auto component = mFaceComponentsA->getComponentOfFace(vertexA.any_face());
            mComponentIsOutsideA[component] = intersections % 2;
            propagateComponentStateRecursiveA(mComponentIsOutsideA, component);
        }

        for (int i = 0; i < mFaceComponentsB->getNumberOfComponents(); ++i) {
            pm::vertex_handle vertexB = pm::vertex_handle::invalid;
            if (mComponentIsOutsideB[i] != (int8_t)InOutState::UNDEFINED)
                continue;

            auto iSectB = iCut.getIntersectionEdgesMarkerB();
            auto componentToFace = mFaceComponentsB->getComponentToFace(i);
            for (auto faceIndex : componentToFace) {
                for (pm::edge_handle& e : faceIndex.of(iSectB.mesh()).edges()) {
                    if (!iSectB[e]) {
                        vertexB = e.halfedgeA().vertex_from();
                        break;
                    }
                }
                if (vertexB != pm::vertex_handle::invalid)
                    break;
            }
            TG_ASSERT(vertexB != pm::vertex_handle::invalid);
            SharedDebugRayInfo rayInfoB = std::make_shared<DebugRayInfo>();
            RayCast rayCast(mSharedOctree->getPlaneMeshB(), mSharedOctree->getPlaneMeshA(), mSharedOctree, rayInfoB);
            glow::timing::CpuTimer timer;
            auto intersections = rayCast.countIntersectionsToOutside(vertexB);
            std::cout << "Ray Cast: " << timer.elapsedMillisecondsD() << "ms" << std::endl;
            //auto intersections = mSharedOctree->countIntersectionsToOutside2(vertexB, mSharedOctree->getPlaneMeshB(), rayInfoB);
            rayInfosB.push_back(rayInfoB);
            auto component = mFaceComponentsB->getComponentOfFace(vertexB.any_face());
            mComponentIsOutsideB[component] = intersections % 2;
            propagateComponentStateRecursiveB(mComponentIsOutsideB, component);
        }
        return;
        #define RAYINFO rayInfosB
        std::vector<tg::dsegment3> lines;
        //auto info = rayInfosB[5];
        for (auto info : RAYINFO) {
            for (int i = 0; i < (int)info->rayPath.size() - 1; ++i) {
                lines.push_back(tg::dsegment3{ob::to_position(info->rayPath[i]), ob::to_position(info->rayPath[i + 1]) });
            }
        }

        for (auto info : RAYINFO) {
            for (int i = 0; i < (int)info->nexPointsCell.size() - 1; ++i) {
                lines.push_back(tg::dsegment3{ tg::dpos3(info->nexPointsCell[i]), tg::dpos3(info->nexPointsCell[i + 1]) });
            }
        }

        //lines.push_back(tg::dsegment3{ tg::dpos3(rayInfo->rayStartDirect), tg::dpos3(rayInfo->rayEndDirect) });

        /*for (int i = 0; i < (int)RAYINFO->nexPointsCell.size() - 1; ++i) {
            lines.push_back(tg::dsegment3{ tg::dpos3(RAYINFO->nexPointsCell[i]), tg::dpos3(RAYINFO->nexPointsCell[i + 1]) });
        }

        std::vector<tg::aabb3> returnBoxes;
        for (auto box : RAYINFO->rayBoxesDirect) {
            returnBoxes.push_back(tg::aabb3(tg::pos3(box.min), tg::pos3(box.max)));
        }*/

        std::vector<AABB> boxes;
        mSharedOctree->insertAABB(boxes);
        std::vector<tg::aabb3> returnBoxes;
        returnBoxes.reserve(boxes.size());
        for (auto box : boxes) {
            returnBoxes.push_back(tg::aabb3(tg::pos3(box.min), tg::pos3(box.max)));
        }

        std::vector<tg::aabb3> hitBoxes;
        for (auto box : RAYINFO[0]->rayBoxesDirect) {
            hitBoxes.push_back(tg::aabb3(tg::pos3(box.min), tg::pos3(box.max)));
        }


        //auto const rayCells = gv::lines(hitBoxes).line_width_world(300000);

        mSharedOctree->getPlaneMeshA().checkAndComputePositions();
        mSharedOctree->getPlaneMeshB().checkAndComputePositions();
        auto const octreeCells = gv::lines(returnBoxes).line_width_world(250000);
        auto const rayPath = gv::lines(lines).line_width_world(300000);
        auto const lines1 = gv::lines(mSharedOctree->getPlaneMeshA().positions()).line_width_world(100000);
        auto const lines2 = gv::lines(mSharedOctree->getPlaneMeshB().positions()).line_width_world(100000);

        {
            auto view = gv::view(octreeCells, tg::color3::blue);
            //gv::view(rayCells, tg::color3::green);
            gv::view(rayPath, tg::color3::red);
            gv::view(mSharedOctree->getPlaneMeshA().positions());
            //gv::view(mSharedOctree->getPlaneMeshB().positions());
            gv::view(lines1);
            gv::view(lines2);
            //auto view = gv::view(positions2, colorsB); // mFaceComponentsB->getColorAssignment());
            //gv::view(positionLines2);
        }
	}

	void propagateComponentStateRecursiveA(std::vector<int8_t>& componentIsOutside, int component) {
		int8_t state = componentIsOutside[component];
		for (int neighborComponent : mComponentNeighborsA[component]) {
			if (componentIsOutside[neighborComponent] == (int8_t)InOutState::UNDEFINED) {
				componentIsOutside[neighborComponent] = (state + 1) % 2;
				propagateComponentStateRecursiveA(componentIsOutside, neighborComponent);
			}
		}
	}

    void propagateComponentStateRecursiveB(std::vector<int8_t>& componentIsOutside, int component) {
        int8_t state = componentIsOutside[component];
        for (int neighborComponent : mComponentNeighborsB[component]) {
            if (componentIsOutside[neighborComponent] == (int8_t)InOutState::UNDEFINED) {
                componentIsOutside[neighborComponent] = (state + 1) % 2;
                propagateComponentStateRecursiveB(componentIsOutside, neighborComponent);
            }
        }
    }

    bool copyFacesFromMesh(pm::vertex_attribute<tg::pos3>& pos, pm::Mesh& mesh, const VertexAttribute& copyFrom, pm::face_attribute<bool>& copyFaces, int scale=10e6) {
        const auto& oldMesh = copyFrom.mesh();
        pm::Mesh& newMesh = mesh;
        for (auto face : oldMesh.faces()) {
            if (!copyFaces[face])
                continue;

            auto he = face.any_halfedge();
            auto heTmp = he.next();
            auto pFrom = newMesh.vertices().add();
            auto oldFrom = copyFrom[heTmp.vertex_from()];
            pos[pFrom] = tg::pos3((double)oldFrom.x / scale, (double)oldFrom.y / scale, (double)oldFrom.z / scale);
            std::vector<pm::halfedge_handle> halfEdges;

            while(he != heTmp) {
                pm::vertex_handle pTo = newMesh.vertices().add();
                auto oldTo = copyFrom[heTmp.vertex_to()];
                pos[pTo] = tg::pos3((double)oldTo.x / scale, (double)oldTo.y / scale, (double)oldTo.z / scale);
                auto newHe = newMesh.halfedges().add_or_get(pFrom, pTo);
                halfEdges.push_back(newHe);
                pFrom = pTo;
                heTmp = heTmp.next();
            }
            auto pTo = halfEdges[0].vertex_from();
            halfEdges.push_back(newMesh.halfedges().add_or_get(pFrom, pTo));
            newMesh.faces().add(halfEdges);
        }
        //pm::deduplicate(newMesh, pos);
        //newMesh.compactify();
        return true;
    }

    void renderFinalResult(const IntersectionCut& iCut, float scale = 3000.f) {
        auto colorsA = getColorToStateA();
        auto colorsB = getColorToStateB();
        mSharedOctree->getPlaneMeshA().checkAndComputePositions();
        mSharedOctree->getPlaneMeshB().checkAndComputePositions();
        auto& planeMeshA = mSharedOctree->getPlaneMeshA();
        auto& planeMeshB = mSharedOctree->getPlaneMeshB();

        // Attributes
        auto colorAtrMaskAIn = pm::face_attribute<bool>(colorsA.map([](tg::color3 c) { return c == tg::color3::white; }));
        auto colorAtrMaskBIn = pm::face_attribute<bool>(colorsB.map([](tg::color3 c) { return c == tg::color3::white; }));
        auto colorAtrMaskAOut = pm::face_attribute<bool>(colorsA.map([](tg::color3 c) { return c != tg::color3::white; }));
        auto colorAtrMaskBOut = pm::face_attribute<bool>(colorsB.map([](tg::color3 c) { return c != tg::color3::white; }));

        //Maskes
        auto maskFaceAIn = gv::masked(colorAtrMaskAIn);
        auto maskFaceBIn = gv::masked(colorAtrMaskBIn);
        auto maskFaceAOut = gv::masked(colorAtrMaskAOut);
        auto maskFaceBOut = gv::masked(colorAtrMaskBOut);

        auto positionsA = gv::make_renderable(mSharedOctree->getPlaneMeshA().positions());
        auto const positionLinesASmall = gv::make_renderable(gv::lines(mSharedOctree->getPlaneMeshA().positions()).line_width_world(4 * scale));
                   
        auto positionsB = gv::make_renderable(gv::make_renderable(mSharedOctree->getPlaneMeshB().positions()));
        auto const positionLinesBSmall = gv::make_renderable(gv::lines(mSharedOctree->getPlaneMeshB().positions()).line_width_world(4 * scale));


        auto const isectLines1 = gv::make_renderable(gv::lines(mSharedOctree->getPlaneMeshA().positions()).line_width_world(30 * scale));
        isectLines1->addAttribute(gv::detail::make_mesh_attribute("aColor", glow::colors::color(0, 0, 0)));
        auto const isectLines2 = gv::make_renderable(gv::lines(mSharedOctree->getPlaneMeshB().positions()).line_width_world(30 * scale));
        isectLines2->addAttribute(gv::detail::make_mesh_attribute("aColor", glow::colors::color(0, 0, 0)));

        int tooglePolygons = 0;
        int colorMode = 0;
        bool toogleLinesA = true;
        bool toogleLinesB = true;
        bool showIntersection = true;

        pm::Mesh resultMesh;
        pm::vertex_attribute<tg::pos3> resultPos(resultMesh);
        copyFacesFromMesh(resultPos, resultMesh, planeMeshA.positions(), colorAtrMaskAOut);
        copyFacesFromMesh(resultPos, resultMesh, planeMeshB.positions(), colorAtrMaskBOut);
        pm::save("../out.obj", resultPos);

        gv::interactive([&](auto dt) {
            auto view = gv::view();
            if (tooglePolygons != 6)
                gv::view(positionsA, gv::print_mode, gv::no_grid);
            if (tooglePolygons != 5)
                gv::view(positionsB, gv::print_mode, gv::no_grid);


            if (showIntersection)
                gv::view(isectLines1, gv::masked(iCut.getIntersectionEdgesMarkerA()));
            //else
                //gv::view(isectLines2, gv::masked(iCut.getIntersectionEdgesMarkerB()));

            if (toogleLinesA)
                gv::view(positionLinesASmall);

            if (toogleLinesB)
                gv::view(positionLinesBSmall);


            ImGui::Begin("Move");
            bool toogled = ImGui::RadioButton("Mesh 1 + Mesh 2", &tooglePolygons, 0);
            toogled |= ImGui::RadioButton("Mesh 2 AND Mesh 1", &tooglePolygons, 1);
            toogled |= ImGui::RadioButton("Mesh 2 OR Mesh 1", &tooglePolygons, 2);
            toogled |= ImGui::RadioButton("Mesh 1 - Mesh 2", &tooglePolygons, 3);
            toogled |= ImGui::RadioButton("Mesh 2 - Mesh 1", &tooglePolygons, 4);
            toogled |= ImGui::RadioButton("Mesh 1", &tooglePolygons, 5);
            toogled |= ImGui::RadioButton("Mesh 2", &tooglePolygons, 6);
            toogled |= ImGui::RadioButton("Color: White", &colorMode, 0);
            toogled |= ImGui::RadioButton("Color: Components", &colorMode, 1);
            toogled |= ImGui::RadioButton("Color: In/Out", &colorMode, 2);
            toogled |= ImGui::Checkbox("Show Lines A", &toogleLinesA);
            toogled |= ImGui::Checkbox("Show Lines B", &toogleLinesB);
            toogled |= ImGui::Checkbox("Show Intersection", &showIntersection);
            if (ImGui::Button("Save")) {
                std::cout << "Store Mesh" << std::endl;
                //pm::Mesh resultMesh;
                //pm::vertex_attribute<tg::pos3> resultPos;
                //copyFacesFromMesh
                
            }
            /*if (ImGui::IsKeyPressed('L')) {
                toogleLines = !toogleLines;
                toogled = true;
            }*/
              
            if (toogled) {
                positionsA = gv::make_renderable(mSharedOctree->getPlaneMeshA().positions());
                positionsB = gv::make_renderable(mSharedOctree->getPlaneMeshB().positions());
                if (tooglePolygons == 1) {
                    positionsA->setMasking(maskFaceAOut);
                    positionsB->setMasking(maskFaceBOut);
                }
                else if (tooglePolygons == 2) {
                    positionsA->setMasking(maskFaceAIn);
                    positionsB->setMasking(maskFaceBIn);
                }
                else if (tooglePolygons == 3) {
                    positionsA->setMasking(maskFaceAOut);
                    positionsB->setMasking(maskFaceBIn);
                }
                else if (tooglePolygons == 4) {
                    positionsA->setMasking(maskFaceAIn);
                    positionsB->setMasking(maskFaceBOut);
                }
                if (colorMode == 0) {
                    gv::configure(*positionsA, tg::color3(0.95));
                    gv::configure(*positionsB, tg::color3(0.95));
                }
                else if (colorMode == 1) {
                    gv::configure(*positionsA, mFaceComponentsA->getColorAssignment());
                    gv::configure(*positionsB, mFaceComponentsB->getColorAssignment());
                }
                else if (colorMode == 2) {
                    gv::configure(*positionsA, colorsA);
                    gv::configure(*positionsB, colorsB);
                }
                    
                gv::view_clear_accumulation();
            }
                
            ImGui::End();
            });
    }

    pm::face_attribute<tg::color3> getColorToStateA() {
        const pm::Mesh& mesh = mSharedOctree->getPlaneMeshA().mesh();
        SharedFaceComponents faceComponents = mFaceComponentsA;
        std::vector<int8_t>& componentIsOutside = mComponentIsOutsideA;

        pm::face_attribute<tg::color3> faceColors = mesh.faces().map([&](pm::face_handle& face) {
            int component = faceComponents->getComponentOfFace(face);
            if (componentIsOutside[component] == 2)
                return tg::color3::red;
            return componentIsOutside[component] == 0 ? tg::color3::white : tg::color3(130 / 255., 177 / 255., 255 / 255.);
            });
        return faceColors;
    }

	pm::face_attribute<tg::color3> getColorToStateB() {
		const pm::Mesh& mesh = mSharedOctree->getPlaneMeshB().mesh();
		SharedFaceComponents faceComponents = mFaceComponentsB;
		std::vector<int8_t>& componentIsOutside = mComponentIsOutsideB;

		pm::face_attribute<tg::color3> faceColors = mesh.faces().map([&](pm::face_handle& face) {
				int component = faceComponents->getComponentOfFace(face);
                if (componentIsOutside[component] == 2)
                    return tg::color3::red;
				return componentIsOutside[component] == 0 ? tg::color3::white :  tg::color3(130 / 255., 177 / 255., 255 / 255.);
			});
		return faceColors;
	}

private:
    //std::unordered_set<int> mCoplanarComponentsA;
    //std::unordered_set<int> mCoplanarComponentsB;
	SharedOctree mSharedOctree;
	SharedFaceComponents mFaceComponentsA;
	SharedFaceComponents mFaceComponentsB;
	std::vector<int8_t> mComponentIsOutsideA;
	std::vector<int8_t> mComponentIsOutsideB;
	std::vector<std::unordered_set<int>> mComponentNeighborsA;
	std::vector<std::unordered_set<int>> mComponentNeighborsB;

};