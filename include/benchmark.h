#include <nexus/app.hh>
#include <intersection_cut.hh>
#include <obj_config.hh>
#include <octree.hh>
#include <face_component_finder.hh>
#include <polymesh/algorithms/deduplicate.hh>
#include <component_categorization.hh>
#include <benchmark_writer.h>
#include <iomanip>
//#include <ctracer/trace-config.hh>
#define GIGA 1000000000
#define MEGA 1000000
#define CLOCK 3799999

namespace Benchmark {
    void printStats(ct::scope& s) {
        auto trace = s.trace();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(trace.time_end() - trace.time_start()).count();
        std::cout << std::endl << "-Timing Stats [" << s.name() << "], total: " << elapsed << "ms" << std::endl;
        auto t = trace.compute_location_stats();
        for (auto te : t) {
            auto cycles = te.total_cycles;
            size_t length = std::strlen(te.loc->name);
            std::cout << "   |- " << te.loc->name << std::right << std::setw(50 - length);
            std::cout << cycles / (double)GIGA << "G" << " Cycles";
            std::cout << ", Time: " << cycles / (double)CLOCK << " ms";
            std::cout << ", Samples: " << te.samples << std::endl;
        }
    }

    void setValueStat(float& v, ct::scope& s) {
        auto trace = s.trace();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(trace.time_end() - trace.time_start()).count();
        v = elapsed / 1000.f;
    }

    int testMesh(ObjConfig& conf, std::string outputPath) {
        int testCount = 1;
        ct::scope rootScope;
        BenchMarkWriter benchmarkWriter(outputPath);
        std::cout << "#############################################################" << std::endl;
        std::cout << "#######                 Benchmark                     #######" << std::endl;
        std::cout << "#############################################################" << std::endl;

        //Load Mesh
        SharedPlaneMesh planeMesh1;
        SharedPlaneMesh planeMesh2;
        {
            ct::scope s("Load Mesh");
            planeMesh1 = conf.getMeshA();
            planeMesh2 = conf.getMeshB();
            setValueStat(benchmarkWriter.mTimeLoadMesh, s);
            printStats(s);            
        }
        //conf.viewMesh(true);
        if (!planeMesh1->allHalfEdgesAreValid())
            return 2;

        if (!planeMesh2->allHalfEdgesAreValid())
            return 2;
        //TG_ASSERT(!planeMesh1->allHalfEdgesAreValid());

        glow::timing::CpuTimer timer;
        //Load Octree

        SharedOctree octree;
        {
            ct::scope s("Build Octree");
            octree = conf.getOctree();
            setValueStat(benchmarkWriter.mTimeBuildOctree, s);
            printStats(s);            
        }
        //conf.viewMesh(true);
        //Cut Mesh
        IntersectionCut iCut;
        {
            ct::scope s("Cut Mesh");
            iCut = octree->cutPolygons();
            setValueStat(benchmarkWriter.mTimeCutMesh, s);
            printStats(s);           
        }
        {
            planeMesh1->checkAndComputePositions();
            planeMesh2->checkAndComputePositions();
            auto view = gv::view(planeMesh2->positions());
            gv::view(gv::lines(planeMesh2->positions()).line_width_world(3000), tg::color3::color(0.0));
            gv::view(gv::lines(planeMesh1->positions()).line_width_world(30000), gv::masked(iCut.getIntersectionEdgesMarkerA()), tg::color3::color(0.0));
            gv::view(planeMesh1->positions());
            gv::view(gv::lines(planeMesh1->positions()).line_width_world(3000), tg::color3::red);
        }
        //test = planeMesh1->allHalfEdgesAreValid();
        //Categorization
        std::shared_ptr<ComponentCategorization> components;
        {
            ct::scope s("Categorization");
            std::shared_ptr<FaceComponentFinder> components1 = std::make_shared<FaceComponentFinder>(*planeMesh1, iCut.getIntersectionEdgesMarkerA());
            std::shared_ptr<FaceComponentFinder> components2 = std::make_shared<FaceComponentFinder>(*planeMesh2, iCut.getIntersectionEdgesMarkerB());
            components = std::make_shared<ComponentCategorization>(octree, components1, components2, iCut);
            setValueStat(benchmarkWriter.mTimeCategorization, s);
            printStats(s);           
        }

        auto trace = rootScope.trace();
        auto c = trace.elapsed_cycles();
        auto m = std::chrono::duration_cast<std::chrono::milliseconds>(trace.time_end() - trace.time_start()).count();
        auto t = timer.elapsedMillisecondsD();
        benchmarkWriter.mTimeComplete = t + conf.initMeshTime();
        std::cout << std::endl;
        std::cout << "----------- Total ---------- " << std::endl;
        std::cout << "Time: " << m << "ms" << std::endl;
        std::cout << "Time CTracer: " << c / (double)CLOCK << "ms (Maybe not exact. Dependent on constant CPU frequence)" << std::endl;
        std::cout << "Time Without PM Load: " << t + conf.initMeshTime() << "ms" << std::endl;
        std::cout << "scope: " << rootScope.trace().elapsed_cycles() / (double)1000000000 << "G cycle" << std::endl;
        conf.getOctree()->printOctreeStats();
        benchmarkWriter.writeToFile();
        //conf.viewMesh(true);
        components->renderFinalResult(iCut);
        return 0;
    }
}