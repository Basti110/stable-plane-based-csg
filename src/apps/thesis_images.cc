#include <nexus/app.hh>
#include <intersection_cut.hh>
#include <obj_config.hh>
#include <octree.hh>
#include <face_component_finder.hh>
#include <polymesh/algorithms/deduplicate.hh>
#include <component_categorization.hh>
#include <iomanip>

APP("Image::Intersection") {
    PlaneMesh planeMesh1;
    PlaneMesh planeMesh2;
    planeMesh1.insertPolygon({ -10, 0, 0 }, { 10, 0, 0 }, { 0, 20, 0 });
    planeMesh2.insertPolygon({ -15, 10, -10 }, { -5, 10, 10 }, { 5, 10, -10 });
    auto view = gv::view(planeMesh1.positions(), gv::no_grid, "test");
    gv::label_style style;
    style.font = { "sans", 30 };
    style.line_width = 2.0f;
    style.pixel_distance = 50;
    gv::view(gv::lines(planeMesh1.positions()).line_width_world(0.1), tg::color3::color(0.0));
    gv::view(planeMesh2.positions());
    gv::view(gv::lines(planeMesh2.positions()).line_width_world(0.1), tg::color3::color(0.0));
    gv::view(gv::label{ "Face 1", style, tg::pos3{0, 15, 0} });
    gv::view(gv::label{ "Face 2", style, tg::pos3{-7, 10, -2} });
}

APP("Image::Touching_Face") {
    PlaneMesh planeMesh1;
    PlaneMesh planeMesh2;
    planeMesh1.insertPolygon({ -10, 0, 0 }, { 10, 0, 0 }, { 0, 20, 0 });
    planeMesh2.insertPolygon({ { -15, 10, -10 }, { -10, 10, 0 }, { 0, 10, 0 }, { 5, 10, -10 } });
    planeMesh2.insertPolygon({ { -10, 10, 0 }, { -5, 10, 10 }, { 0, 10, 0 }});
    //auto view = gv::view(planeMesh1.positions());
    //auto g = gv::grid();
    auto view = gv::view(planeMesh1.positions(), gv::no_grid, "test");
    gv::label_style style;
    style.font = { "sans", 30 };
    style.line_width = 2.0f;
    style.pixel_distance = 50;
    gv::view(gv::lines(planeMesh1.positions()).line_width_world(0.1), tg::color3::color(0.0), "test");
    gv::view(planeMesh2.positions());
    gv::view(gv::lines(planeMesh2.positions()).line_width_world(0.1), tg::color3::color(0.0));
    gv::view(gv::label{ "Face 1", style, tg::pos3{0, 15, 0} });
    gv::view(gv::label{ "Face 2", style, tg::pos3{-5, 10, -5} });
    gv::view(gv::label{ "Face 3", style, tg::pos3{-5, 10, 5} });
}