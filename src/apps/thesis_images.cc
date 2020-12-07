#include <nexus/app.hh>
#include <intersection_cut.hh>
#include <obj_config.hh>
#include <octree.hh>
#include <face_component_finder.hh>
#include <polymesh/algorithms/deduplicate.hh>
#include <component_categorization.hh>
#include <glow-extras/viewer/canvas.hh>
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

APP("Image::BasePlaneTest1") {
    auto c = gv::canvas();
    tg::pos3 pos1 = { -7, 0, -7 };
    tg::pos3 pos2 = { 7, 0, -7 };
    tg::pos3 pos3 = { 0, 0, 7 };
    c.add_face(tg::triangle3{ {-10, -6, 0}, {-10, 6, 0}, {10, 6, 0} });
    c.add_face(tg::triangle3{ {10, 6, 0}, {10, -6, 0}, {-10, -6, 0} });
    c.add_face(tg::triangle3{ pos1, pos2, pos3 });
    c.set_line_width_px(2);
    c.add_line(pos1, tg::pos3{ -7, 0, -0.1 }, tg::color3::blue);
    c.add_line(pos2, tg::pos3{ 7, 0, -0.1 }, tg::color3::blue);
    c.add_line(pos3, tg::pos3{ 0, 0, 0 }, tg::color3::blue);

    gv::label_style style;
    style.font = { "sans", 30 };
    style.line_width = 2.0f;
    style.line_start_distance = 0;
    style.pixel_distance = 50;
    c.add_label(pos1, "sign -1", style);
    c.add_label(pos2, "sign -1", style);
    c.add_label(pos3, "sign 1", style);
    c.add_label(tg::pos3{ -10, 5, 0 }, "Base Plane", style);
    c.add_arrow(tg::pos3{ 0, 2, 0.1 }, tg::pos3{ 0, 2, 8 }, 0.5f, tg::color3::red);
}

APP("Image::BasePlaneTest2") {
    auto c = gv::canvas();
    tg::pos3 pos1 = { -7, 0, 1 };
    tg::pos3 pos2 = { 7, 0, 1 };
    tg::pos3 pos3 = { 0, 0, 10 };
    c.add_face(tg::triangle3{ {-10, -6, 0}, {-10, 6, 0}, {10, 6, 0} });
    c.add_face(tg::triangle3{ {10, 6, 0}, {10, -6, 0}, {-10, -6, 0} });
    c.add_face(tg::triangle3{ pos1, pos2, pos3 });
    c.set_line_width_px(2);
    c.add_line(pos1, tg::pos3{ -7, 0, -0.1 }, tg::color3::blue);
    c.add_line(pos2, tg::pos3{ 7, 0, -0.1 }, tg::color3::blue);
    c.add_line(pos3, tg::pos3{ 0, 0, 0 }, tg::color3::blue);

    gv::label_style style;
    style.font = { "sans", 30 };
    style.line_width = 2.0f;
    style.line_start_distance = 0;
    style.pixel_distance = 50;
    c.add_label(pos1, "sign 1", style);
    c.add_label(pos2, "sign 1", style);
    c.add_label(pos3, "sign 1", style);
    c.add_label(tg::pos3{ -10, 5, 0 }, "Base Plane", style);
    c.add_arrow(tg::pos3{ 0, 2, 0.1 }, tg::pos3{ 0, 2, 8 }, 0.5f, tg::color3::red);
}

APP("Image::BasePlaneTest3") {
    auto c = gv::canvas();
    tg::pos3 pos1 = { -7, 5, 0 };
    tg::pos3 pos2 = { 7, 5, 0 };
    tg::pos3 pos3 = { 0, -5, 0 };
    c.add_face(tg::triangle3{ {-10, -6, 0}, {-10, 6, 0}, {10, 6, 0} });
    c.add_face(tg::triangle3{ {10, 6, 0}, {10, -6, 0}, {-10, -6, 0} });
    c.add_face(tg::triangle3{ pos1, pos2, pos3 });
    c.set_line_width_px(2);
    c.add_line(pos1, pos2, tg::color3::black);
    c.add_line(pos2, pos3, tg::color3::black);
    c.add_line(pos3, pos1, tg::color3::black);

    gv::label_style style;
    style.font = { "sans", 30 };
    style.line_width = 2.0f;
    style.line_start_distance = 0;
    style.pixel_distance = 50;
    c.add_label(pos1, "sign 0", style);
    c.add_label(pos2, "sign 0", style);
    c.add_label(pos3, "sign 0", style);
    c.add_label(tg::pos3{ -10, 5, 0 }, "Base Plane", style);
    c.add_arrow(tg::pos3{ 0, 2, 0.1 }, tg::pos3{ 0, 2, 8 }, 0.5f, tg::color3::red);
}