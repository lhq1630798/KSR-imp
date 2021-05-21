#include <CGAL/random_convex_hull_in_disc_2.h>
#include "cgal/cgal_object.h"
#include "util/convex.h"

#include "util/log.h"
#undef assert
#define assert(expr) R_assert(expr)

namespace EC {

Polygons_3 generate_rand_polys_3(size_t num)
{
    auto rand = CGAL::Random{0};
    auto polys_3 = Polygons_3{};

    const double RADIUS = 1.0;
    int N = 5;
    boost::mt19937 gen;
    gen.seed(0u);
    while (num--)
    {
        Points_2 v;
        auto plane = Plane_3{
            FT{rand.get_double(-1, 1)},
            FT{rand.get_double(-1, 1)},
            FT{rand.get_double(-1, 1)},
            FT{rand.get_double(-0.5, 0.5)}};
        random_convex_hull_in_disc_2(N, RADIUS, gen, std::back_inserter(v), K());
        polys_3.push_back(Polygon_3{plane, v});
    }

    return polys_3;
}

Polygons_3 generate_polys_3()
{
    constexpr auto float_min = std::numeric_limits<float>::min();
    auto rand = CGAL::Random{0};
    auto polys_3 = Polygons_3{};
    const double RADIUS = 0.5;
    const double S = 0.5;
    int N = 15;
    boost::mt19937 gen;
    gen.seed(0u);
    {
        auto plane = Plane_3{1, 0, 0, S};
        Points_2 v;
        random_convex_hull_in_disc_2(N, RADIUS, gen, std::back_inserter(v), K());
        for (auto &point_2 : v)
            point_2 = point_2 + Vector_2{rand.get_double(0, 1) * float_min, rand.get_double(0, 1) * float_min};
        polys_3.push_back(Polygon_3{plane, v});
    }
    {
        auto plane = Plane_3{1, 0, 0, -S};
        Points_2 v;
        random_convex_hull_in_disc_2(N, RADIUS, gen, std::back_inserter(v), K());
        for (auto &point_2 : v)
            point_2 = point_2 + Vector_2{rand.get_double(0, 1) * float_min, rand.get_double(0, 1) * float_min};
        polys_3.push_back(Polygon_3{plane, v});
    }
    {
        auto plane = Plane_3{0, 1, 0, S};
        Points_2 v;
        random_convex_hull_in_disc_2(N, RADIUS, gen, std::back_inserter(v), K());
        for (auto &point_2 : v)
            point_2 = point_2 + Vector_2{rand.get_double(0, 1) * float_min, rand.get_double(0, 1) * float_min};
        polys_3.push_back(Polygon_3{plane, v});
    }
   {
        auto plane = Plane_3{0, 1, 0, -S};
        Points_2 v;
        random_convex_hull_in_disc_2(N, RADIUS, gen, std::back_inserter(v), K());
        for (auto &point_2 : v)
            point_2 = point_2 + Vector_2{rand.get_double(0, 1) * float_min, rand.get_double(0, 1) * float_min};
        polys_3.push_back(Polygon_3{plane, v});
    }
    {
        auto plane = Plane_3{0, 0, 1, S};
        Points_2 v;
        random_convex_hull_in_disc_2(N, RADIUS, gen, std::back_inserter(v), K());
        for (auto &point_2 : v)
            point_2 = point_2 + Vector_2{rand.get_double(0, 1) * float_min, rand.get_double(0, 1) * float_min};
        polys_3.push_back(Polygon_3{plane, v});
    }
    {
        auto plane = Plane_3{0, 0, 1, -S};
        Points_2 v;
        random_convex_hull_in_disc_2(N, RADIUS, gen, std::back_inserter(v), K());
        for (auto &point_2 : v)
            point_2 = point_2 + Vector_2{rand.get_double(0, 1) * float_min, rand.get_double(0, 1) * float_min};
        polys_3.push_back(Polygon_3{plane, v});
    }

    // polys_3.erase(polys_3.begin());
    return polys_3;
}



// void inf_perturb(Polygons_3 &polygons_3){
//     auto inf = std::numeric_limits<float>::min();
//     for (auto &poly_3 : polygons_3)
//         for (auto &point_2 : poly_3.points_2()){

//         }
// }

Polygon_3::Polygon_3(Plane_3 plane, Polygon_2 polygon, Vec3 color)
    : _plane(plane), _polygon_2(std::move(polygon)), _color(color)
{
    update_points_3();
    // assert(_polygon_2.is_simple());
    //assert(_polygon_2.is_convex());
}

Segments_3 Polygon_3::edges_3() const
{
    auto segments_3 = Segments_3{};
    auto num = points_3().size();
    assert(num >= 3);
    for (size_t i = 1; i < num; i++)
        segments_3.push_back(Segment_3{ points_3()[i - 1], points_3()[i] }); //edges_3[0] corresponds to points_3[0,1]
    segments_3.push_back(Segment_3{ points_3()[num - 1], points_3()[0] });
    return segments_3;
}


std::optional<std::pair<Polygon_3, Polygon_3>> Polygon_3::split_by_plane(Plane_3 cutting_plane) const
{
    if (plane() == cutting_plane || plane().opposite() == cutting_plane)
        return {};
    auto edges = edges_3();
    std::vector<Point_2> poly1, poly2;
    auto *new_poly = &poly1;
    for (auto e : edges) {
        new_poly->push_back(plane().to_2d(e.point(0)));
        if (auto result = CGAL::intersection(e, cutting_plane))
        {
            if (auto inters_point = boost::get<Point_3>(&*result))
            {
                auto point_2 = plane().to_2d(*inters_point);
                new_poly->push_back(point_2);
                if (new_poly == &poly1) 
                    new_poly = &poly2;
                else 
                    new_poly = &poly1;
                new_poly->push_back(point_2);
            }
            else if (auto inters_seg = boost::get<Segment_3>(&*result)) {
                std::cout << "Warning Segment_3 on plane\n";
                return {};
            }
        }
    }
    if (poly2.empty()) return {};
    assert((poly1.size() + poly2.size()) == (4 + edges.size()));
    Polygon_3 new_poly1{ plane(), poly1 }, new_poly2{ plane(), poly2 };
    //split inline points
    for (auto& [point_3, normal] : inline_points)
    {
        auto point_2 = plane().to_2d(point_3);
        if (new_poly1.polygon_2().has_on_bounded_side(point_2))
        {
            assert(!new_poly2.polygon_2().has_on_bounded_side(point_2));
            new_poly1.inline_points.emplace_back(point_3, normal);
        }
        else if (new_poly2.polygon_2().has_on_bounded_side(point_2))
        {
            new_poly2.inline_points.emplace_back(point_3, normal);
        }
    }

    return { { new_poly1, new_poly2 } };
}

void Polygon_3::update_points_3()
{
    _points_3.clear();
    _points_3.reserve(size());
    for (const auto& p : points_2())
        _points_3.push_back(_plane.to_3d(p));

    // center
    Vector_2 center_V = CGAL::NULL_VECTOR;
    for (const auto& point_2 : _polygon_2.container())
        center_V += point_2 - CGAL::ORIGIN;
    center_V = center_V / _polygon_2.size();
    auto center_P = CGAL::ORIGIN + center_V;
    _center = plane().to_3d(center_P);
}

}// namaspace EC