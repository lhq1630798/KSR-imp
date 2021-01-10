#include "cgal_object.h"

Segments_3 Polygon_3::edges_3() const
{
    auto segments_3 = Segments_3{};
    auto num = _points_3.size();
    assert(num >= 3);
    segments_3.push_back(Segment_3{_points_3[num - 1], _points_3[0]});
    for (size_t i = 1; i < num; i++)
        segments_3.push_back(Segment_3{_points_3[i - 1], _points_3[i]});
    return segments_3;
}

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
    auto float_min = std::numeric_limits<float>::min();
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
    // {
    //     auto plane = Plane_3{1, 0, 0, -S};
    //     Points_2 v;
    //     random_convex_hull_in_disc_2(N, RADIUS, gen, std::back_inserter(v), K());
    //     for (auto &point_2 : v)
    //         point_2 = point_2 + Vector_2{rand.get_double(0, 1) * float_min, rand.get_double(0, 1) * float_min};
    //     polys_3.push_back(Polygon_3{plane, v});
    // }
    // {
    //     auto plane = Plane_3{0, 1, 0, S};
    //     Points_2 v;
    //     random_convex_hull_in_disc_2(N, RADIUS, gen, std::back_inserter(v), K());
    //     for (auto &point_2 : v)
    //         point_2 = point_2 + Vector_2{rand.get_double(0, 1) * float_min, rand.get_double(0, 1) * float_min};
    //     polys_3.push_back(Polygon_3{plane, v});
    // }
    // {
    //     auto plane = Plane_3{0, 1, 0, -S};
    //     Points_2 v;
    //     random_convex_hull_in_disc_2(N, RADIUS, gen, std::back_inserter(v), K());
    //     for (auto &point_2 : v)
    //         point_2 = point_2 + Vector_2{rand.get_double(0, 1) * float_min, rand.get_double(0, 1) * float_min};
    //     polys_3.push_back(Polygon_3{plane, v});
    // }
    // {
    //     auto plane = Plane_3{0, 0, 1, S};
    //     Points_2 v;
    //     random_convex_hull_in_disc_2(N, RADIUS, gen, std::back_inserter(v), K());
    //     for (auto &point_2 : v)
    //         point_2 = point_2 + Vector_2{rand.get_double(0, 1) * float_min, rand.get_double(0, 1) * float_min};
    //     polys_3.push_back(Polygon_3{plane, v});
    // }
    // {
    //     auto plane = Plane_3{0, 0, 1, -S};
    //     Points_2 v;
    //     random_convex_hull_in_disc_2(N, RADIUS, gen, std::back_inserter(v), K());
    //     for (auto &point_2 : v)
    //         point_2 = point_2 + Vector_2{rand.get_double(0, 1) * float_min, rand.get_double(0, 1) * float_min};
    //     polys_3.push_back(Polygon_3{plane, v});
    // }
    return polys_3;
}

Polygons_3 get_convex(std::string path)
{
    std::vector<Detected_shape> detected_shape = region_growing(path);
    Polygons_3 results;
    for (auto shape : detected_shape)
    {
        Plane p = shape.first;
        auto plane = Plane_3{
            FT{p.a()},
            FT{p.b()},
            FT{p.c()},
            FT{p.d()}};

        std::vector<Point_2> points2;
        for (auto v : shape.second)
        {
            auto point = Point_3{FT{v.x()}, FT{v.y()}, FT{v.z()}};
            Point_3 project_point = plane.projection(point);
            points2.push_back(plane.to_2d(project_point));
        }

        //get convex point
        std::vector<Point_2> convex_points;
        CGAL::convex_hull_2(points2.begin(), points2.end(), std::back_inserter(convex_points));
        Polygon_2 polygon2 = Polygon_2(convex_points.begin(), convex_points.end());
        assert(polygon2.is_simple());
        results.push_back(Polygon_3{plane, polygon2});
    }
    return results;
}

// void inf_perturb(Polygons_3 &polygons_3){
//     auto inf = std::numeric_limits<float>::min();
//     for (auto &poly_3 : polygons_3)
//         for (auto &point_2 : poly_3.points_2()){

//         }
// }