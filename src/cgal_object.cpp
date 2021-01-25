#include "cgal_object.h"

Vec3 rand_color()
{
    static auto color_rand = CGAL::Random{0};
    return Vec3{(float)color_rand.get_double(0, 0.8),
                (float)color_rand.get_double(0.2, 1),
                (float)color_rand.get_double(0.2, 1)};
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