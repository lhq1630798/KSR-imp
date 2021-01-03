#include "cgal_object.h"
#include "gl_object.h"
Polygon_Mesh::Polygon_Mesh(std::vector<Polygon_3> polygons_3)
{
    for (const auto &poly : polygons_3)
    {
        auto verts = std::vector<Vec3>{};
        for (const auto &point : poly.points_3())
        {
            verts.push_back(Vec3{
                (float)CGAL::to_double(point.x()),
                (float)CGAL::to_double(point.y()),
                (float)CGAL::to_double(point.z()),
            });
        }
        _polygons.push_back(Polygon_GL{std::move(verts)});
    }
}

// Polygon_3::Polygon_3(std::vector<Vec3> points_f)
// {
//     assert(points_f.size() >= 3);
//     auto points_3 = std::vector<Point_3>{};
//     for (const auto &p : points_f)
//         points_3.push_back(Point_3{p.x, p.y, p.z});

//     _plane = Plane_3{points_3[0], points_3[1], points_3[2]};
//     auto points_2 = std::vector<Point_2>{};
//     for (auto &p : points_3)
//         points_2.push_back(_plane.to_2d(p));
//     _polygon = Polygon_2{points_2.begin(), points_2.end()};
//     init_points_3();
// }

void Polygon_3::init_points_3()
{
    _points_3.reserve(points_2().size());
    for (const auto &p : points_2())
        _points_3.push_back(_plane.to_3d(p));
}

std::vector<Segment_3> Polygon_3::edges_3() const
{
    auto segments_3 = std::vector<Segment_3>{};
    auto num = _points_3.size();
    assert(num >= 3);
    segments_3.push_back(Segment_3{_points_3[num - 1], _points_3[0]});
    for (size_t i = 1; i < num; i++)
        segments_3.push_back(Segment_3{_points_3[i - 1], _points_3[i]});
    return segments_3;
}

std::vector<Polygon_3> generate_poly_3(size_t num)
{
    auto rand = CGAL::Random{0};
    auto polys_3 = std::vector<Polygon_3>{};

    const double RADIUS = 1.0;
    int N = 5;
    boost::mt19937 gen;
    gen.seed(0u);
    while (num--)
    {
        std::vector<Point_2> v;
        auto plane = Plane_3{
            RT{rand.get_double(-1, 1)},
            RT{rand.get_double(-1, 1)},
            RT{rand.get_double(-1, 1)},
            RT{rand.get_double(-1, 0.5)}};
        random_convex_hull_in_disc_2(N, RADIUS, gen, std::back_inserter(v), K());
        polys_3.push_back(Polygon_3{plane, v});
    }

    return polys_3;
}