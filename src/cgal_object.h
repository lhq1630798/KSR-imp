#pragma once

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/random_convex_hull_in_disc_2.h>
#include <CGAL/Polygon_2.h>
#include <vector>

using K = CGAL::Exact_predicates_exact_constructions_kernel;
using Point_3 = CGAL::Point_3<K>;
using Point_2 = CGAL::Point_2<K>;
using Plane_3 = CGAL::Plane_3<K>;
using Line_3 = CGAL::Line_3<K>;
using Line_2 = CGAL::Line_2<K>;
using Segment_3 = CGAL::Segment_3<K>;
using Segment_2 = CGAL::Segment_2<K>;
using Vector_3 = CGAL::Vector_3<K>;
using Polygon_2 = CGAL::Polygon_2<K>;
using RT = K::RT;

class Polygon_3
{
    //Plane_3 + Polygon_2
    Plane_3 _plane;
    Polygon_2 _polygon;
    std::vector<Point_3> _points_3;
    void Polygon_3::init_points_3();

public:
    Polygon_3(Plane_3 plane, Polygon_2 polygon)
        : _plane(plane), _polygon(polygon) { init_points_3(); }
    Polygon_3(Plane_3 plane, std::vector<Point_2> points)
        : _plane(plane), _polygon(points.begin(), points.end()) { init_points_3(); }
    // explicit Polygon_3(std::vector<Vec3>);
    // explicit Polygon_3(Polygon_GL polygon_GL) : Polygon_3(polygon_GL._verts) {}

    const std::vector<Point_2> &points_2() const { return _polygon.container(); }
    const Plane_3 &plane() const { return _plane; }
    const Polygon_2 &polygon_2() const { return _polygon; }
    const std::vector<Point_3> &points_3() const { return _points_3; }
    std::vector<Segment_3> edges_3() const;
};

std::vector<Polygon_3> generate_poly_3(size_t num);
std::vector<Polygon_3> decompose(const std::vector<Polygon_3> &);