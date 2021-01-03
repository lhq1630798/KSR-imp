#pragma once

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include "globject.h"
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

class Polygon_3
{
    //Plane_3 + Polygon_2
    Plane_3 _plane;
    Polygon_2 _polygon;
public:
    Polygon_3(Plane_3 plane, Polygon_2 polygon)
        : _plane(plane), _polygon(polygon) {}
    Polygon_3(Plane_3 plane, std::vector<Point_2> points)
        : _plane(plane), _polygon(points.begin(), points.end()) {}
    explicit Polygon_3(std::vector<Vec3> points_f)
    {
        assert(points_f.size() >= 3);
        auto points_3 = std::vector<Point_3>{};
        for (const auto &p : points_f)
            points_3.push_back(Point_3{p.x, p.y, p.z});

        _plane = Plane_3{points_3[0], points_3[1], points_3[2]};
        auto points_2 = std::vector<Point_2>{};
        for (auto &p : points_3)
            points_2.push_back(_plane.to_2d(p));
        _polygon = Polygon_2{points_2.begin(), points_2.end()};
    }
    explicit Polygon_3(Polygon_Mesh polygon_f) : Polygon_3(polygon_f._verts) {}

    const std::vector<Point_2> &points_2() const { return _polygon.container(); }
    const Plane_3 &plane() const { return _plane; }
    const Polygon_2 &polygon_2() const { return _polygon; }
    std::vector<Point_3> points_3() const
    {
        auto points_3 = std::vector<Point_3>{};
        points_3.reserve(points_2().size());
        for (const auto &p : points_2())
            points_3.push_back(_plane.to_3d(p));
        return points_3;
    }
};