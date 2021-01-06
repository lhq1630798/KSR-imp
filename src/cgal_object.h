#pragma once

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/random_convex_hull_in_disc_2.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Random.h>
#include <vector>
#include <GLFW/glfw3.h>
#include "math/vec3.h"
#include "region_growing.h"

using K = CGAL::Exact_predicates_exact_constructions_kernel;
// using K = CGAL::Simple_cartesian<CGAL::Gmpq>;
using Point_2 = CGAL::Point_2<K>;
using Points_2 = std::vector<Point_2>;
using Point_3 = CGAL::Point_3<K>;
using Points_3 = std::vector<Point_3>;
using Plane_3 = CGAL::Plane_3<K>;
using Line_2 = CGAL::Line_2<K>;
using Line_3 = CGAL::Line_3<K>;
using Ray_2 = CGAL::Ray_2<K>;
using Segment_2 = CGAL::Segment_2<K>;
using Segments_2 = std::vector<Segment_2>;
using Segment_3 = CGAL::Segment_3<K>;
using Segments_3 = std::vector<Segment_3>;
using Vector_2 = CGAL::Vector_2<K>;
using Vector_3 = CGAL::Vector_3<K>;
using Polygon_2 = CGAL::Polygon_2<K>;
class Polygon_3;
using Polygons_3 = std::vector<Polygon_3>;
using FT = K::FT;

class Timer
{
public:
    bool enable = true;
    template <typename Callable, typename... Args>
    decltype(auto) operator()(const std::string &func_name, Callable &&func, Args &&... args)
    {
        if (!enable)
            return func(args...);
        struct time
        {
            double start_time;
            const std::string &_name;
            time(const std::string &func_name) : _name(func_name), start_time(glfwGetTime()) {}
            ~time() { std::cout << "time for " + _name + " : " << glfwGetTime() - start_time << "s" << std::endl; }
        } t(func_name);
        return func(args...);
    }
};

class Polygon_3
{
    //Plane_3 + Polygon_2
public:
    Polygon_3(Plane_3 plane, Polygon_2 polygon, Vec3 color = rand_color())
        : _plane(plane), _polygon_2(std::move(polygon)), _color(color)
    {
        update_points_3();
        assert(_polygon_2.is_simple());
        assert(_polygon_2.is_convex());
    }
    Polygon_3(Plane_3 plane, const Points_2 &points, Vec3 color = rand_color())
        : Polygon_3(plane, Polygon_2{points.begin(), points.end()}, color) {}

    const Points_2 &points_2() const { return _polygon_2.container(); }
    const Plane_3 &plane() const { return _plane; }
    const Polygon_2 &polygon_2() const { return _polygon_2; }
    const Points_3 &points_3() const { return _points_3; }
    Segments_3 edges_3() const;
    std::optional<Line_2> intersect(const Polygon_3 &) const;

	bool has_on(const Point_3 &point_3) const
	{
		if (!plane().has_on(point_3))
			return false;
		return !polygon_2().has_on_unbounded_side(plane().to_2d(point_3));
	}

    Point_2 project_2(const Point_3 &point_3) const
    {
        return plane().to_2d(point_3);
    }
    Line_2 project_2(const Line_3 &line_3) const
    {
        auto p1 = project_2(line_3.point(0)), p2 = project_2(line_3.point(1));
        return Line_2{p1, p2};
    }
    Segment_2 project_2(const Segment_3 &segment_3) const
    {
        auto p1 = project_2(segment_3.point(0)), p2 = project_2(segment_3.point(1));
        return Segment_2{p1, p2};
    }
    Polygon_2 project_2(const Polygon_3 &polygon_3) const
    {
        auto points_2 = Points_2{};
        for (const auto &p_3 : polygon_3.points_3())
            points_2.push_back(project_2(p_3));
        return Polygon_2{points_2.begin(), points_2.end()};
    }
    Vec3 _color;

private:
    static Vec3 rand_color()
    {
        static auto color_rand = CGAL::Random{0};
        return Vec3{(float)color_rand.get_double(0, 0.8),
                    (float)color_rand.get_double(0.2, 1),
                    (float)color_rand.get_double(0.2, 1)};
    }

protected:
    void update_points_3()
    {
        _points_3.clear();
        _points_3.reserve(points_2().size());
        for (const auto &p : points_2())
            _points_3.push_back(_plane.to_3d(p));
    }
    // _plane, _polygon_2, _points_3 must be consistent with each other !
    // Derived class must be responsible for that !
    Plane_3 _plane;
    Polygon_2 _polygon_2;
    Points_3 _points_3;
};


Polygons_3 generate_poly_3(size_t num);
Polygons_3 generate_box();
Polygons_3 decompose(const Polygons_3 &);
Polygons_3 get_convex(std::string path);

inline bool line_polygon_intersect_2(const Line_2 &line, const Polygon_2 &poly)
{
    for (auto edge = poly.edges_begin(); edge != poly.edges_end(); edge++)
    {
        if (auto res = CGAL::intersection(*edge, line))
            return true;
    }
    return false;
}
inline bool segment_polygon_intersect_2(const Segment_2 &segment_2, const Polygon_2 &poly)
{
    // auto segment = Polygon_2{};
    // segment.push_back(segment_2.point(0));
    // segment.push_back(segment_2.point(1));
	// return CGAL::do_intersect(segment, poly);
    for (auto edge = poly.edges_begin(); edge != poly.edges_end(); edge++)
    {
        if (auto res = CGAL::intersection(*edge, segment_2))
            return true;
    }
    return false;
}

inline Point_2 project_line_2(const Line_2 &line_2, const Point_2 &point_2)
{
    return line_2.projection(point_2);
}
inline Segment_2 project_line_2(const Line_2 &line_2, const Segment_2 &segment_2)
{
    auto p1 = project_line_2(line_2, segment_2.point(0)), p2 = project_line_2(line_2, segment_2.point(1));
    return Segment_2{p1, p2};
}
inline Segment_2 project_line_2(const Line_2 &line_2, const Polygon_2 &polygon_2)
{
    auto points_2 = Points_2{};
    for (const auto &p_2 : polygon_2.container())
        points_2.push_back(project_line_2(line_2, p_2));
    return Segment_2{*CGAL::left_vertex_2(points_2.begin(), points_2.end()),
                     *CGAL::right_vertex_2(points_2.begin(), points_2.end())};
}

void check_intersect_free(Polygons_3 &);