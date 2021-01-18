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
            return func(std::forward<Args>(args)...);
        struct time
        {
            double start_time;
            const std::string &_name;
            time(const std::string &func_name) : _name(func_name), start_time(glfwGetTime()) {}
            ~time() { std::cout << "time for " + _name + " : " << glfwGetTime() - start_time << "s" << std::endl; }
        } t(func_name);
        return func(std::forward<Args>(args)...);
    }
};


Vec3 rand_color();


class Polygon_3
{
    //Plane_3 + Polygon_2
public:
    Polygon_3(Plane_3 plane, Polygon_2 polygon, Vec3 color = rand_color())
        : _plane(plane), _polygon_2(std::move(polygon)), _color(color)
    {
        update_points_3();
        // assert(_polygon_2.is_simple());
        assert(_polygon_2.is_convex());
    }
    Polygon_3(Plane_3 plane, const Points_2 &points, Vec3 color = rand_color())
        : Polygon_3(plane, Polygon_2{points.begin(), points.end()}, color) {}

    size_t size() const {return points_2().size();};
    const Points_2 &points_2() const { return _polygon_2.container(); }
    const Plane_3 &plane() const { return _plane; }
    const Polygon_2 &polygon_2() const { return _polygon_2; }
    const Points_3 &points_3() const { return _points_3; }


    Vec3 _color;

private: 
    void update_points_3()
    {
        _points_3.clear();
        _points_3.reserve(size());
        for (const auto &p : points_2())
            _points_3.push_back(_plane.to_3d(p));
    }

    // _plane, _polygon_2, _points_3 must be consistent with each other !
    Plane_3 _plane;
    Polygon_2 _polygon_2;
    Points_3 _points_3;
};


Polygons_3 generate_rand_polys_3(size_t num);
Polygons_3 generate_polys_3();
Polygons_3 get_convex(std::string path);





