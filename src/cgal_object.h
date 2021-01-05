#pragma once

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/random_convex_hull_in_disc_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Random.h>
#include <vector>
#include <GLFW/glfw3.h>
#include "math/vec3.h"

using K = CGAL::Exact_predicates_exact_constructions_kernel;
// using K = CGAL::Simple_cartesian<CGAL::Gmpq>;
using Point_3 = CGAL::Point_3<K>;
using Point_2 = CGAL::Point_2<K>;
using Plane_3 = CGAL::Plane_3<K>;
using Line_3 = CGAL::Line_3<K>;
using Line_2 = CGAL::Line_2<K>;
using Segment_3 = CGAL::Segment_3<K>;
using Segment_2 = CGAL::Segment_2<K>;
using Vector_3 = CGAL::Vector_3<K>;
using Vector_2 = CGAL::Vector_2<K>;
using Polygon_2 = CGAL::Polygon_2<K>;
using FT = K::FT;

class Timer
{
public:
    bool enable = true;
    template <typename Callable, typename... Args>
    decltype(auto) operator()(const std::string &func_name, Callable &&func, Args &&... args)
    {
        if (!enable) return func(args...);
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
        : _plane(plane), _polygon_2(std::move(polygon)), _color(color) { update_points_3(); }
    Polygon_3(Plane_3 plane, const std::vector<Point_2> &points, Vec3 color = rand_color())
        : Polygon_3(plane, Polygon_2{points.begin(), points.end()}, color) {}

    const std::vector<Point_2> &points_2() const { return _polygon_2.container(); }
    const Plane_3 &plane() const { return _plane; }
    const Polygon_2 &polygon_2() const { return _polygon_2; }
    const std::vector<Point_3> &points_3() const { return _points_3; }
    std::vector<Segment_3> edges_3() const;
    std::optional<Line_2> intersect(const Polygon_3 &) const;
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
    Plane_3 _plane;
    Polygon_2 _polygon_2;
    std::vector<Point_3> _points_3;
};

// class K_Polygon_3
// {
//     enum mode {frozen,sliding,normal};
//     Polygon_3 _polygon_3;
//     std::vector<Vector_2> _speed;
//     std::vector<mode> _status;
// public:
//     explicit K_Polygon_3(Polygon_3 poly_3) : _polygon_3(poly_3)
//     {
//         auto center = Vector_2{};
//         for (const auto &point_2 : poly_3.points_2())
//             center += point_2 - CGAL::ORIGIN;
//         center = center / poly_3.points_2().size();
//         for (const auto &point_2 : poly_3.points_2()){
//             _speed.push_back(point_2 - (CGAL::ORIGIN + center));
//             _status.push_back(normal);
//         }
//     }
//     const Polygon_3 &polygon_3() const { return _polygon_3; }
//     void update(Polygon_3 poly_3)
//     {
//         _polygon_3 = std::move(poly_3);
//     }
//     Polygon_3 move_dt(FT t)
//     {
//         auto new_points_2 = std::vector<Point_2>{};
//         for (size_t i = 0; i < _polygon_3.points_2().size(); i++)
//             new_points_2.push_back(_polygon_3.points_2()[i] + _speed[i] * t);
//         return Polygon_3{_polygon_3.plane(), new_points_2, _polygon_3._color};
//     }
// };

class K_Polygon_3 : public Polygon_3
{
    enum mode {frozen,sliding,normal};
    std::vector<Vector_2> _speed;
    std::vector<mode> _status;
public:
    explicit K_Polygon_3(Polygon_3 poly_3) : Polygon_3(poly_3)
    {
        auto center = Vector_2{};
        for (const auto &point_2 : poly_3.points_2())
            center += point_2 - CGAL::ORIGIN;
        center = center / poly_3.points_2().size();
        for (const auto &point_2 : poly_3.points_2()){
            _speed.push_back(point_2 - (CGAL::ORIGIN + center));
            _status.push_back(normal);
        }
    }
    void update(Polygon_2 poly_2)
    {
        _polygon_2 = std::move(poly_2);
        update_points_3();
    }
    Polygon_2 move_dt(FT t)
    {
        auto new_points_2 = std::vector<Point_2>{};
        for (size_t i = 0; i < points_2().size(); i++)
            new_points_2.push_back(points_2()[i] + _speed[i] * t);
        return Polygon_2{new_points_2.begin(), new_points_2.end()};
    }
};

std::vector<Polygon_3> generate_poly_3(size_t num);
std::vector<Polygon_3> generate_box();
std::vector<Polygon_3> decompose(const std::vector<Polygon_3> &);