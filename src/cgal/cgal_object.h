#pragma once

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/random_convex_hull_in_disc_2.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/property_map.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Random.h>
#include <vector>

#include "glm/vec3.hpp"
using Vec3 = glm::vec3;

inline Vec3 rand_color()
{
    static auto color_rand = CGAL::Random{ 0 };
    return Vec3{ (float)color_rand.get_double(0, 0.8),
                (float)color_rand.get_double(0.2, 1),
                (float)color_rand.get_double(0.2, 1) };
}

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
using Direction_3 = CGAL::Direction_3<K>;
using Direction_2 = CGAL::Direction_2<K>;
using PWN = std::pair< Point_3, Vector_3>;
using PWN_vector = std::vector<PWN>;
using Detected_shape = std::pair<Plane_3, std::vector<PWN>>;

namespace EPIC
{ // inexact_constructions_kernel
	using EPIC_K = CGAL::Exact_predicates_inexact_constructions_kernel;
	using in_Point = EPIC_K::Point_3;
	using in_Vector = EPIC_K::Vector_3;
	using in_FT = EPIC_K::FT;
	using Point_with_normal = std::pair<in_Point, in_Vector>;
	using Pwn_vector = std::vector<Point_with_normal>;
	using Point_map = CGAL::First_of_pair_property_map<Point_with_normal>;
	using Normal_map = CGAL::Second_of_pair_property_map<Point_with_normal>;
	// converter
	using IK_to_EK = CGAL::Cartesian_converter<EPIC_K, K>;
	using EK_to_IK = CGAL::Cartesian_converter<K, EPIC_K>;

	using Surface_Mesh = CGAL::Surface_mesh<in_Point>;
	using vertex_descriptor = Surface_Mesh::Vertex_index;
	using face_descriptor = Surface_Mesh::Face_index;
	using Vertex_index = CGAL::SM_Vertex_index;
    
}



class Polygon_3
{
    //Plane_3 + Polygon_2
public:
    Polygon_3(Plane_3 plane, Polygon_2 polygon, Vec3 color = rand_color())
        : _plane(plane), _polygon_2(std::move(polygon)), _color(color)
    {
        update_points_3();
        // assert(_polygon_2.is_simple());
        //assert(_polygon_2.is_convex());
    }
    Polygon_3(Plane_3 plane, const Points_2 &points, Vec3 color = rand_color())
        : Polygon_3(plane, Polygon_2{points.begin(), points.end()}, color) {}

    void set_inline_points(PWN_vector points)
    {
        inline_points = std::move(points);
    }
    size_t size() const { return points_2().size(); };
    const Points_2 &points_2() const { return _polygon_2.container(); }
    const Plane_3 &plane() const { return _plane; }
    const Polygon_2 &polygon_2() const { return _polygon_2; }
    const Points_3 &points_3() const { return _points_3; }

    Vec3 _color;
	PWN_vector inline_points;

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


std::optional<std::pair<Point_3, Point_3>> plane_polygon_intersect_3(const Plane_3 &plane, const Polygon_3 &polygon);
