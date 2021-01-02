#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include "globject.h"
#include <vector>
#include <algorithm>

using K = CGAL::Exact_predicates_exact_constructions_kernel;
using E_Point = CGAL::Point_3<K>;
using E_Point_2 = CGAL::Point_2<K>;
using E_Plane = CGAL::Plane_3<K>;
using E_Line = CGAL::Line_3<K>;
using E_Line_2 = CGAL::Line_2<K>;
using E_Segment = CGAL::Segment_3<K>;
using E_Segment_2 = CGAL::Segment_2<K>;
using E_Vec = CGAL::Vector_3<K>;
using E_Polygon = std::pair<E_Plane, std::vector<E_Point>>;
using Polygon2 = CGAL::Polygon_2<K>;

// convert from float type to exact type
std::vector<E_Polygon> f2E_polygons(std::vector<Polygon> polygons)
{
    auto e_polygons = std::vector<E_Polygon>{};
    for (const auto &polygon : polygons)
    {
        auto e_points = std::vector<E_Point>{};
        for (const auto &p : polygon._verts)
        {
            e_points.push_back(E_Point{p.x, p.y, p.z});
        }
        assert(e_points.size() >= 3);
        // project to plane
        auto e_plane = E_Plane{e_points[0], e_points[1], e_points[2]};
        for (auto &p : e_points)
        {
            p = e_plane.projection(p);
        }
        e_polygons.push_back(std::make_pair(e_plane, e_points));
    }
    return e_polygons;
}

// convert from exact type to float type
std::vector<Polygon> E2f_polygons(std::vector<E_Polygon> e_polygons)
{
    auto polygons = std::vector<Polygon>{};
    for (const auto &detected_polygon : e_polygons)
    {
        auto verts = std::vector<Vec3>{};
        for (const auto &point : detected_polygon.second)
        {
            verts.push_back(Vec3{
                (float)CGAL::to_double(point.x()),
                (float)CGAL::to_double(point.y()),
                (float)CGAL::to_double(point.z()),
            });
        }
        polygons.push_back(Polygon{std::move(verts)});
    }
    return polygons;
}

std::optional<std::pair<E_Point, E_Point>> plane_polygon_intersect(const E_Plane &plane, const std::vector<E_Point> &points)
{
    auto segments = std::vector<E_Segment>{};
    auto num = points.size();
    assert(num >= 3);
    segments.push_back(E_Segment{points[num - 1], points[0]});
    for (size_t i = 1; i < num; i++)
    {
        segments.push_back(E_Segment{points[i - 1], points[i]});
    }
    auto inters = std::vector<E_Point>{};
    for (const auto &seg : segments)
    {
        // std::cout << seg << std::endl;
        // std::cout << plane << std::endl;
        auto result = CGAL::intersection(seg, plane);
        if (result)
        {
            if (auto inters_point = boost::get<E_Point>(&*result))
                inters.push_back(*inters_point);
            else if (auto inters_seg = boost::get<E_Segment>(&*result))
                std::cout << "coplaner in plane_polygon_intersect\n";
        }
    }
    if (inters.empty())
        return {};
    assert(inters.size() == 2);
    return std::make_optional(std::make_pair(inters[0], inters[1]));
}

E_Polygon Poly_2_to_E_Poly(Polygon2 poly_2, E_Plane plane)
{
    auto points = std::vector<E_Point>{};
    for (const auto &p_2 : poly_2.container())
    {
        points.push_back(plane.to_3d(p_2));
    }
    return std::make_pair(plane, points);
}

std::pair<Polygon2, Polygon2> split_polygon_2(Polygon2 polygon, E_Line_2 line)
{
    auto p_s = std::vector<std::pair<E_Point_2, E_Segment_2>>{};
    for (auto edge = polygon.edges_begin(); edge != polygon.edges_end(); edge++)
    {
        auto res = CGAL::intersection(*edge, line);
        if (res)
        {
            //todo: corner case
            if (auto inters_point = boost::get<E_Point_2>(&*res))
                p_s.push_back(std::make_pair(*inters_point, *edge));
        }
    }
    assert(p_s.size() == 2);
    auto poly1 = std::vector<E_Point_2>{}, poly2 = std::vector<E_Point_2>{};
    auto poly_p = &poly1;
    for (auto edge = polygon.edges_begin(); edge != polygon.edges_end(); edge++)
    {
        poly_p->push_back(edge->start());
        if (*edge == p_s[0].second)
        {
            assert(poly_p == &poly1);
            poly_p->push_back(p_s[0].first);
            poly_p = &poly2;
            poly_p->push_back(p_s[0].first);
        }
        else if (*edge == p_s[1].second)
        {
            assert(poly_p == &poly2);
            poly_p->push_back(p_s[1].first);
            poly_p = &poly1;
            poly_p->push_back(p_s[1].first);
        }
    }
    assert((poly1.size() + poly2.size()) == (polygon.size() + 4));
    return std::make_pair(Polygon2{poly1.begin(), poly1.end()}, Polygon2{poly2.begin(), poly2.end()});
}

// if p2 intersect p1, then spilt  p1 and return the two new polygon
std::optional<std::pair<E_Polygon, E_Polygon>> split_by(const E_Polygon &poly1, const E_Polygon &poly2)
{
    if (poly1.first == poly2.first) return {};
    auto inter_points = plane_polygon_intersect(poly1.first, poly2.second);
    if (!inter_points)
        return {};
    auto [p1, p2] = *inter_points;

    {
        auto res = CGAL::intersection(poly1.first, poly2.first);
        assert(res);
        auto line = boost::get<E_Line>(&*res);
        assert(line);
        assert(line->has_on(p1) && line->has_on(p1));
    }

    auto points_2 = std::vector<E_Point_2>{};
    for (const auto &point : poly1.second)
    {
        points_2.push_back(poly1.first.to_2d(point));
    }
    auto polygon_2 = Polygon2{points_2.begin(), points_2.end()};
    assert(polygon_2.is_simple());
    assert(polygon_2.is_convex());

    auto p1_on_poly1 = poly1.first.to_2d(p1);
    auto p2_on_poly1 = poly1.first.to_2d(p2);
    auto line_2 = E_Line_2{p1_on_poly1, p2_on_poly1};

    bool intersect = false;
    if (polygon_2.has_on_bounded_side(p1_on_poly1) ||
        polygon_2.has_on_bounded_side(p2_on_poly1)) //todo: corner case
    {
        intersect = true;
    }
    else
    {
        auto segment = E_Segment_2{p1_on_poly1, p2_on_poly1};
        for (auto edge = polygon_2.edges_begin(); edge != polygon_2.edges_end(); edge++)
        {
            //todo: corner case
            auto res = CGAL::intersection(*edge, segment);
            if (res)
            {
                intersect = true;
                break;
            }
        }
    }

    if (!intersect)
        return {};
    auto new_poly_2 = split_polygon_2(polygon_2, line_2);
    return std::make_pair(Poly_2_to_E_Poly(new_poly_2.first, poly1.first), Poly_2_to_E_Poly(new_poly_2.second, poly1.first));
}

//decompose polygons into intersection-free ones
std::vector<Polygon> decompose(std::vector<Polygon> polygons)
{

    auto e_polygons = f2E_polygons(polygons);

    auto decomposed_polygons = e_polygons;
    for (const auto &origin_p : e_polygons)
    {
        auto temp = std::vector<E_Polygon>{};
        for (const auto &decomposed_p : decomposed_polygons)
        {
            auto split = split_by(decomposed_p, origin_p);
            if (split)
            {
                temp.push_back(split->first);
                temp.push_back(split->second);
            }
            else
            {
                temp.push_back(decomposed_p);
            }
        }
        decomposed_polygons = std::move(temp);
    }

    return E2f_polygons(decomposed_polygons);
}
