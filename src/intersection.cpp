#include <algorithm>
#include "cgal_object.h"

std::optional<std::pair<Point_3, Point_3>> plane_polygon_intersect(const Plane_3 &plane, const Polygon_3 &polygon)
{
    auto segments_3 = polygon.edges_3();
    auto inters = std::vector<Point_3>{};
    for (const auto &seg : segments_3)
    {
        auto result = CGAL::intersection(seg, plane);
        if (result)
        {
            if (auto inters_point = boost::get<Point_3>(&*result))
                inters.push_back(*inters_point);
            else if (auto inters_seg = boost::get<Segment_3>(&*result))
                std::cout << "coplaner in plane_polygon_intersect\n";
        }
    }
    if (inters.empty())
        return {};
    assert(inters.size() == 2);
    return {{inters[0], inters[1]}};
}

std::pair<Polygon_2, Polygon_2> split_polygon_2(Polygon_2 polygon, Line_2 line)
{
    auto p_s = std::vector<std::pair<Point_2, Segment_2>>{};
    for (auto edge = polygon.edges_begin(); edge != polygon.edges_end(); edge++)
    {
        auto res = CGAL::intersection(*edge, line);
        if (res)
        {
            //todo: corner case
            if (auto inters_point = boost::get<Point_2>(&*res))
                p_s.push_back(std::make_pair(*inters_point, *edge));
        }
    }
    assert(p_s.size() == 2);
    auto poly1 = std::vector<Point_2>{}, poly2 = std::vector<Point_2>{};
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
    return {
        Polygon_2{poly1.begin(), poly1.end()},
        Polygon_2{poly2.begin(), poly2.end()},
    };
}

// if p2 intersect p1, then spilt  p1 and return the two new polygon
std::optional<std::pair<Polygon_3, Polygon_3>> split_by(const Polygon_3 &poly1, const Polygon_3 &poly2)
{
    if (poly1.plane() == poly2.plane())
        return {};
    auto inter_points = plane_polygon_intersect(poly1.plane(), poly2);
    if (!inter_points)
        return {};
    auto [p1, p2] = *inter_points;

    {
        auto res = CGAL::intersection(poly1.plane(), poly2.plane());
        assert(res);
        auto line = boost::get<Line_3>(&*res);
        assert(line);
        assert(line->has_on(p1) && line->has_on(p1));
    }

    auto polygon_2 = poly1.polygon_2();
    assert(polygon_2.is_simple());
    assert(polygon_2.is_convex());

    auto p1_on_poly1 = poly1.plane().to_2d(p1);
    auto p2_on_poly1 = poly1.plane().to_2d(p2);
    auto line_2 = Line_2{p1_on_poly1, p2_on_poly1};

    bool intersect = false;
    if (polygon_2.has_on_bounded_side(p1_on_poly1) ||
        polygon_2.has_on_bounded_side(p2_on_poly1)) //todo: corner case
    {
        intersect = true;
    }
    else
    {
        auto segment = Segment_2{p1_on_poly1, p2_on_poly1};
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
    return {{Polygon_3{poly1.plane(), new_poly_2.first},
             Polygon_3{poly1.plane(), new_poly_2.second}}};
}

//decompose polygons into intersection-free ones
std::vector<Polygon_3> decompose(const std::vector<Polygon_3> &polygons_3)
{

    auto decomposed_polygons = polygons_3;
    for (const auto &origin_p : polygons_3)
    {
        auto temp = std::vector<Polygon_3>{};
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

    return decomposed_polygons;
}