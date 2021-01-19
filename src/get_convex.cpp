#include "cgal_object.h"
#include "region_growing.h"

// region_growing.h does not need to be included in cgal_object.h

Polygons_3 get_convex(std::string path)
{
    std::vector<Detected_shape> detected_shape = region_growing(path);
    Polygons_3 results;
    for (auto &[plane, points] : detected_shape)
    {

        auto plane_3 = Plane_3{
            FT{plane.a()},
            FT{plane.b()},
            FT{plane.c()},
            FT{plane.d()}};

        std::vector<Point_3> points_3;
        std::vector<Point_2> projected_points;
        for (auto v : points)
        {
            auto point_3 = Point_3{FT{v.x()}, FT{v.y()}, FT{v.z()}};
            points_3.push_back(point_3);
            Point_3 projected = plane_3.projection(point_3);
            projected_points.push_back(plane_3.to_2d(projected));
        }

        //get convex point
        std::vector<Point_2> convex_points;
        CGAL::convex_hull_2(projected_points.begin(), projected_points.end(), std::back_inserter(convex_points));
        Polygon_2 polygon2 = Polygon_2(convex_points.begin(), convex_points.end());
        assert(polygon2.is_simple());
        assert(polygon2.is_convex());

        auto poly3 = Polygon_3{plane_3, polygon2};
        poly3.set_inline_points(std::move(points_3));
        results.push_back(std::move(poly3));
    }
    return results;
}