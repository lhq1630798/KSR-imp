#include "cgal_object.h"
#include "region_growing.h"

// region_growing.h does not need to be included in cgal_object.h

Polygons_3 detect_shape(std::string path)
{
    std::vector<Detected_shape> detected_shape = region_growing(path);
    Polygons_3 results;
    for (auto &[plane, pwn] : detected_shape)
    {

        auto plane_3 = Plane_3{
            FT{plane.a()},
            FT{plane.b()},
            FT{plane.c()},
            FT{plane.d()}};

        PWN_E inline_points;
        std::vector<Point_2> projected_points;
        for (auto [v, n] : pwn)
        {
            auto point_3 = Point_3{FT{v.x()}, FT{v.y()}, FT{v.z()}};
			auto normal = Vector_3{FT{n.x()}, FT{n.y()}, FT{n.z()} };
			inline_points.push_back(std::make_pair(point_3, normal));
            Point_3 projected = plane_3.projection(point_3);
            projected_points.push_back(plane_3.to_2d(projected));
        }

        auto polygon2 = get_convex(projected_points.begin(), projected_points.end());

        auto poly3 = Polygon_3{plane_3, std::move(polygon2)};
        poly3.set_inline_points(std::move(inline_points));
        results.push_back(std::move(poly3));
    }
    return results;
}

Polygon_2 get_convex(Points_2::const_iterator begin, Points_2::const_iterator end){
            //get convex point
        std::vector<Point_2> convex_points;
        CGAL::convex_hull_2(begin, end, std::back_inserter(convex_points));
        Polygon_2 polygon2 = Polygon_2(convex_points.begin(), convex_points.end());
        assert(polygon2.is_simple());
        assert(polygon2.is_convex());
        return polygon2;
}
