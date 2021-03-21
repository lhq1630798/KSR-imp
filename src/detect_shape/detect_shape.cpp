#include "cgal/cgal_object.h"
#include "region_growing.h"
#include "ransac.h"
#include "util/log.h"
#include "util/convex.h"

Polygons_3 detect_shape(const EPIC::Surface_Mesh &polygon_mesh, const DetectShape_Params &params)
{
	auto detected_shape = region_growing_on_mesh(polygon_mesh, params);

	//auto detected_shape = ransac(pwns);
	Polygons_3 results;
	for (const auto&[plane_3, pwn] : detected_shape)
	{
		std::vector<Point_2> projected_points;
		for (const auto &[point_3, normal] : pwn)
		{
			Point_3 projected = plane_3.projection(point_3);
			projected_points.push_back(plane_3.to_2d(projected));
		}

		auto polygon2 = get_convex(projected_points.begin(), projected_points.end());
		polygon2 = simplify_convex(polygon2);

		//for (const auto& p : projected_points)
		//	assert(!polygon2.has_on_unbounded_side(p));

		auto poly3 = Polygon_3{ plane_3, std::move(polygon2) };
		poly3.set_inline_points(pwn);
		results.push_back(std::move(poly3));
	}
	return results;
}

Polygons_3 detect_shape(const EPIC::Pwn_vector &pwns, const DetectShape_Params &params)
{
	auto detected_shape = region_growing_on_points(pwns, params);
	
	//auto detected_shape = ransac(pwns, params);
	Polygons_3 results;
	for (const auto& [plane_3, pwn] : detected_shape)
	{
		std::vector<Point_2> projected_points;
		for (const auto &[point_3, normal] : pwn)
		{
			Point_3 projected = plane_3.projection(point_3);
			projected_points.push_back(plane_3.to_2d(projected));
		}

		auto polygon2 = get_convex(projected_points.begin(), projected_points.end());
		polygon2 = simplify_convex(polygon2);
		
		//for (const auto& p : projected_points)
		//	assert(!polygon2.has_on_unbounded_side(p));

		auto poly3 = Polygon_3{ plane_3, std::move(polygon2) };
		poly3.set_inline_points(pwn);
		results.push_back(std::move(poly3));
	}
	return results;
}

