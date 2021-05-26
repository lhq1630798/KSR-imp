#include "cgal/cgal_object.h"
#include "region_growing.h"
#include "ransac.h"
//#include "util/log.h"
#include "util/convex.h"
#include <fmt/core.h>

#include <CGAL/Alpha_shape_2.h>
#include <CGAL/Alpha_shape_vertex_base_2.h>
#include <CGAL/Alpha_shape_face_base_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>

#include <CGAL/Partition_traits_2.h>
#include <CGAL/partition_2.h>


EC::Polygons_3 detect_convexShape(std::vector<EC::Detected_shape>& detected_shape)
{
	using namespace EC;
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
		//for (const auto& p : projected_points)
		//	assert(!polygon2.has_on_unbounded_side(p));

		//polygon2 = simplify_convex(polygon2);
		//if (polygon2.size() <= 2) { //discard degenerate polygon after simplification
		//	fmt::print("discard {}-edge polygon\n", polygon2.size());
		//	continue;
		//}
		auto poly3 = Polygon_3{ plane_3, std::move(polygon2) };
		poly3.set_inline_points(pwn);
		results.push_back(std::move(poly3));
	}
	return results;
}


typedef CGAL::Alpha_shape_vertex_base_2<IC::K>                   Vb;
typedef CGAL::Alpha_shape_face_base_2<IC::K>                     Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb>                    Tds;
typedef CGAL::Delaunay_triangulation_2<IC::K, Tds>               Triangulation_2;
typedef CGAL::Alpha_shape_2<Triangulation_2>                            Alpha_shape_2;

typedef CGAL::Exact_predicates_tag                                                       Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<IC::K, CGAL::Default, Itag>    CDT;


void CDTriangulation(CDT& cdt, std::map<IC::Point_2, CDT::Vertex_handle>& p_index, Alpha_shape_2& as) {
	
	//insert vertex
	auto vit = as.alpha_shape_vertices_begin();
	while (vit != as.alpha_shape_vertices_end()) {
		CDT::Vertex_handle v = cdt.insert((*vit)->point());
		p_index[(*vit)->point()] = v;
		vit++;
	}

	//insert edge
	auto eit = as.alpha_shape_edges_begin();
	while (eit != as.alpha_shape_edges_end()) {
		switch (as.classify(*eit))
		{
			case Alpha_shape_2::SINGULAR:
				eit++;
				continue;
			default:
				break;
		}
		CDT::Vertex_handle v1 = p_index[as.segment(*eit).source()];
		CDT::Vertex_handle v2 = p_index[as.segment(*eit).target()];
		
		if (!v1->is_valid() || !v2->is_valid())
			std::cout << "invalid!" << std::endl;

		cdt.insert_constraint(v1, v2);
		eit++;
	}
	
}

std::list<IC::Triangle_3> Triangles_of_alphaShape(const std::vector<EC::Detected_shape>& detected_shape, float scale) {
	std::list<IC::Triangle_3> triangles;

	
	EK_to_IK to_inexact;
	for (const auto&[plane_3, pwn] : detected_shape)
	{
		if (pwn.size() < 10) //TODO : find_optimal_alpha() reports an error, why?
			continue;
		IC::Plane_3 plane = to_inexact(plane_3);
		std::list<IC::Point_2> projected_points;
		for (const auto &[point_3, normal] : pwn)
		{
			IC::Point_3 projected = plane.projection(to_inexact(point_3));
			projected_points.push_back(plane.to_2d(projected));
		}

		//Alpha_shape_2 as = get_alpha_shape(projected_points);
		Alpha_shape_2 as(projected_points.begin(), projected_points.end(), IC::FT(10000), Alpha_shape_2::REGULARIZED);
		//std::cout << *as.find_optimal_alpha(1) << std::endl;

		/******** Todo:use average sense as alpha value**********/
		//as.set_alpha(*as.find_optimal_alpha(1));
		as.set_alpha(*as.find_optimal_alpha(1) * scale); //scale optimal alpha value
		//as.set_alpha(alpha_value);

		CDT cdt;
		std::map<IC::Point_2, CDT::Vertex_handle> p_index;
		CDTriangulation(cdt, p_index, as);

		auto fit = cdt.finite_faces_begin();
		while (fit != cdt.finite_faces_end()) {

			IC::Point_2 p1 = fit->vertex(0)->point();
			IC::Point_2 p2 = fit->vertex(1)->point();
			IC::Point_2 p3 = fit->vertex(2)->point();
			
			if (CGAL::collinear(p1, p2, p3))
				continue;
			
			IC::Point_2 center = IC::Point_2(0, 0) + (((p1 - IC::Point_2(0, 0)) + (p2 - IC::Point_2(0, 0)) + (p3 - IC::Point_2(0, 0))) * 1.0 / 3.0);
			int res = as.classify(center);
			
			if (res == Alpha_shape_2::INTERIOR) {
				triangles.push_back(IC::Triangle_3(plane.to_3d(p1), plane.to_3d(p2), plane.to_3d(p3)));
			}

			fit++;
		}

	}
	return triangles;
}