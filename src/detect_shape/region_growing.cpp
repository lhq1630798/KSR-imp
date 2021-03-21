#include "region_growing.h"
#include "gui/app.h"
#include <CGAL/Shape_detection/Region_growing/Region_growing.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing_on_point_set.h>

#include <CGAL/Shape_detection/Region_growing/Region_growing_on_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>

#include <CGAL/linear_least_squares_fitting_3.h>
#include <CGAL/Regularization/regularize_planes.h>
#include <fmt/core.h>
#include <algorithm>

using namespace EPIC;
namespace PMP = CGAL::Polygon_mesh_processing;

std::vector<Detected_shape> region_growing_on_points(Pwn_vector points, const DetectShape_Params& params) {
	using Neighbor_query = CGAL::Shape_detection::Point_set::K_neighbor_query<EPIC_K, Pwn_vector, Point_map>;
	using Region_type = CGAL::Shape_detection::Point_set::Least_squares_plane_fit_region<EPIC_K, Pwn_vector, Point_map, Normal_map>;
	using Sorting = CGAL::Shape_detection::Point_set::Least_squares_plane_fit_sorting<EPIC_K, Pwn_vector, Neighbor_query, Point_map>;
	using Region_growing = CGAL::Shape_detection::Region_growing<Pwn_vector, Neighbor_query, Region_type, Sorting::Seed_map>;
	using Region = std::vector<std::size_t>;
	using Regions = std::vector<Region>;
	
	EK_to_IK to_inexact;
	IK_to_EK to_exact;

	// Default parameter values for the data file point_set_3.xyz.
	const std::size_t k = params.neigbor_K;
	const EPIC_K::FT          max_distance_to_plane = params.max_distance_to_plane;
	const EPIC_K::FT          max_accepted_angle = params.max_accepted_angle;
	const std::size_t min_region_size = params.min_region_size;

	// Create instances of the classes Neighbor_query and Region_type.
	Neighbor_query neighbor_query(
		points,
		k,
		Point_map());

	Region_type region_type(
		points,
		max_distance_to_plane, max_accepted_angle, min_region_size,
		Point_map(), Normal_map());

	//sort indices
	Sorting sorting(points, neighbor_query, Point_map());
	sorting.sort();

	// Create an instance of the region growing class.
	Region_growing region_growing(points, neighbor_query, region_type, sorting.seed_map());

	// Run the algorithm.
	Regions regions;
	region_growing.detect(std::back_inserter(regions));

	// Print the number of found regions.
	std::cout << "* " << regions.size() <<
		" regions have been found"
		<< std::endl;

	auto num = std::accumulate(
		regions.begin(),
		regions.end(),
		0.0,
		[](auto sum, const Region &r) {return sum + r.size(); }
	);
	fmt::print("{:.3} coverage\n", num / points.size());

	// fit plane
	std::vector<EPIC_K::Plane_3> detected_plane;
	for (const auto& region : regions) {
		std::vector<in_Point> points_coord;
		// Iterate through all region items.
		for (const auto index : region) {
			const Point_with_normal& point = *(points.begin() + index);
			points_coord.push_back(point.first);
		}

		// The best fit plane will be a plane fitted to all region points with
		// its normal being perpendicular to the plane.
		EPIC_K::Plane_3 plane;
		linear_least_squares_fitting_3(points_coord.begin(), points_coord.end(), plane, CGAL::Dimension_tag<0>());
		detected_plane.push_back(plane);
	}

	// regularize_planes
	std::vector<int> point_shape_index_map(points.size(), -1);
	int idx = 0;
	for (const auto &region : regions) {
		for (auto index : region) {
			point_shape_index_map[index] = idx;
		}
		idx++;
	}
	if (params.regularize) {
		CGAL::regularize_planes(points,
			Point_map(),
			detected_plane,
			CGAL::Identity_property_map<EPIC_K::Plane_3>(),
			CGAL::make_property_map(point_shape_index_map),
			true,  // regularize parallelism
			true,  // regularize orthogonality
			true, // regularize coplanarity
			false,  // regularize Z-symmetry 
			10,  // 10 degrees of tolerance for parallelism / orthogonality
			0.01 // tolerance coplanarity
		);
		//merge co-plane points
		auto it = regions.begin();
		while (it != regions.end()) {
			auto plane = detected_plane[point_shape_index_map[(*it)[0]]];

			auto other_it = it + 1;
			while (other_it != regions.end()) {
				auto other_plane = detected_plane[point_shape_index_map[(*other_it)[0]]];
				if (other_plane == plane || other_plane.opposite() == plane) {
					it->insert(it->end(), other_it->begin(), other_it->end());
					other_it = regions.erase(other_it);
				}
				else other_it++;
			}
			it++;
		}
		fmt::print("{} planes after regularization\n", regions.size());
	}

	// convert to exact kernel type
	std::vector<Detected_shape> detected_shape;
	for (const auto& region : regions) {
		Plane_3 plane = to_exact(detected_plane[point_shape_index_map[region[0]]]);
		//std::cout << "plane " << plane << std::endl;
		PWN_vector region_points;
		for (const auto index : region) {
			const Point_with_normal& point = *(points.begin() + index);
			region_points.emplace_back(
				to_exact(point.first),
				to_exact(point.second));
		}

		detected_shape.emplace_back(plane, region_points);
	}

	return detected_shape;
}

std::vector<Detected_shape> region_growing_on_mesh(Surface_Mesh polygon_mesh, const DetectShape_Params& params) {
	//campute vertices normal
	auto vnormals = polygon_mesh.add_property_map<vertex_descriptor, in_Vector>("v:normals", CGAL::NULL_VECTOR).first;
	auto fnormals = polygon_mesh.add_property_map<face_descriptor, in_Vector>("f:normals", CGAL::NULL_VECTOR).first;
	PMP::compute_normals(polygon_mesh, vnormals, fnormals);
	

	EK_to_IK to_inexact;
	IK_to_EK to_exact;

	using Face_range = typename Surface_Mesh::Face_range;
	
	using Neighbor_query = CGAL::Shape_detection::Polygon_mesh::One_ring_neighbor_query<Surface_Mesh>;
	using Region_type = CGAL::Shape_detection::Polygon_mesh::Least_squares_plane_fit_region<EPIC_K, Surface_Mesh>;
	
	using Sorting = CGAL::Shape_detection::Polygon_mesh::Least_squares_plane_fit_sorting<EPIC_K, Surface_Mesh, Neighbor_query>;

	using Region = std::vector<std::size_t>;
	using Regions = std::vector<Region>;

	using Vertex_to_point_map = typename Region_type::Vertex_to_point_map;
	using Region_growing = CGAL::Shape_detection::Region_growing<Face_range, Neighbor_query, Region_type, typename Sorting::Seed_map>;

	
	const Face_range face_range = faces(polygon_mesh);
	
	// Default parameter values for the data file polygon_mesh.off.
	const EPIC_K::FT          max_distance_to_plane = params.max_distance_to_plane;
	const EPIC_K::FT          max_accepted_angle = params.max_accepted_angle;
	const std::size_t min_region_size = params.min_region_size;

	// Create instances of the classes Neighbor_query and Region_type.
	Neighbor_query neighbor_query(polygon_mesh);

	const Vertex_to_point_map vertex_to_point_map(
		get(CGAL::vertex_point, polygon_mesh));

	Region_type region_type(
		polygon_mesh,
		max_distance_to_plane, max_accepted_angle, min_region_size,
		vertex_to_point_map);

	// Sort face indices.
	Sorting sorting(
		polygon_mesh, 
		neighbor_query,
		vertex_to_point_map);
	sorting.sort();

	// Create an instance of the region growing class.
	Region_growing region_growing(
		face_range, 
		neighbor_query, 
		region_type,
		sorting.seed_map());

	// Run the algorithm.
	Regions regions;
	region_growing.detect(std::back_inserter(regions));
	// Print the number of found regions.
	std::cout << "* " << regions.size() <<
		" regions have been found"
		<< std::endl;

	// fit plane
	//using size_type = typename Surface_Mesh::size_type;
	//std::vector<Detected_shape> detected_shape;
	//for (const auto& region : regions) {
	//	// Iterate through all region items.
	//	std::vector<in_Point> points_coord;
	//	PWN_vector region_points;
	//	for (const auto index : region){
	//		for (vertex_descriptor vd : vertices_around_face(polygon_mesh.halfedge(face_descriptor(static_cast<size_type>(index))), polygon_mesh)) {
	//			points_coord.push_back(polygon_mesh.point(vd));
	//			region_points.emplace_back(
	//				to_exact(polygon_mesh.point(vd)),
	//				to_exact(vnormals[vd]));
	//		}
	//	}
	//	EPIC_K::Plane_3 plane;
	//	linear_least_squares_fitting_3(points_coord.begin(), points_coord.end(), plane, CGAL::Dimension_tag<0>());
	//	Plane_3 ek_plane = to_exact(plane);
	//	detected_shape.emplace_back(ek_plane, region_points);
	//}

	using size_type = typename Surface_Mesh::size_type;
	std::vector<EPIC_K::Plane_3> detected_plane;
	Pwn_vector points;
	for (const auto& region : regions) {
		// Iterate through all region items.
		std::vector<in_Point> points_coord;
		for (const auto index : region){
			for (vertex_descriptor vd : vertices_around_face(polygon_mesh.halfedge(face_descriptor(static_cast<size_type>(index))), polygon_mesh)) {
				points_coord.push_back(polygon_mesh.point(vd));
				points.emplace_back(polygon_mesh.point(vd), vnormals[vd]);
			}
		}

		// The best fit plane will be a plane fitted to all region points with
		// its normal being perpendicular to the plane.
		EPIC_K::Plane_3 plane;
		linear_least_squares_fitting_3(points_coord.begin(), points_coord.end(), plane, CGAL::Dimension_tag<0>());
		detected_plane.push_back(plane);
	}

	//TODO: regularize_planes
	std::vector<int> point_shape_index_map(points.size(), -1);

	std::vector<int> tmesh_shape_index_map(polygon_mesh.faces().size());

	int idx = 0;
	int p_index = 0;
	for (const auto &region : regions) {
		for (auto index : region) {
			for (vertex_descriptor vd : vertices_around_face(polygon_mesh.halfedge(face_descriptor(static_cast<size_type>(index))), polygon_mesh)) {
				point_shape_index_map[p_index] = idx;
				p_index++;
			}
			tmesh_shape_index_map[index] = idx;
			
		}
		idx++;
	}
	if (params.regularize) {
		CGAL::regularize_planes(points,
			Point_map(),
			detected_plane,
			CGAL::Identity_property_map<EPIC_K::Plane_3>(),
			CGAL::make_property_map(point_shape_index_map),
			true,  // regularize parallelism
			true,  // regularize orthogonality
			true, // regularize coplanarity
			false,  // regularize Z-symmetry 
			10,  // 10 degrees of tolerance for parallelism / orthogonality
			0.01 // tolerance coplanarity
		);
		//merge co-plane points
		auto it = regions.begin();
		while (it != regions.end()) {
			auto plane = detected_plane[tmesh_shape_index_map[(*it)[0]]];

			auto other_it = it + 1;
			while (other_it != regions.end()) {
				auto other_plane = detected_plane[tmesh_shape_index_map[(*other_it)[0]]];
				if (other_plane == plane || other_plane.opposite() == plane) {
					it->insert(it->end(), other_it->begin(), other_it->end());
					other_it = regions.erase(other_it);
				}
				else other_it++;
			}
			it++;
		}
		fmt::print("{} planes after regularization\n", regions.size());
	}

	// convert to exact kernel type
	std::vector<Detected_shape> detected_shape;
	for (const auto& region : regions) {
		Plane_3 plane = to_exact(detected_plane[tmesh_shape_index_map[region[0]]]);
		//std::cout << "plane " << plane << std::endl;
		PWN_vector region_points;
		for (const auto index : region) {
			for (vertex_descriptor vd : vertices_around_face(polygon_mesh.halfedge(face_descriptor(static_cast<size_type>(index))), polygon_mesh)) {
				region_points.emplace_back(
					to_exact(polygon_mesh.point(vd)),
					to_exact(vnormals[vd]));
			}
		}
		detected_shape.emplace_back(plane, region_points);
	}

	return detected_shape;

}