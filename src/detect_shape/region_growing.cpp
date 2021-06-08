#include "region_growing.h"
#include "gui/app.h"
#include <CGAL/Shape_detection/Region_growing/Region_growing.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing_on_point_set.h>

#include <CGAL/Shape_detection/Region_growing/Region_growing_on_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/IO/write_ply_points.h>
#include <CGAL/linear_least_squares_fitting_3.h>
#include <CGAL/Regularization/regularize_planes.h>
#include <fmt/core.h>
#include <algorithm>

#include "util/config.h"
#include "shape_diameter.h"

using namespace IC;
namespace PMP = CGAL::Polygon_mesh_processing;

// TODO : adaptive regularization?
// TODO : min_region_size : use area instead of points num
namespace Region_Growing {

	Regions region_growing_on_points(const IC::PWN_vector &points) {
		using Neighbor_query = CGAL::Shape_detection::Point_set::K_neighbor_query<IC::K, IC::PWN_vector, Point_map>;
		using Region_type = CGAL::Shape_detection::Point_set::Least_squares_plane_fit_region<IC::K, IC::PWN_vector, Point_map, Normal_map>;
		using Sorting = CGAL::Shape_detection::Point_set::Least_squares_plane_fit_sorting<IC::K, IC::PWN_vector, Neighbor_query, Point_map>;
		using Region_growing = CGAL::Shape_detection::Region_growing<IC::PWN_vector, Neighbor_query, Region_type, Sorting::Seed_map>;

		const auto& params = Config::Detection::get();
		// Default parameter values for the data file point_set_3.xyz.
		const std::size_t k = params.neigbor_K;
		const IC::FT  max_distance_to_plane = params.max_distance_to_plane;
		const IC::FT  max_accepted_angle = params.max_accepted_angle;
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

		if (params.save_result) {
			//save region growing result
		}

		return regions;
	}

	void save_region_growing_mesh(IC::Surface_Mesh mesh, Regions regions) {
		//save region growing result
		using Color = CGAL::Color;
		auto face_color = mesh.add_property_map<Surface_Mesh::Face_index, Color>("f:color", Color(0, 0, 0)).first;
		// Iterate through all regions.
		for (const auto& region : regions) {
			// Generate a random color.
			const Color color(
				static_cast<unsigned char>(rand() % 256),
				static_cast<unsigned char>(rand() % 256),
				static_cast<unsigned char>(rand() % 256));
			// Iterate through all region items.
			using size_type = typename Surface_Mesh::size_type;
			for (const auto index : region)
				face_color[Surface_Mesh::Face_index(static_cast<size_type>(index))] = color;
		}
		std::string path = Config::read<std::string>("save_path") + "region_growing.off";
		std::ofstream file(path, std::ios::binary);
		file << mesh;
		file.close();
	}

	Regions region_growing_on_mesh(const IC::Surface_Mesh& mesh) {
		using Neighbor_query = CGAL::Shape_detection::Polygon_mesh::One_ring_neighbor_query<Surface_Mesh>;
		using Region_type = CGAL::Shape_detection::Polygon_mesh::Least_squares_plane_fit_region<IC::K, Surface_Mesh>;
		using Sorting = CGAL::Shape_detection::Polygon_mesh::Least_squares_plane_fit_sorting<IC::K, Surface_Mesh, Neighbor_query>;
		using Region_growing = CGAL::Shape_detection::Region_growing<Surface_Mesh::Face_range, Neighbor_query, Region_type, Sorting::Seed_map>;

		const auto& params = Config::Detection::get();
		// Default parameter values for the data file polygon_mesh.off.
		const IC::FT  max_distance_to_plane = params.max_distance_to_plane;
		const IC::FT  max_accepted_angle = params.max_accepted_angle;
		const std::size_t min_region_size = params.min_region_size;

		// Create instances of the classes Neighbor_query and Region_type.
		Neighbor_query neighbor_query(mesh);

		using Vertex_to_point_map = typename Region_type::Vertex_to_point_map;
		const Vertex_to_point_map vertex_to_point_map(
			get(CGAL::vertex_point, mesh));

		Region_type region_type(
			mesh,
			max_distance_to_plane, max_accepted_angle, min_region_size,
			vertex_to_point_map);

		// Sort face indices.
		Sorting sorting(mesh, neighbor_query, vertex_to_point_map);
		sorting.sort();

		// Create an instance of the region growing class.
		Region_growing region_growing(faces(mesh), neighbor_query, region_type, sorting.seed_map());

		// Run the algorithm.
		Regions regions;
		region_growing.detect(std::back_inserter(regions));



		return regions;
	}

	void regularize_planes(std::vector<IC::Plane_3>& out_planes, Regions regions, const IC::PWN_vector& points) {
		// point to plane map
		std::vector<int> point_shape_index_map(points.size(), -1);
		int idx = 0;
		for (const auto& region : regions) {
			for (auto index : region) {
				point_shape_index_map[index] = idx;
			}
			idx++;
		}
		auto index_map = CGAL::make_property_map(point_shape_index_map);

		const auto& params = Config::Regularization::get();
		CGAL::regularize_planes(points,
			Point_map(),
			out_planes,
			CGAL::Identity_property_map<IC::Plane_3>(),
			index_map,
			params.parallelism,  // regularize parallelism
			params.orthogonality,  // regularize orthogonality
			params.coplanarity, // regularize coplanarity
			params.Z_symmetry,  // regularize Z-symmetry 
			params.paral_degree,  // 1 degrees of tolerance for parallelism / orthogonality
			params.coplane_dist // tolerance coplanarity
		);
	}


	std::vector<IC::Plane_3> fit_planes(Regions regions, const IC::PWN_vector& points) {

		// Print the number of found regions.
		std::cout << "* " << regions.size() <<
			" regions have been found"
			<< std::endl;
		auto num = std::accumulate(
			regions.begin(),
			regions.end(),
			0.0,
			[](auto sum, const Region& r) {return sum + r.size(); }
		);
		fmt::print("{} inliners, {:.3} coverage\n", num, num / points.size());

		// fit plane
		std::vector<IC::Plane_3> planes;
		for (const auto& region : regions) {
			std::vector<IC::Point_3> points_coord;
			IC::Vector_3 average_normal = CGAL::NULL_VECTOR;
			// Iterate through all region items.
			for (const auto index : region) {
				const IC::PWN& point = *(points.begin() + index);
				points_coord.push_back(point.first);
				average_normal += point.second;
			}
			average_normal /= region.size();

			// The best fit plane will be a plane fitted to all region points with
			// its normal being perpendicular to the plane.
			IC::Plane_3 plane;
			//linear_least_squares_fitting_3(points_coord.begin(), points_coord.end(), plane, CGAL::Dimension_tag<0>());
			IC::Point_3 centroid;
			linear_least_squares_fitting_3(
				points_coord.begin(),
				points_coord.end(),
				plane,
				centroid,
				CGAL::Dimension_tag<0>(),
				IC::K(),
				CGAL::Eigen_diagonalize_traits<IC::FT>()
			);
			if (plane.orthogonal_vector() * average_normal < 0)
				plane = plane.opposite();
			planes.push_back(plane);
		}

		if (Config::Detection::get().use_regularization) {
			regularize_planes(planes, regions, points);
		}

		return planes;
	}

	std::vector<EC::Detected_shape> create_detect_shape(std::vector<IC::Plane_3> planes, Regions regions, const IC::PWN_vector& points) {
		// convert to exact kernel type
		IK_to_EK to_exact;
		std::vector<EC::Detected_shape> detected_shape;
		for (int i = 0; i < regions.size(); i++) {
			EC::Plane_3 plane = to_exact(planes[i]);
			//std::cout << "plane " << plane << std::endl;
			EC::PWN_vector region_points;
			for (const auto index : regions[i]) {
				const IC::PWN& point = points[index];
				region_points.emplace_back(
					to_exact(point.first),
					to_exact(point.second));
			}
			detected_shape.emplace_back(plane, std::move(region_points));
		}

		return detected_shape;
	}


	std::vector<EC::Detected_shape> detectshape_on_points(IC::PWN_vector points) {

		Regions regions = region_growing_on_points(points);

		// fit plane
		std::vector<IC::Plane_3> detected_plane = fit_planes(regions, points);

		auto detected_shape = create_detect_shape(detected_plane, regions, points);

		return detected_shape;
	}



	std::vector<EC::Detected_shape> detectshape_on_mesh(Surface_Mesh mesh) {

		Regions regions;
		if (Config::Detection::get().shape_diameter)
			regions = region_growing_sdf(mesh);
		else
			regions = region_growing_on_mesh(mesh);

		// convert face to PWN
		IC::PWN_vector face_centers;
		face_centers.resize(mesh.num_faces());
		auto vnormals = mesh.add_property_map<vertex_descriptor, IC::Vector_3>("v:normals", CGAL::NULL_VECTOR).first;
		auto fnormals = mesh.add_property_map<face_descriptor, IC::Vector_3>("f:normals", CGAL::NULL_VECTOR).first;
		PMP::compute_normals(mesh, vnormals, fnormals);
		for (const auto& region : regions) {
			for (const auto index : region) {
				std::vector<IC::Point_3> points;
				auto fd = face_descriptor(static_cast<Surface_Mesh::size_type>(index));
				for (auto vd : mesh.vertices_around_face(mesh.halfedge(fd))) {
					points.push_back(mesh.point(vd));
				}
				assert(points.size() == 3);
				auto center = CGAL::centroid(points[0], points[1], points[2]);
				face_centers[index] = PWN{ center, fnormals[fd] };
			}
		}

		// fit plane
		std::vector<IC::Plane_3> detected_plane = fit_planes(regions, face_centers);

		auto detected_shape = create_detect_shape(detected_plane, regions, face_centers);

		return detected_shape;
	}

}
