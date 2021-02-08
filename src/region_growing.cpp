#include "region_growing.h"

#include <CGAL/Shape_detection/Region_growing/Region_growing.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing_on_point_set.h>
#include <CGAL/linear_least_squares_fitting_3.h>
#include <CGAL/Regularization/regularize_planes.h>
#include <fmt/core.h>
#include <algorithm>

using namespace EPIC;
using Neighbor_query = CGAL::Shape_detection::Point_set::K_neighbor_query<EPIC_K, Pwn_vector, Point_map>;
using Region_type = CGAL::Shape_detection::Point_set::Least_squares_plane_fit_region<EPIC_K, Pwn_vector, Point_map, Normal_map>;
using Region_growing = CGAL::Shape_detection::Region_growing<Pwn_vector, Neighbor_query, Region_type>;
using Region = std::vector<std::size_t>;
using Regions = std::vector<Region>;

std::vector<Detected_shape> region_growing(EPIC::Pwn_vector points, bool regularize) {
	EK_to_IK to_inexact;
	IK_to_EK to_exact;

	// Default parameter values for the data file point_set_3.xyz.
	const std::size_t k = 12;
	const EPIC_K::FT          max_distance_to_plane = 2;
	const EPIC_K::FT          max_accepted_angle = 20;
	const std::size_t min_region_size = 50;

	// Create instances of the classes Neighbor_query and Region_type.
	Neighbor_query neighbor_query(
		points,
		k,
		Point_map());

	Region_type region_type(
		points,
		max_distance_to_plane, max_accepted_angle, min_region_size,
		Point_map(), Normal_map());

	// Create an instance of the region growing class.
	Region_growing region_growing(points, neighbor_query, region_type);

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
		std::vector<EPIC_K::Point_3> points_coord;
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
	if (regularize) {
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
		std::cout << "plane " << plane << std::endl;
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