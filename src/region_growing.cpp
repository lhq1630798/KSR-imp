#include "region_growing.h"

#include <CGAL/Shape_detection/Region_growing/Region_growing.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing_on_point_set.h>
#include <CGAL/linear_least_squares_fitting_3.h>

using namespace EPIC;
using Neighbor_query = CGAL::Shape_detection::Point_set::K_neighbor_query<EPIC_K, Pwn_vector, Point_map>;
using Region_type = CGAL::Shape_detection::Point_set::Least_squares_plane_fit_region<EPIC_K, Pwn_vector, Point_map, Normal_map>;
using Region_growing = CGAL::Shape_detection::Region_growing<Pwn_vector, Neighbor_query, Region_type>;
using Region = std::vector<std::size_t>;
using Regions = std::vector<Region>;

std::vector<Detected_shape> region_growing(EPIC::Pwn_vector points) {
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

	// Iterate through all regions.
	std::vector<Detected_shape> detected_shape;
	for (const auto& region : regions) {
		PWN_vector region_points;
		std::vector<EPIC_K::Point_3> points_coord;

		// Iterate through all region items.
		for (const auto index : region) {
			const Point_with_normal& point = *(points.begin() + index);
			region_points.emplace_back(
					to_exact(point.first),
					to_exact(point.second));
			points_coord.push_back(point.first);
		}

		// The best fit plane will be a plane fitted to all region points with
		// its normal being perpendicular to the plane.
		EPIC_K::Plane_3 plane;
		linear_least_squares_fitting_3(points_coord.begin(), points_coord.end(), plane, CGAL::Dimension_tag<0>());

		//std::cout << plane << std::endl;
		//std::cout << region_points.size() << std::endl;

		detected_shape.emplace_back(to_exact(plane), region_points);
	}

	// Save the result to a file in the user-provided path if any.

	std::cout << std::endl <<
		"region_growing_on_point_set_2 example finished"
		<< std::endl << std::endl;

	return detected_shape;
}