#include "region_growing.h"

std::vector<Detected_shape> region_growing(std::string path) {
	std::cout << std::endl <<
		"region_growing_on_point_set_3 example started"
		<< std::endl << std::endl;

	// Load data
	Pwn_vector points;
	std::ifstream stream(path);
	if (!stream ||
		// !CGAL::read_ply_points(
		!CGAL::read_off_points(
			stream,
			std::back_inserter(points),
			CGAL::parameters::point_map(Point_map()).
			normal_map(Normal_map()))) {
		std::cerr << "Error: cannot read file " << path << std::endl;
		return {};
	}

	std::cout << "* loaded " << points.size() << " points with normals" << std::endl;

	// Default parameter values for the data file point_set_3.xyz.
	const std::size_t k = 12;
	const FT2          max_distance_to_plane = FT2(2);
	const FT2          max_accepted_angle = FT2(20);
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
		Pwn_vector region_points;
		std::vector<Point> points_coord;

		// Iterate through all region items.
		for (const auto index : region) {
			const Point_with_normal& point = *(points.begin() + index);
			region_points.push_back(point);
			points_coord.push_back(point.first);
		}

		// The best fit plane will be a plane fitted to all region points with
		// its normal being perpendicular to the plane.
		Plane plane;
		linear_least_squares_fitting_3(points_coord.begin(), points_coord.end(), plane, CGAL::Dimension_tag<0>());

		//std::cout << plane << std::endl;
		//std::cout << region_points.size() << std::endl;

		detected_shape.push_back(std::make_pair(plane, region_points));
	}

	// Save the result to a file in the user-provided path if any.

	std::cout << std::endl <<
		"region_growing_on_point_set_2 example finished"
		<< std::endl << std::endl;

	return detected_shape;
}