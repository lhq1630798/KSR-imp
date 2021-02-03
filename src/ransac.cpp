#include "ransac.h"
#include "cgal_object.h"

using namespace Ransac;

std::vector<Detected_shape> ransac(const std::vector<PWN> &points_E)
{
	EK_to_IK to_inexact;
	IK_to_EK to_exact;
	//********read point with normal********
	// Points with normals.
	Pwn_vector points{};

	for(auto [p, n] : points_E){
		points.emplace_back(to_inexact(p), to_inexact(n));
	}
	//std::cout << points_E.front().first << " " << points_E.front().second << std::endl;
	//std::cout << points.front().first << " " << points.front().second << std::endl;
	//************ processing ************
	Efficient_ransac ransac;            // Instantiate shape detection engine.
	ransac.set_input(points);           // Provide input data.
	ransac.add_shape_factory<Plane_Shape>();  // Register detection of planes.

	// Measure time before setting up the shape detection.
	CGAL::Timer time;
	time.start();
	ransac.preprocess();                // Build internal data structures.
	time.stop();                        // Measure time after preprocessing.
	std::cout << "preprocessing took: " << time.time() * 1000 << "ms" << std::endl;

	//// Set parameters for shape detection.
	//Efficient_ransac::Parameters parameters;
	//// Set probability to miss the largest primitive at each iteration.
	//parameters.probability = 0.05;
	//// Detect shapes with at least 200 points.
	//parameters.min_points = 200;
	//// Set maximum Euclidean distance between a point and a shape.
	//parameters.epsilon = 0.002;
	//// Set maximum Euclidean distance between points to be clustered.
	//parameters.cluster_epsilon = 0.01;
	//// Set maximum normal deviation.
	//// 0.9 < dot(surface_normal, point_normal);
	//parameters.normal_threshold = 0.9;



	time.reset();
	time.start();
	ransac.detect();                // Detect shapes.
	time.stop();
	auto shapes = ransac.shapes();

	// Compute coverage, i.e. ratio of the points assigned to a shape.
	FT coverage = FT(points.size() - ransac.number_of_unassigned_points()) / FT(points.size());
	std::cout << "time: " << time.time() * 1000 << "ms" << std::endl;
	std::cout << shapes.end() - shapes.begin() << " primitives, " << coverage << " coverage" << std::endl;

	//print the detected plane with normal
	Efficient_ransac::Shape_range::iterator it = shapes.begin();
	while (it != shapes.end()) {
		// Get specific parameters depending on the detected shape.
		if (Plane_Shape* plane = dynamic_cast<Plane_Shape*>(it->get())) {
			auto normal = plane->plane_normal();
			std::cout << "Plane with normal " << normal << std::endl;
		}
		// Proceed with the next detected shape.
		it++;
	}

	// // Regularize detected planes.
	// Efficient_ransac::Plane_range planes = ransac.planes();
	// CGAL::regularize_planes(points,
	// 	Point_map(),
	// 	planes,
	// 	CGAL::Shape_detection::Plane_map<Traits>(),
	// 	CGAL::Shape_detection::Point_to_shape_index_map<Traits>(points, planes),
	// 	true,  // regularize parallelism
	// 	true,  // regularize orthogonality
	// 	false, // do not regularize coplanarity
	// 	true,  // regularize Z-symmetry (default)
	// 	10);   // 10 degrees of tolerance for parallelism / orthogonality

	// //print regularized plane with normal
	// std::cout << planes.end() - planes.begin() << " primitives" << std::endl;
	// it = shapes.begin();
	// while (it != shapes.end()) {
	// 	// Get specific parameters depending on the detected shape.
	// 	if (Plane* plane = dynamic_cast<Plane*>(it->get())) {
	// 		Vector normal = plane->plane_normal();
	// 		std::cout << "Plane with normal " << normal << std::endl;
	// 	}
	// 	// Proceed with the next detected shape.
	// 	it++;
	// }
	//********get planes and points on each plane********

	it = shapes.begin();
	std::vector<Detected_shape> detected_shape;

	int i = 0;
	while (it != shapes.end()) {
		boost::shared_ptr<Efficient_ransac::Shape> shape = *it;
		std::cout << (*it)->info();

		// Iterate through point indices assigned to each detected shape.
		std::vector<std::size_t>::const_iterator
			index_it = (*it)->indices_of_assigned_points().begin();

		Plane_Shape* plane = dynamic_cast<Plane_Shape*>(it->get());
		auto normal = plane->plane_normal();
		auto plane_3 = to_exact(plane->operator CGAL::Plane_3<CGAL::Epick>());

		//out: get convex point index
		std::vector<PWN> v;

		while (index_it != (*it)->indices_of_assigned_points().end()) {
			// Retrieve point.
			const Point_with_normal& p = *(points.begin() + (*index_it));
			v.push_back(std::make_pair(to_exact(p.first), to_exact(p.second)));

			// Proceed with the next point.
			index_it++;
		}

		detected_shape.push_back(std::make_pair(plane_3, v));

		std::cout << std::endl;

		// Proceed with the next detected shape.
		i++;
		it++;
	}
	return detected_shape;
}