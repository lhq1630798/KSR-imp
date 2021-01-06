#include "ransac.h"

std::vector<Detected_shape> ransac(std::string path)
{
	//********read point with normal********
	// Points with normals.
	Pwn_vector points;

	// Load point set from a file.
	std::ifstream stream(path);
	if (!stream ||
		!CGAL::read_xyz_points(
			stream,
			std::back_inserter(points),
			CGAL::parameters::point_map(Point_map()).
			normal_map(Normal_map()))) {
		std::cerr << "Error: cannot read file cube.pwn!" << std::endl;
		return {};
	}

	//************ processing ************
	Efficient_ransac ransac;            // Instantiate shape detection engine.
	ransac.set_input(points);           // Provide input data.
	ransac.add_shape_factory<Plane>();  // Register detection of planes.

	// Measure time before setting up the shape detection.
	CGAL::Timer time;
	time.start();
	ransac.preprocess();                // Build internal data structures.
	time.stop();                        // Measure time after preprocessing.
	std::cout << "preprocessing took: " << time.time() * 1000 << "ms" << std::endl;

	// Perform detection once (you can also perform several time and choose the best coverage)
	Efficient_ransac::Shape_range shapes = ransac.shapes();

	time.reset();
	time.start();
	ransac.detect();                // Detect shapes.
	time.stop();
	shapes = ransac.shapes();

	// Compute coverage, i.e. ratio of the points assigned to a shape.
	FT coverage = FT(points.size() - ransac.number_of_unassigned_points()) / FT(points.size());
	std::cout << "time: " << time.time() * 1000 << "ms" << std::endl;
	std::cout << shapes.end() - shapes.begin() << " primitives, " << coverage << " coverage" << std::endl;

	//print the detected plane with normal
	Efficient_ransac::Shape_range::iterator it = shapes.begin();
	while (it != shapes.end()) {
		// Get specific parameters depending on the detected shape.
		if (Plane* plane = dynamic_cast<Plane*>(it->get())) {
			Vector normal = plane->plane_normal();
			std::cout << "Plane with normal " << normal << std::endl;
		}
		// Proceed with the next detected shape.
		it++;
	}

	// Regularize detected planes.
	Efficient_ransac::Plane_range planes = ransac.planes();
	CGAL::regularize_planes(points,
		Point_map(),
		planes,
		CGAL::Shape_detection::Plane_map<Traits>(),
		CGAL::Shape_detection::Point_to_shape_index_map<Traits>(points, planes),
		true,  // regularize parallelism
		true,  // regularize orthogonality
		false, // do not regularize coplanarity
		true,  // regularize Z-symmetry (default)
		10);   // 10 degrees of tolerance for parallelism / orthogonality

	//print regularized plane with normal
	std::cout << planes.end() - planes.begin() << " primitives" << std::endl;
	it = shapes.begin();
	while (it != shapes.end()) {
		// Get specific parameters depending on the detected shape.
		if (Plane* plane = dynamic_cast<Plane*>(it->get())) {
			Vector normal = plane->plane_normal();
			std::cout << "Plane with normal " << normal << std::endl;
		}
		// Proceed with the next detected shape.
		it++;
	}
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

		Plane* plane = dynamic_cast<Plane*>(it->get());
		Vector normal = plane->plane_normal();
		FT d = plane->d();

		//out: get convex point index
		std::vector<Point> v;

		//计算每个平面的convex
		while (index_it != (*it)->indices_of_assigned_points().end()) {
			// Retrieve point.
			const Point_with_normal& p = *(points.begin() + (*index_it));
			v.push_back(p.first);

			// Proceed with the next point.
			index_it++;
		}

		detected_shape.push_back(std::make_pair(*plane,v));

		std::cout << std::endl;

		// Proceed with the next detected shape.
		i++;
		it++;
	}
	return detected_shape;
}
