#include "scene.h"
#include <nfd/nfd.h>
#include <CGAL/IO/read_off_points.h>
#include <CGAL/IO/read_ply_points.h>
#include <fmt/core.h>
#include "ransac.h"

void Scene::load_point_cloud()
{
	char* path = nullptr;
	NFD_OpenDialog(point_file_types, nullptr, &path);
	if (path) {
		fmt::print("loading {}\n", path);
		if (read_PWN(std::string(path))) {
			fmt::print("* loaded {} points with normals\n", points.size());
			init_point_cloud();
		}
		else
			fmt::print(stderr, "Error: cannot read file {}\n", path);

		free(path);
	}
}

bool Scene::read_PWN(fs::path path)
{
	points.clear();
	point_cloud.reset();
	std::ifstream stream(path);
	auto paremeters =
		CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PWN>{}).
		normal_map(CGAL::Second_of_pair_property_map<PWN>{});
		//CGAL::parameters::point_map(Ransac::Point_map()).
		//normal_map(Ransac::Normal_map());
	if (stream) {
		if (path.extension() == ".ply")
			return CGAL::read_ply_points(
				stream,
				std::back_inserter(points),
				paremeters);

		else if (path.extension() == ".off")
			return CGAL::read_off_points(
				stream,
				std::back_inserter(points),
				paremeters);
	}
	return false;
}

void Scene::init_point_cloud() {
	std::cout << points.front().first << " " << points.front().second << std::endl;

	// TODO: centralize and scale points(not just point cloud)

	// visualization
	std::vector<Vec3> point_GL;
	for (auto& [p, n] : points)
		point_GL.emplace_back((float)CGAL::to_double(p.x()),
			(float)CGAL::to_double(p.y()),
			(float)CGAL::to_double(p.z()));
	point_cloud = std::make_unique<Point_cloud_GL>(std::move(point_GL));
}

void Scene::detect_shape()
{
	if (points.empty()) return;
	detected_shape = ::detect_shape(points);
	mesh = std::make_unique<Polygon_Mesh>(detected_shape);
}
