#include "Manager.h"
#include <nfd/nfd.h>
#include <CGAL/IO/read_off_points.h>
#include <CGAL/IO/read_ply_points.h>
#include <fmt/core.h>
#include "platform.h"

void Manager::load_point_cloud()
{
	char* path = nullptr;
	NFD_OpenDialog(point_file_types, nullptr, &path);
	if (path) {
		fmt::print("loading {}\n", path);
		if (read_PWN(std::string(path)))
			fmt::print("* loaded {} points with normals\n", points.size());
		else
			fmt::print(stderr, "Error: cannot read file {}\n", path);
		free(path);
	}
}

bool Manager::read_PWN(fs::path path)
{
	reset();
	std::ifstream stream(path);
	if (!stream) return false;

	auto paremeters =
		CGAL::parameters::point_map(EPIC::Point_map()).
		normal_map(EPIC::Normal_map());

	if (path.extension() == ".ply") {
		if (!CGAL::read_ply_points(stream, std::back_inserter(points), paremeters))
			return false;
	}
	else if (path.extension() == ".off") {
		if (!CGAL::read_off_points(stream, std::back_inserter(points), paremeters))
			return false;
	}
	// else if (path.extension() == ".pwn"){...}
	else
		return false;

	init_points();
	return true;
}

void Manager::reset()
{
	points.clear();
	kpolys_set.reset();
	k_queue.reset();
	mesh.reset();
	point_cloud.reset();
}

void Manager::init_points() {
	// normalize normal vector
	for (auto&[p, n] : points) {
		n = n / CGAL::sqrt(n.squared_length());
	}

	// TODO: centralize and scale points(not just point cloud)

	// visualization
	std::vector<Vec3> point_GL;
	for (const auto&[p, n] : points)
		point_GL.emplace_back((float)CGAL::to_double(p.x()),
		(float)CGAL::to_double(p.y()),
			(float)CGAL::to_double(p.z()));
	point_cloud = std::make_unique<Point_cloud_GL>(std::move(point_GL));
}

void Manager::detect_shape(bool regularize)
{
	//detected_shape = timer("generate_rand_polys_3", generate_rand_polys_3, 3);
	//detected_shape = timer("generate_polys_3", generate_polys_3);
	if (points.empty()) return;
	detected_shape = ::detect_shape(points, regularize);
	mesh = std::make_unique<Polygon_Mesh>(detected_shape);
}

void Manager::init_Kqueue()
{
	if (detected_shape.empty()) return;
	//size_t K = 0; // 0 means exhausted
	size_t K = 1;
	kpolys_set = std::make_unique<KPolygons_SET>(detected_shape, K);
	*mesh = kpolys_set->Get_mesh();
	k_queue = std::make_unique<Kinetic_queue>(*kpolys_set);
}

void Manager::partition()
{
	if (!k_queue) init_Kqueue();
	if (!k_queue) return;
	timer("kinetic partition", &Kinetic_queue::Kpartition, *k_queue);
	*mesh = kpolys_set->Get_mesh();
}

void Manager::extract_surface()
{
	if (!k_queue) return;
	assert(k_queue->is_done());
	//timer("set in-liners", &KPolygons_SET::set_inliner_points, *kpolys_set, points);
	/**mesh = */timer("extract surface", ::extract_surface, *kpolys_set);
	//*mesh = ::extract_surface(kpolys_set);
}
