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
		if (read_PWN(std::string(path))) {
			fmt::print("* loaded {} points with normals\n", points.size());
			init_point_cloud();
			filename = fs::path{ path }.stem().string();
		}
		else
			fmt::print(stderr, "Error: cannot read file {}\n", path);

		free(path);
	}
}

bool Manager::read_PWN(fs::path path)
{
	reset();
	std::ifstream stream(path);
	auto paremeters =
		CGAL::parameters::point_map(EPIC::Point_map()).
		normal_map(EPIC::Normal_map());

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

void Manager::reset()
{
	points.clear();
	detected_shape.clear();
	kpolys_set.reset();
	k_queue.reset();
	mesh.reset();
	point_cloud.reset();
}

void Manager::init_point_cloud() {
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

void Manager::detect_shape(DetectShape_Params params)
{
	//detected_shape = timer("generate_rand_polys_3", generate_rand_polys_3, 3);
	//detected_shape = timer("generate_polys_3", generate_polys_3);
	if (points.empty()) return;
	detected_shape = ::detect_shape(points, params);
	mesh = std::make_unique<Polygon_Mesh>(detected_shape);
}

void Manager::init_Kqueue(size_t K)// 0 means exhausted
{
	if (detected_shape.empty()) return;
	kpolys_set = std::make_unique<KPolygons_SET>(detected_shape, K);
	mesh = kpolys_set->Get_mesh();
	k_queue = std::make_unique<Kinetic_queue>(*kpolys_set);
}

void Manager::partition()
{
	if (!k_queue) return;
	timer("kinetic partition", &Kinetic_queue::Kpartition, *k_queue);
	mesh = kpolys_set->Get_mesh();
}

void Manager::extract_surface(double lamda)
{
	if (!k_queue || !k_queue->is_done()) return;
	assert(k_queue->is_done());
	//timer("set in-liners", &KPolygons_SET::set_inliner_points, *kpolys_set, points);
	/* *mesh = */
	auto surface_lines = timer("extract surface", ::extract_surface, *kpolys_set, filename, lamda);
	lines = std::move(surface_lines.second);
	mesh = std::move(surface_lines.first);
		
	//*mesh = ::extract_surface(kpolys_set);
}
