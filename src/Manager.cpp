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
	std::vector<EPIC::EPIC_K::Point_3> points_coord;
	for (auto&[p, n] : points) {
		points_coord.push_back(p);
	}

	auto box = CGAL::bbox_3(points_coord.begin(), points_coord.end());
	double length = box.xmax() - box.xmin();
	double weight = box.ymax() - box.ymin();
	double height = box.zmax() - box.zmin();
	EPIC::EPIC_K::Point_3 center = EPIC::EPIC_K::Point_3{ (box.xmax() + box.xmin()) / 2, (box.ymax() + box.ymin()) / 2, (box.zmax() + box.zmin()) / 2 };
	translate = CGAL::ORIGIN - center;
	scale = std::max({ length, weight, height })/2;

	for (int i = 0; i < points.size(); i++) {
		points[i].first = CGAL::ORIGIN + ((points[i].first - CGAL::ORIGIN) + translate) / scale;
	}

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
	Vec3 trans = Vec3{ translate.x(), translate.y(),translate.z() };
	auto surface_lines = timer("extract surface", ::extract_surface, *kpolys_set, filename, lamda, trans, scale);
	lines = std::move(surface_lines.second);
	mesh = std::move(surface_lines.first);
		
	//*mesh = ::extract_surface(kpolys_set);
}
