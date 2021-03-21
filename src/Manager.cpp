#include "Manager.h"
#include <nfd/nfd.h>
#include <CGAL/IO/read_off_points.h>
#include <CGAL/IO/read_ply_points.h>
#include <fmt/core.h>
#include "gui/platform.h"
#include <getopt/getopt.hpp>
#include "detect_shape/region_growing.h"

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
		if (path.extension() == ".ply") {
			if (CGAL::read_ply_points(
				stream,
				std::back_inserter(points),
				paremeters))
				return true;
			// try binary
			std::ifstream stream(path, std::ios::binary);
			return CGAL::read_ply_points(
				stream,
				std::back_inserter(points),
				paremeters);
		}


		else if (path.extension() == ".off")
			return CGAL::read_off_points(
				stream,
				std::back_inserter(points),
				paremeters);
	}
	return false;
}

void Manager::load_mesh()
{
	char* path = nullptr;
	NFD_OpenDialog(point_file_types, nullptr, &path);
	if (path) {
		fmt::print("loading {}\n", path);
		if (read_mesh(std::string(path))) {
			//fmt::print("* loaded {} points with normals\n", points.size());
			init_mesh();
			filename = fs::path{ path }.stem().string();
		}
		else
			fmt::print(stderr, "Error: cannot read file {}\n", path);

		free(path);
	}
}

bool Manager::read_mesh(fs::path path)
{
	reset();
	std::ifstream stream(path);
	
	if (stream) {
		if (path.extension() == ".ply") {
			if (CGAL::read_ply(stream, input_mesh)) {
				fmt::print("* loaded {} points\n", vertices(input_mesh).size());
				fmt::print("* loaded {} faces\n", faces(input_mesh).size());
				return true;
			}
		}
		else if (path.extension() == ".off") {
			if (CGAL::read_off(stream, input_mesh)) {
				fmt::print("* loaded {} points\n", vertices(input_mesh).size());
				fmt::print("* loaded {} faces\n", faces(input_mesh).size());
				return true;
			}
		}
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
	inited_mesh.reset();
}

void Manager::init_point_cloud() {
	// normalize normal vector
	for (auto&[p, n] : points) {
		n = n / CGAL::sqrt(n.squared_length());
	}

	// centralize and scale points(not just point cloud)
	std::vector<EPIC::in_Point> points_coord;
	for (auto&[p, n] : points) {
		points_coord.push_back(p);
	}

	auto box = CGAL::bbox_3(points_coord.begin(), points_coord.end());
	double length = box.xmax() - box.xmin();
	double weight = box.ymax() - box.ymin();
	double height = box.zmax() - box.zmin();
	EPIC::in_Point center = EPIC::in_Point{ (box.xmax() + box.xmin()) / 2, (box.ymax() + box.ymin()) / 2, (box.zmax() + box.zmin()) / 2 };
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

void Manager::init_mesh() {
	
	// TODO: centralize and scale points(not just point cloud)
	std::vector<EPIC::in_Point> points_coord;
	for (EPIC::vertex_descriptor vd : vertices(input_mesh)) {
		points_coord.push_back(EPIC::in_Point{
			input_mesh.point(vd).x(),
			input_mesh.point(vd).y(),
			input_mesh.point(vd).z() });
	}

	auto box = CGAL::bbox_3(points_coord.begin(), points_coord.end());
	double length = box.xmax() - box.xmin();
	double weight = box.ymax() - box.ymin();
	double height = box.zmax() - box.zmin();
	EPIC::in_Point center = EPIC::in_Point{ (box.xmax() + box.xmin()) / 2, (box.ymax() + box.ymin()) / 2, (box.zmax() + box.zmin()) / 2 };
	translate = CGAL::ORIGIN - center;
	scale = std::max({ length, weight, height }) / 2;

	for (EPIC::vertex_descriptor vd : vertices(input_mesh)) {
		input_mesh.point(vd) = EPIC::in_Point{
			(input_mesh.point(vd).x() + translate.x())/scale,
			(input_mesh.point(vd).y() + translate.y())/scale,
			(input_mesh.point(vd).z() + translate.z())/scale };
	}

	// visualization
	std::vector<Polygon_GL> polys_3;
	for (EPIC::face_descriptor fd : faces(input_mesh))
	{
		//std::cout << fd << std::endl;
		std::vector<Vec3> verts;
		for (EPIC::vertex_descriptor vd : vertices_around_face(input_mesh.halfedge(fd), input_mesh)) {
			Vec3 p = Vec3{
				input_mesh.point(vd).x(),
				input_mesh.point(vd).y(),
				input_mesh.point(vd).z() };
			verts.push_back(p);
		}
		polys_3.push_back(Polygon_GL(verts));
	}
	
	inited_mesh = std::make_unique<Polygon_Mesh>(polys_3);
}

void Manager::detect_shape(DetectShape_Params params)
{
	//detected_shape = timer("generate_rand_polys_3", generate_rand_polys_3, 3);
	//detected_shape = timer("generate_polys_3", generate_polys_3);
	if (!input_mesh.is_empty()) {
		auto detect_shape = region_growing_on_mesh(input_mesh, params);
		detected_shape = ::detect_shape(detect_shape);
	}
	else if (!points.empty()) {
		auto detect_shape = region_growing_on_points(points, params);
		detected_shape = ::detect_shape(detect_shape);
	}
	else {
		return;
	}
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
	//Vec3 trans = Vec3{ translate.x(), translate.y(),translate.z() };
	auto [surface, surface_lines] = timer("extract surface", ::extract_surface, *kpolys_set, filename, lamda, translate, scale);
	lines = std::move(surface_lines);
	mesh = std::move(surface);
	
}

int Manager::run_offline(fs::path file)
{
	auto lamda = getarg(0.5, "-l", "--lambda");
	auto K = getarg(2, "-K", "-k");
	auto param = DetectShape_Params{};
	param.max_accepted_angle = getarg(param.max_accepted_angle, "--angle");
	param.min_region_size = getarg(param.min_region_size, "--size");
	param.max_distance_to_plane = getarg(param.max_distance_to_plane, "--dist", "--distance");
	fmt::print("lambda {}, K {}, max_accepted_angle {}, min_region_size {}, max_distance_to_plane {}\n",
		lamda, K, param.max_accepted_angle, param.min_region_size, param.max_distance_to_plane);

	if (!read_PWN(file)) {
		fmt::print(stderr, "Error: cannot read file {}\n", file.string());
		return 1;
	}
	fmt::print("* loaded {} points with normals\n", points.size());
	init_point_cloud();
	filename = fs::path{ file }.stem().string();

	detect_shape(param);
	init_Kqueue(K);
	partition();
	extract_surface(lamda);
	return 0;
}
