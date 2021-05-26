#include "Manager.h"
#include <nfd/nfd.h>
#include <CGAL/IO/read_off_points.h>
#include <CGAL/IO/read_ply_points.h>
#include <fmt/core.h>
#include "gui/platform.h"
#include <getopt/getopt.hpp>
#include "detect_shape/region_growing.h"
#include "detect_shape/ransac.h"
#include "detect_shape/detect_shape.h"
#include "util/config.h"

void Manager::load_point_cloud()
{
	char* path = nullptr;
	NFD_OpenDialog(point_file_types, nullptr, &path);
	if (path) {
		fmt::print("loading {}\n", path);
		if (read_PWN(std::string(path))) {
			fmt::print("* loaded {} points with normals\n", points.size());
			init_point_cloud();
			ES_params.filename = fs::path{ path }.stem().string();
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
		CGAL::parameters::point_map(IC::Point_map()).
		normal_map(IC::Normal_map());

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
			init_mesh();
			ES_params.filename = fs::path{ path }.stem().string();
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
			// try binary
			std::ifstream stream(path, std::ios::binary);
			if (CGAL::read_ply(stream, input_mesh))
			{
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
	input_mesh.clear();

	//detected_shape.clear();
	convex_shape.clear();
	//alpha_triangles.clear();

	kpolys_set.reset();
	k_queue.reset();

	point_cloud.reset();
	inited_mesh.reset();

	alpha_mesh.reset();

	mesh.reset();
	lines.reset();
}

void Manager::init_point_cloud() {
	// normalize normal vector
	for (auto&[p, n] : points) {
		n = n / CGAL::sqrt(n.squared_length());
	}

	// centralize and scale points
	std::vector<IC::Point_3> points_coord;
	for (auto&[p, n] : points) {
		points_coord.push_back(p);
	}

	auto box = CGAL::bbox_3(points_coord.begin(), points_coord.end());
	double length = box.xmax() - box.xmin();
	double weight = box.ymax() - box.ymin();
	double height = box.zmax() - box.zmin();
	IC::Point_3 center = IC::Point_3{ (box.xmax() + box.xmin()) / 2, (box.ymax() + box.ymin()) / 2, (box.zmax() + box.zmin()) / 2 };
	ES_params.translate = CGAL::ORIGIN - center;
	ES_params.scale = std::max({ length, weight, height })/2;

	for (int i = 0; i < points.size(); i++) {
		points[i].first = CGAL::ORIGIN + ((points[i].first - CGAL::ORIGIN) + ES_params.translate) / ES_params.scale;
	}

	//Todo: compute alpha value = point dense
	{

	}

	// visualization
	std::vector<Vec3> point_GL;
	for (const auto&[p, n] : points)
		point_GL.emplace_back((float)CGAL::to_double(p.x()),
			(float)CGAL::to_double(p.y()),
			(float)CGAL::to_double(p.z()));
	if (!Config::read<bool>("headless"))
		point_cloud = std::make_unique<GL::Point_cloud>(std::move(point_GL));
}

void Manager::init_mesh() {

	// centralize and scale mesh
	std::vector<IC::Point_3> points_coord;
	for (IC::vertex_descriptor vd : vertices(input_mesh)) {
		points_coord.push_back(IC::Point_3{
			input_mesh.point(vd).x(),
			input_mesh.point(vd).y(),
			input_mesh.point(vd).z() });
	}

	auto box = CGAL::bbox_3(points_coord.begin(), points_coord.end());
	double length = box.xmax() - box.xmin();
	double weight = box.ymax() - box.ymin();
	double height = box.zmax() - box.zmin();
	IC::Point_3 center = IC::Point_3{ (box.xmax() + box.xmin()) / 2, (box.ymax() + box.ymin()) / 2, (box.zmax() + box.zmin()) / 2 };
	ES_params.translate = CGAL::ORIGIN - center;
	ES_params.scale = std::max({ length, weight, height }) / 2;

	for (IC::vertex_descriptor vd : vertices(input_mesh)) {
		input_mesh.point(vd) = IC::Point_3{
			(input_mesh.point(vd).x() + ES_params.translate.x())/ ES_params.scale,
			(input_mesh.point(vd).y() + ES_params.translate.y())/ ES_params.scale,
			(input_mesh.point(vd).z() + ES_params.translate.z())/ ES_params.scale };
	}

	//Todo: compute alpha value = sqrt(longest edge)
	{

	}

	//ES_params.alpha_triangles = Triangles_of_alphaShape(ES_params.detected_shape, alpha_value);
	//TODO:
	// for now we use input mesh instead of alpha shape
	//ES_params.alpha_triangles.clear();
	//for (auto fd : faces(input_mesh))
	//{
	//	std::vector<IC::Point_3> points;
	//	for (auto vd : vertices_around_face(input_mesh.halfedge(fd), input_mesh)) {
	//		auto p = IC::Point_3{
	//			input_mesh.point(vd).x(),
	//			input_mesh.point(vd).y(),
	//			input_mesh.point(vd).z() };
	//		points.push_back(p);
	//	}
	//	assert(points.size() == 3);
	//	ES_params.alpha_triangles.push_back(IC::Triangle_3{ points[0], points[1] ,points[2] });
	//}

	if (Config::read<bool>("headless"))
		return;

	// visualization
	std::vector<Vec3> verts;
	std::vector<GL::Mesh::Index> idxs;
	int count = 0;
	for (IC::face_descriptor fd : faces(input_mesh))
	{
		for (IC::vertex_descriptor vd : vertices_around_face(input_mesh.halfedge(fd), input_mesh)) {
			Vec3 p = Vec3{
				input_mesh.point(vd).x(),
				input_mesh.point(vd).y(),
				input_mesh.point(vd).z() };
			verts.push_back(p);
			idxs.push_back(count);
			count++;
		}
	}
	// add a face at bottom for visualization
	verts.push_back(Vec3{ -1,-1,-1 });
	verts.push_back(Vec3{ 1,-1,-1 });
	verts.push_back(Vec3{ 1, 1,-1 });
	idxs.push_back(count++);
	idxs.push_back(count++);
	idxs.push_back(count++);


	auto m = GL::Mesh{verts,idxs};
	inited_mesh = std::make_unique<GL::Mesh>(m);
}

void Manager::detect_shape()
{
	//detected_shape = timer("generate_rand_polys_3", generate_rand_polys_3, 3);
	//detected_shape = timer("generate_polys_3", generate_polys_3);
	auto& parms = Config::Detection::get();
	auto method = parms.method;
	if (!input_mesh.is_empty())
	{
		if (method == "ransac")
		{
			//Todo: ES_params.detected_shape = ransac(input_mesh, params);
			/********** Todo:shape merge **********/
		}
		else if (method == "region_growing")
		{
			ES_params.detected_shape = region_growing_on_mesh(input_mesh);
			/********** Todo:shape merge **********/

		}
	}
	else if (!points.empty()) {
		if (method == "ransac")
		{
			//ES_params.detected_shape = ransac(points, params);
			/********** Todo:shape merge **********/
		}
		else if (method == "region_growing")
		{
			ES_params.detected_shape = region_growing_on_points(points);
			/********** Todo:shape merge **********/



		}
	}
	else {
		return;
	}

	convex_shape = detect_convexShape(ES_params.detected_shape);
	ES_params.alpha_triangles = Triangles_of_alphaShape(ES_params.detected_shape, parms.alpha_scale);

	if (Config::read<bool>("headless"))
		return;

	//visualize convex_shape
	mesh = std::make_unique<GL::Polygon_Mesh>(convex_shape);

	//visualize alpha_shape
	std::vector<Vec3> verts;
	std::vector<GL::Mesh::Index> idxs;
	int count = 0;
	for (auto triangle : ES_params.alpha_triangles)
	{
		verts.push_back(Vec3{ triangle.vertex(0).x(), triangle.vertex(0).y(), triangle.vertex(0).z() });
		verts.push_back(Vec3{ triangle.vertex(1).x(), triangle.vertex(1).y(), triangle.vertex(1).z() });
		verts.push_back(Vec3{ triangle.vertex(2).x(), triangle.vertex(2).y(), triangle.vertex(2).z() });
		idxs.push_back(count);
		idxs.push_back(count+1);
		idxs.push_back(count+2);
		count = count+3;
	}
	auto m = GL::Mesh{ verts,idxs };
	alpha_mesh = std::make_unique<GL::Mesh>(m);
}

void Manager::init_BSP()
{
	auto& params = Config::Partition::get();

	if (convex_shape.empty())
		return;
	bsp = std::make_unique<BSP::BSP_Partition>(convex_shape, params.expand_scale);
	if (!Config::read<bool>("headless"))
		mesh = bsp->Get_mesh();
}

void Manager::partition()
{
	if (!bsp)
		return;
	timer("partition", &BSP::BSP_Partition::partition, *bsp);
	if (!Config::read<bool>("headless"))
		mesh = bsp->Get_mesh();
}

void Manager::extract_surface()
{
	if (!bsp || !bsp->is_done())
		return;
	auto [surface, surface_lines, F_Number] = timer("extract surface", Extract_Surface, bsp->lcc, ES_params /*filename, lamda, translate, scale, detected_shape, GC_term*/);

	if (!Config::read<bool>("headless"))
	{
		lines = std::move(surface_lines);
		mesh = std::move(surface);
	}
	Number_of_Facets = F_Number;
}
int Manager::run_offline(fs::path file)
{

	//if (!read_PWN(file)) {
	if (!read_mesh(file)) {
		fmt::print(stderr, "Error: cannot read file {}\n", file.string());
		return 1;
	}
	//init_point_cloud();
	init_mesh();
	ES_params.filename = fs::path{ file }.stem().string();


	detect_shape();
	init_BSP();
	partition();
	extract_surface();
	return 0;
}
