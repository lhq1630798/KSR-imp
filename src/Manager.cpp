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
			init_mesh();
			ES_params.filename = fs::path{ path }.stem().string();
		}
		else
			fmt::print(stderr, "Error: cannot read file {}\n", path);

		free(path);
	}
}

//no binary
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
	std::vector<EPIC::in_Point> points_coord;
	for (auto&[p, n] : points) {
		points_coord.push_back(p);
	}

	auto box = CGAL::bbox_3(points_coord.begin(), points_coord.end());
	double length = box.xmax() - box.xmin();
	double weight = box.ymax() - box.ymin();
	double height = box.zmax() - box.zmin();
	EPIC::in_Point center = EPIC::in_Point{ (box.xmax() + box.xmin()) / 2, (box.ymax() + box.ymin()) / 2, (box.zmax() + box.zmin()) / 2 };
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
	point_cloud = std::make_unique<Point_cloud_GL>(std::move(point_GL));
}

void Manager::init_mesh() {
	
	// centralize and scale mesh
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
	ES_params.translate = CGAL::ORIGIN - center;
	ES_params.scale = std::max({ length, weight, height }) / 2;

	for (EPIC::vertex_descriptor vd : vertices(input_mesh)) {
		input_mesh.point(vd) = EPIC::in_Point{
			(input_mesh.point(vd).x() + ES_params.translate.x())/ ES_params.scale,
			(input_mesh.point(vd).y() + ES_params.translate.y())/ ES_params.scale,
			(input_mesh.point(vd).z() + ES_params.translate.z())/ ES_params.scale };
	}

	//Todo: compute alpha value = sqrt(longest edge)
	{

	}

	// visualization
	std::vector<Vec3> verts;
	std::vector<Mesh::Index> idxs;
	int count = 0;
	for (EPIC::face_descriptor fd : faces(input_mesh))
	{
		for (EPIC::vertex_descriptor vd : vertices_around_face(input_mesh.halfedge(fd), input_mesh)) {
			Vec3 p = Vec3{
				input_mesh.point(vd).x(),
				input_mesh.point(vd).y(),
				input_mesh.point(vd).z() };
			verts.push_back(p);
			idxs.push_back(count);
			count++;
		}
	}

	auto m = Mesh{verts,idxs};
	inited_mesh = std::make_unique<Mesh>(m);
}

void Manager::detect_shape(DetectShape_Params params, int DetectShape_option)
{
	//detected_shape = timer("generate_rand_polys_3", generate_rand_polys_3, 3);
	//detected_shape = timer("generate_polys_3", generate_polys_3);
	if (!input_mesh.is_empty()) {
		if (DetectShape_option == 1) {
			//Todo: ES_params.detected_shape = ransac(input_mesh, params);
			/********** Todo:shape merge **********/
		}
		else if (DetectShape_option == 2) {
			ES_params.detected_shape = region_growing_on_mesh(input_mesh, params);
			/********** Todo:shape merge **********/

		}
		convex_shape = detect_convexShape(ES_params.detected_shape);
		ES_params.alpha_triangles = Triangles_of_alphaShape(ES_params.detected_shape, alpha_value);
	}
	else if (!points.empty()) {
		if (DetectShape_option == 1) {
			ES_params.detected_shape = ransac(points, params);
			/********** Todo:shape merge **********/
		}
		else if (DetectShape_option == 2) {
			ES_params.detected_shape = region_growing_on_points(points, params);
			/********** Todo:shape merge **********/



		}
		convex_shape = detect_convexShape(ES_params.detected_shape);
		ES_params.alpha_triangles = Triangles_of_alphaShape(ES_params.detected_shape, alpha_value);
	}
	else {
		return;
	}

	//visualize convex_shape
	mesh = std::make_unique<Polygon_Mesh>(convex_shape);

	//visualize alpha_shape
	std::vector<Vec3> verts;
	std::vector<Mesh::Index> idxs;
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
	auto m = Mesh{ verts,idxs };
	alpha_mesh = std::make_unique<Mesh>(m);

}

void Manager::init_Kqueue(size_t K)// 0 means exhausted
{
	if (convex_shape.empty()) return;
	kpolys_set = std::make_unique<KPolygons_SET>(convex_shape, K);
	mesh = kpolys_set->Get_mesh();
	k_queue = std::make_unique<Kinetic_queue>(*kpolys_set);
}

void Manager::init_BSP(float expand_scale)
{
	if (convex_shape.empty()) return;
	bsp = std::make_unique<BSP::BSP_Partition>(convex_shape, expand_scale);
	mesh = bsp->Get_mesh();
}

void Manager::partition()
{
	if (!k_queue) return;
	timer("kinetic partition", &Kinetic_queue::Kpartition, *k_queue);
	mesh = kpolys_set->Get_mesh();
}

//void Manager::extract_surface(double lamda, int GC_term)
//{
//	if (!k_queue || !k_queue->is_done()) return;
//	assert(k_queue->is_done());
//	//timer("set in-liners", &KPolygons_SET::set_inliner_points, *kpolys_set, points);
//	
//	ES_params.lamda = lamda;
//	ES_params.GC_term = GC_term;
//
//	// save centralized result
//	auto param = ES_params;
//	param.translate = EPIC::in_Vector{0,0,0};
//	auto [surface, surface_lines, F_Number] = timer("extract surface", Extract_Surface, *kpolys_set, param/*filename, lamda, translate, scale, detected_shape, GC_term*/);
//
//	// auto [surface, surface_lines, F_Number] = timer("extract surface", Extract_Surface, *kpolys_set, ES_params/*filename, lamda, translate, scale, detected_shape, GC_term*/);
//	lines = std::move(surface_lines);
//	mesh = std::move(surface);
//	Number_of_Facets = F_Number;
//	
//}
void Manager::extract_surface(double lamda, int GC_term)
{
	if (!bsp || !bsp->is_done()) return;

	ES_params.lamda = lamda;
	ES_params.GC_term = GC_term;

	// save centralized result
	auto param = ES_params;
	param.translate = EPIC::in_Vector{ 0,0,0 };
	auto [surface, surface_lines, F_Number] = timer("extract surface", Extract_Surface, bsp->lcc, param/*filename, lamda, translate, scale, detected_shape, GC_term*/);

	lines = std::move(surface_lines);
	mesh = std::move(surface);
	Number_of_Facets = F_Number;

}
int Manager::run_offline(fs::path file)
{
	auto lamda = getarg(1, "-l", "--lambda");
	auto K = getarg(2, "-K", "-k");
	auto GC_term = getarg(3, "-g", "--gc");
	auto DetectShape_option = getarg(2, "-d", "--ds");
	auto param = DetectShape_Params{};
	param.max_accepted_angle = getarg(param.max_accepted_angle, "--angle");
	param.min_region_size = getarg(param.min_region_size, "--size");
	param.max_distance_to_plane = getarg(param.max_distance_to_plane, "--dist", "--distance");
	fmt::print("lambda {}, K {}, max_accepted_angle {}, min_region_size {}, max_distance_to_plane {}\n",
		lamda, K, param.max_accepted_angle, param.min_region_size, param.max_distance_to_plane);

	//if (!read_PWN(file)) {
	if (!read_mesh(file)) {
		fmt::print(stderr, "Error: cannot read file {}\n", file.string());
		return 1;
	}
	//fmt::print("* loaded {} points with normals\n", points.size());
	//init_point_cloud();
	init_mesh();
	ES_params.filename = fs::path{ file }.stem().string();

	detect_shape(param, DetectShape_option);
	init_Kqueue(K);
	partition();
	extract_surface(lamda,GC_term);
	return 0;
}
