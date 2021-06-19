#include "structure_detect.h"
#include "util/config.h"
#define CGAL_EIGEN3_ENABLED 
#include <CGAL/poisson_surface_reconstruction.h>
#include <CGAL/Poisson_reconstruction_function.h>
#include <execution>
#include <mutex>

using Poisson_reconstruction_function = CGAL::Poisson_reconstruction_function<IC::K>;
using Distinction_map = IC::Surface_Mesh::Property_map< IC::face_descriptor, double>;

std::vector<IC::Vector_3> sphere_sample(int samples = 50) {
	std::vector<IC::Vector_3> rays;
	double pi = 2 * acos(0.0);
	//std::cout << pi << std::endl;
	double gold = 3 - sqrt(5);
	//std::cout << gold << std::endl;
	for (int i = 0; i < samples; i++) {
		double z = 1 - (double(i) / double(samples - 1)) * 2;
		double theta = pi * i * gold;
		double x = cos(theta) * sqrt(1 - z * z);
		double y = sin(theta) * sqrt(1 - z * z);
		//std::cout << x <<" "<< y <<" "<< z << std::endl;
		rays.push_back(IC::Vector_3(x, y, z));
	}
	return rays;
}

Poisson_reconstruction_function get_implicit_function(const IC::Surface_Mesh& mesh) {
	std::vector<IC::PWN> point_with_normals;
	for (auto fd : mesh.faces()) {
		std::vector<IC::Point_3> points;
		auto h = mesh.halfedge(fd);
		for (auto vd : mesh.vertices_around_face(h)) {
			points.push_back(mesh.point(vd));
		}
		assert(points.size() == 3);
		auto center = CGAL::centroid(points[0], points[1], points[2]);
		auto normal = CGAL::unit_normal(points[0], points[1], points[2]);
		point_with_normals.push_back({ center ,normal });
	}

	Poisson_reconstruction_function function(point_with_normals.begin(), point_with_normals.end(), IC::Point_map(), IC::Normal_map());
	if (!function.compute_implicit_function()) {
		fmt::print("Poisson indicator function fail\n");
	}
	else {
		fmt::print("compute_implicit_function done\n");
	}
	return function;
}

std::pair<double,double> add_distinction(IC::Surface_Mesh& mesh, double radius, const Poisson_reconstruction_function& function) {
	auto samples = sphere_sample(300);
	Distinction_map distinction = mesh.add_property_map<IC::face_descriptor, double>("f:distinction", 0).first;
	double min = 1, max = 0;
	std::mutex map_mutex;
	std::for_each(std::execution::par, mesh.faces().begin(), mesh.faces().end(),
		[&](IC::face_descriptor fd) {

		std::vector<IC::Point_3> points;
		auto h = mesh.halfedge(fd);
		for (auto vd : mesh.vertices_around_face(h)) {
			points.push_back(mesh.point(vd));
		}
		assert(points.size() == 3);
		auto center = CGAL::centroid(points[0], points[1], points[2]);

		double count = 0;
		for (auto ray : samples) {
			auto point = center + radius * ray;
			if (function(point) > 0) {
				count++;
			}
		}
		double value = std::abs(count * 2.0 / samples.size() - 1);

		std::lock_guard lock_map{ map_mutex };
		if (value < min) min = value;
		if (value > max) max = value;
		distinction[fd] = value;
	});
	return { min,max };
}

void structure_detect(IC::Surface_Mesh& mesh) {
	auto function = get_implicit_function(mesh);

	using Color = CGAL::Color;
	auto color_map = mesh.add_property_map<IC::face_descriptor, Color>("f:color", Color(0, 0, 0));
	auto face_color = color_map.first;
	if (!color_map.second) {
		color_map = mesh.property_map<IC::face_descriptor, Color>("f:color");
		assert(color_map.second);
		face_color = color_map.first;
	}

	for (auto r : { 0.05, 0.07, 0.1, 0.15, 0.2 }) {
		auto [min, max] = add_distinction(mesh, r, function);
		auto distinction_map = mesh.property_map<IC::face_descriptor, double>("f:distinction").first;
		for (auto fd : mesh.faces()) {
			auto value = (distinction_map[fd]-min)/(max-min);
			value = 1 - value;
			value = 255 * value;
			Color color(value, value, value);
			face_color[fd] = color;
		}
		std::ofstream out(Config::read<std::string>("save_path") + "poisson_" + std::to_string(r) + ".off");
		out << mesh;
	}
}