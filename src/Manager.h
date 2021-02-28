#pragma once
#include <string>
#include <filesystem>
#include "cgal_object.h"
#include "kinetic.h"

namespace fs = std::filesystem;
class Manager
{
public:
	void load_point_cloud();
	bool read_PWN(fs::path path);
	void detect_shape(bool regularize = true);
	void init_Kqueue();
	void partition();
	void extract_surface();
	EPIC::Pwn_vector points;
	Polygons_3 detected_shape;
	std::unique_ptr<KPolygons_SET> kpolys_set;
	std::unique_ptr<Kinetic_queue> k_queue;
	std::unique_ptr<Polygon_Mesh> mesh;
	std::unique_ptr<Point_cloud_GL> point_cloud;
private:
	void reset();
	void init_points();
	static inline const char* point_file_types = "ply,off";
};