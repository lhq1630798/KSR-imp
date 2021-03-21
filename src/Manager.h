#pragma once
#include <string>
#include <filesystem>
#include "cgal/cgal_object.h"
#include "partition/kinetic.h"

namespace fs = std::filesystem;
struct DetectShape_Params;

class Manager
{
public:
	void load_point_cloud();
	void load_mesh();
	void detect_shape(DetectShape_Params params);
	void init_Kqueue(size_t K = 1);
	void partition();
	void extract_surface(double lamda);
	int run_offline(fs::path file);
	std::string filename;
	EPIC::in_Vector translate;
	double scale;
	EPIC::Pwn_vector points;
	EPIC::Surface_Mesh input_mesh;
	Polygons_3 detected_shape;
	std::unique_ptr<KPolygons_SET> kpolys_set;
	std::unique_ptr<Kinetic_queue> k_queue;
	std::unique_ptr<Polygon_Mesh> mesh;
	std::unique_ptr<Polygon_Mesh> inited_mesh;
	std::unique_ptr<Point_cloud_GL> point_cloud;
	std::unique_ptr<Lines_GL> lines;
private:
	bool read_PWN(fs::path path);
	bool read_mesh(fs::path path);
	void reset();
	void init_point_cloud();
	void init_mesh();
	static inline const char* point_file_types = "ply,off";
};