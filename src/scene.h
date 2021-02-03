#pragma once
#include <string>
#include <filesystem>
#include "cgal_object.h"
#include "kinetic.h"
#include "ransac.h"

namespace fs = std::filesystem;
class Scene
{
public:
	void load_point_cloud();
	void detect_shape();
	//Ransac::Pwn_vector points;
	std::vector<PWN> points;
	Polygons_3 detected_shape;
	std::unique_ptr<KPolygons_SET> kpolys_set;
	std::unique_ptr<Kinetic_queue> k_queue;
	std::unique_ptr<Polygon_Mesh> mesh;
	std::unique_ptr<Point_cloud_GL> point_cloud;
private:
	bool read_PWN(fs::path path);
	void init_point_cloud();
	static inline const char* point_file_types = "ply,off";
};