#pragma once
#include <string>
#include <filesystem>
#include "cgal/cgal_object.h"
#include "partition/BSP.h"
#include "partition/kinetic.h"
#include "extract_surface/extract_surface.h"

namespace fs = std::filesystem;
struct DetectShape_Params;

class Manager
{
public:
	void load_point_cloud();
	void load_mesh();
	void detect_shape(DetectShape_Params params, int DetectShape_option);//convex shape + alpha shape
	void init_Kqueue(size_t K = 1);
	void init_BSP(float);
	void partition();
	void extract_surface(double lamda, int GC_term);
	int run_offline(fs::path file);

	IC::PWN_vector points;//input points
	IC::Surface_Mesh input_mesh;//input mesh

	EC::Polygons_3 convex_shape;

	std::unique_ptr<Kinetic::KPolygons_SET> kpolys_set;
	std::unique_ptr<Kinetic::Kinetic_queue> k_queue;

	std::unique_ptr<BSP::BSP_Partition> bsp;

	//std::unique_ptr<GL::Polygon_Mesh> inited_mesh;
	std::unique_ptr<GL::Mesh> inited_mesh;//Normalized input mesh
	std::unique_ptr<GL::Point_cloud> point_cloud;//Normalized input points

	std::unique_ptr<GL::Mesh> alpha_mesh;

	std::unique_ptr<GL::Polygon_Mesh> mesh;//extracted surface mesh
	std::unique_ptr<GL::Lines> lines;//extracted surface mesh boundary

	double alpha_value=0.5;
	int Number_of_Facets;

	ExtractSurface_Params ES_params;

private:
	bool read_PWN(fs::path path);
	bool read_mesh(fs::path path);
	void reset();
	void init_point_cloud();
	void init_mesh();
	static inline const char* point_file_types = "ply,off";
};