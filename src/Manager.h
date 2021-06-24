#pragma once
#include <string>
#include <filesystem>
#include "cgal/cgal_object.h"
#include "detect_shape/hierarchical.h"
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
	void detect_shape();//convex shape + alpha shape
	void init_Kqueue();
	void init_BSP();
	void partition();
	void extract_surface();
	void save_results();
	void save_convex();
	int run_offline(fs::path file);

	IC::PWN_vector points;//input points
	IC::Surface_Mesh input_mesh;//input mesh

	EC::Polygons_3 convex_shape;

	std::unique_ptr<Hierarchical::FaceQEM> face_qem;
	std::unique_ptr<Hierarchical::PointQEM> points_qem;
	// 
	std::unique_ptr<Kinetic::KPolygons_SET> kpolys_set;
	std::unique_ptr<Kinetic::Kinetic_queue> k_queue;
	std::unique_ptr<BSP::BSP_Partition> bsp;
	
	std::unique_ptr<GL::Mesh> inited_mesh;//Normalized input mesh
	std::unique_ptr<GL::Point_cloud> point_cloud;//Normalized input points

	std::unique_ptr<GL::Mesh> alpha_mesh;
	std::unique_ptr<GL::Mesh> qem_mesh;
	std::unique_ptr<GL::Points> qem_points;

	std::unique_ptr<GL::Polygon_Mesh> mesh;//extracted surface mesh
	std::unique_ptr<GL::Lines> lines;//extracted surface mesh boundary

	int Number_of_Output_Facets;
	int Number_of_Polyhedra;
	double Partition_Time;
	double Extract_Time;

	ExtractSurface_Params ES_params;

	void process_detected_shape(std::vector<EC::Detected_shape>);
private:
	bool read_PWN(fs::path path);
	bool read_mesh(fs::path path);
	void reset();
	void init_point_cloud();
	void init_mesh();
	static inline const char* point_file_types = "ply,off";
};