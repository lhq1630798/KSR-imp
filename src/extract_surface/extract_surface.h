#pragma once
#include "partition/kinetic.h"
#include "partition/BSP.h"
struct ExtractSurface_Params
{
	std::string filename;
	IC::Vector_3 translate;
	double scale;
	double lamda;
	int GC_term;
	std::vector<EC::Detected_shape> detected_shape;
	std::list<IC::Triangle_3> alpha_triangles;
};

struct ExtractSurface_Params;
//std::tuple<std::unique_ptr<GL::Polygon_Mesh>, std::unique_ptr<GL::Lines>, int > Extract_Surface(const KPolygons_SET& polygons_set, ExtractSurface_Params& ES_params);
std::tuple<std::unique_ptr<GL::Polygon_Mesh>, std::unique_ptr<GL::Lines>, int > Extract_Surface(BSP::LCC_3& cm, ExtractSurface_Params& ES_params);
