#pragma once
#include "partition/kinetic.h"
struct ExtractSurface_Params
{
	std::string filename;
	EPIC::in_Vector translate;
	double scale;
	double lamda;
	int GC_term;
	std::vector<Detected_shape> detected_shape;
	std::list<EPIC::in_Triangle> alpha_triangles;
};

struct ExtractSurface_Params;
std::tuple<std::unique_ptr<Polygon_Mesh>, std::unique_ptr<Lines_GL>, int > Extract_Surface(const KPolygons_SET& polygons_set, ExtractSurface_Params& ES_params);
