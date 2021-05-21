#pragma once
#include "cgal/cgal_object.h"

struct DetectShape_Params
{
	bool regularize = true;
	float max_distance_to_plane = 0.01;
	float max_accepted_angle = 50;
	int min_region_size = 50;
	int neigbor_K = 12;
};

struct DetectShape_Params;
EC::Polygons_3 detect_convexShape(std::vector<EC::Detected_shape>&);//convex shape
std::list<IC::Triangle_3> Triangles_of_alphaShape(const std::vector<EC::Detected_shape>& detected_shape, float scale);//coarse surface