#pragma once
#include "cgal/cgal_object.h"

struct DetectShape_Params
{
	bool regularize = true;
	float max_distance_to_plane = 0.01;
	float max_accepted_angle = 20;
	int min_region_size = 50;
	int neigbor_K = 12;
};

struct DetectShape_Params;
Polygons_3 detect_shape(const EPIC::Pwn_vector& pwns, const DetectShape_Params&);