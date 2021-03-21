#pragma once
#include "detect_shape/detect_shape.h"

std::vector<Detected_shape> region_growing_on_points(EPIC::Pwn_vector points, const DetectShape_Params& params);

std::vector<Detected_shape> region_growing_on_mesh(EPIC::Surface_Mesh polygon_mesh, const DetectShape_Params& params);