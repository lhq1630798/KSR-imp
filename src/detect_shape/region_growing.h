#pragma once
#include "detect_shape/detect_shape.h"

std::vector<EC::Detected_shape> region_growing_on_points(IC::PWN_vector points, const DetectShape_Params& params);

std::vector<EC::Detected_shape> region_growing_on_mesh(IC::Surface_Mesh polygon_mesh, const DetectShape_Params& params);