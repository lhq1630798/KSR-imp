#pragma once
#include "cgal_object.h"

std::vector<Detected_shape> region_growing(EPIC::Pwn_vector points, const DetectShape_Params& params);