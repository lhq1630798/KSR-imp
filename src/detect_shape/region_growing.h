#pragma once
#include "detect_shape/detect_shape.h"

std::vector<Detected_shape> region_growing(EPIC::Pwn_vector points, const DetectShape_Params& params);