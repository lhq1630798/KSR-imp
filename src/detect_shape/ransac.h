#pragma once
#include "detect_shape/detect_shape.h"

std::vector<Detected_shape> ransac(EPIC::Pwn_vector points, const DetectShape_Params& params);