#pragma once
#include "detect_shape/detect_shape.h"

std::vector<EC::Detected_shape> ransac(IC::PWN_vector points, const DetectShape_Params& params);