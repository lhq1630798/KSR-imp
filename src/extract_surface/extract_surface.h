#pragma once
#include "kinetic.h"

std::optional<std::vector<Vec3>> extract_surface(const KPolygons_SET& polygons_set, std::string filename, double lamda);
