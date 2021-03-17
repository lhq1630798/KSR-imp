#pragma once
#include "partition/kinetic.h"

std::pair<std::unique_ptr<Polygon_Mesh>, std::unique_ptr<Lines_GL> > extract_surface(const KPolygons_SET& polygons_set, std::string filename, double lamda, Vec3 trans, double scale);
