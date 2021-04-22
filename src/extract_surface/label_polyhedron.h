#pragma once
#include "min-cut/graph.h"
#include "build_map.h"
#include <iostream>

#include "extract_surface/extract_surface.h"

typedef Graph<double, double, double> GraphType;
GraphType* label_polyhedron(CMap_3& cm, std::vector<Dart_handle> C, Neighbor N, const KPolygons_SET& polygons_set, ExtractSurface_Params& ES_params);

