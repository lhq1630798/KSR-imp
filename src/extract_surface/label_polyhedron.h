#pragma once
#include "min-cut/graph.h"
#include "build_map.h"
#include <iostream>

typedef Graph<double, double, double> GraphType;
GraphType* label_polyhedron(CMap_3& cm, std::vector<Dart_handle> C, Neighbor N, const KPolygons_SET& polygons_set, double lamda);

