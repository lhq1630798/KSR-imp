#pragma once
#include <iostream>
#include <cstdlib>
#include <map>
#include <string>
#include "kinetic.h"
#include "face_graph.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include "build_map.h"
#include "label_polyhedron.h"

//output mesh
typedef CGAL::Exact_predicates_inexact_constructions_kernel  inexact_K;
typedef inexact_K::FT                                        in_FT;
typedef inexact_K::Point_3                                   in_Point;
typedef inexact_K::Vector_3                                  in_Vector;
typedef CGAL::Surface_mesh<in_Point>                         Surface_Mesh;
typedef Surface_Mesh::Vertex_index                           vertex_descriptor;
typedef Surface_Mesh::Face_index                             face_descriptor;
typedef CGAL::SM_Vertex_index                                Vertex_index;

std::optional<std::vector<Vec3>> extract_surface(KPolygons_SET& polygons_set, std::string filename);