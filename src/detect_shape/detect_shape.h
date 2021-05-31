#pragma once
#include "cgal/cgal_object.h"


EC::Polygons_3 detect_convexShape(std::vector<EC::Detected_shape>&);//convex shape
std::list<IC::Triangle_3> Triangles_of_alphaShape(const std::vector<EC::Detected_shape>& detected_shape, float scale);//coarse surface