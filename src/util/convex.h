#pragma once
#include "cgal/cgal_object.h"

EC::Polygon_2 get_convex(EC::Points_2::const_iterator begin, EC::Points_2::const_iterator end);
EC::Polygon_2 simplify_convex(const EC::Polygon_2& polygon);
