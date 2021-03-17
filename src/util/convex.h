#pragma once
#include "cgal/cgal_object.h"

Polygon_2 get_convex(Points_2::const_iterator begin, Points_2::const_iterator end);
Polygon_2 simplify_convex(const Polygon_2& polygon);
