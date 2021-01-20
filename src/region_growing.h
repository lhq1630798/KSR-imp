#pragma once
// STL includes.
#include <string>
#include <vector>
#include <utility>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
// CGAL includes.
#include <CGAL/array.h>
#include <CGAL/property_map.h>
// #include <CGAL/IO/read_xyz_points.h>
#include <CGAL/IO/read_off_points.h>
#include <CGAL/IO/write_ply_points.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing_on_point_set.h>
#include <CGAL/linear_least_squares_fitting_3.h>
// Type declarations.
using Kernel = CGAL::Simple_cartesian<double>;
using FT2 = typename Kernel::FT;
using Point = typename Kernel::Point_3;
using Vector = typename Kernel::Vector_3;
using Plane = typename Kernel::Plane_3;
using Point_with_normal = std::pair<Point, Vector>;
using Pwn_vector = std::vector<Point_with_normal>;
using Point_map = CGAL::First_of_pair_property_map<Point_with_normal>;
using Normal_map = CGAL::Second_of_pair_property_map<Point_with_normal>;

using Neighbor_query = CGAL::Shape_detection::Point_set::K_neighbor_query<Kernel, Pwn_vector, Point_map>;
using Region_type = CGAL::Shape_detection::Point_set::Least_squares_plane_fit_region<Kernel, Pwn_vector, Point_map, Normal_map>;
using Region_growing = CGAL::Shape_detection::Region_growing<Pwn_vector, Neighbor_query, Region_type>;

using Region = std::vector<std::size_t>;
using Regions = std::vector<Region>;
using Color = std::array<unsigned char, 3>;
using Point_with_color = std::pair<Point, Color>;

using Detected_shape = std::pair<Plane, std::vector<Point>>;
std::vector<Detected_shape> region_growing(std::string path);