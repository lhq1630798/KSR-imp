#pragma once
#include "cgal_object.h"

#include <CGAL/Timer.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/property_map.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>
#include <CGAL/Cartesian_converter.h>

namespace Ransac
{
    // Type declarations.
    typedef CGAL::Exact_predicates_inexact_constructions_kernel EPIC_K;
    typedef std::pair<EPIC_K::Point_3, EPIC_K::Vector_3> Point_with_normal;
    typedef std::vector<Point_with_normal> Pwn_vector;
    typedef CGAL::First_of_pair_property_map<Point_with_normal> Point_map;
    typedef CGAL::Second_of_pair_property_map<Point_with_normal> Normal_map;
    typedef CGAL::Shape_detection::Efficient_RANSAC_traits<EPIC_K, Pwn_vector, Point_map, Normal_map> Traits;
    typedef CGAL::Shape_detection::Efficient_RANSAC<Traits> Efficient_ransac;
    typedef CGAL::Shape_detection::Plane<Traits> Plane_Shape;
    // converter
    typedef CGAL::Cartesian_converter<EPIC_K, K> IK_to_EK;
    typedef CGAL::Cartesian_converter<K, EPIC_K> EK_to_IK;
    using Detected_shape = std::pair<Plane_3, std::vector<PWN>>;
} // namespace Ransac

std::vector<Ransac::Detected_shape> ransac(const std::vector<PWN> &points_E);