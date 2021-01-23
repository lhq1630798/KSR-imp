#pragma once
#include <CGAL/Combinatorial_map.h>
#include <CGAL/Cell_attribute.h>
#include <iostream>
#include <cstdlib>
#include "kinetic.h"
#include "min-cut/graph.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

//output mesh
typedef CGAL::Exact_predicates_inexact_constructions_kernel  inexact_K;
typedef inexact_K::FT                                        in_FT;
typedef inexact_K::Point_3                                   in_Point;
typedef inexact_K::Vector_3                                  in_Vector;
typedef CGAL::Surface_mesh<in_Point>                         Surface_Mesh;
typedef Surface_Mesh::Vertex_index                           vertex_descriptor;
typedef Surface_Mesh::Face_index                             face_descriptor;
typedef CGAL::SM_Vertex_index                                Vertex_index;


struct face_attributes {
	FT area;
	Plane_3 plane;
	PWN_E inline_points;
};

struct polyhedra_attributes {
	int number;
	Point_3 center;
};

struct Sum_functor
{
	template<class Cell_attribute>
	void operator()(Cell_attribute& ca1, Cell_attribute& ca2)
	{
		ca1.info() = ca1.info();
	}
};
struct Divide_by_two_functor
{
	template<class Cell_attribute>
	void operator()(Cell_attribute& ca1, Cell_attribute& ca2)
	{
		ca1.info() = ca1.info();
		ca2.info() = ca1.info();
	}
};

// My item class
struct Myitem
{
	template<class CMap>
	struct Dart_wrapper
	{
		typedef CGAL::Cell_attribute<CMap, polyhedra_attributes> Polyhedra_attribute; // A number
		typedef CGAL::Cell_attribute<CMap, face_attributes, CGAL::Tag_true, Sum_functor, Divide_by_two_functor > Face_attribute; // area of this face
		typedef CGAL::Cell_attribute<CMap, Point_3> Vertex_attribute; // A vertex
		typedef std::tuple<Vertex_attribute, void, Face_attribute, Polyhedra_attribute> Attributes;
	};
};

// Definition of my combinatorial map.
typedef CGAL::Combinatorial_map<3, Myitem> CMap_3;
typedef CMap_3::Dart_handle Dart_handle;
typedef CMap_3::Attribute_type<0>::type    Vertex_attribute;
typedef CMap_3::Attribute_type<2>::type    Face_attribute;
typedef CMap_3::Attribute_type<3>::type    Polyhedra_attribute;
typedef CMap_3::size_type                  size_type;

Dart_handle make_polygon(CMap_3& amap, KPolygon_2& polygon, Plane_3 plane);
void build_map(CMap_3& cm, KPolygons_SET& polygons_set);
int D(CMap_3& cm, Dart_handle d, int status);
void extract_surface(KPolygons_SET& polygons_set);