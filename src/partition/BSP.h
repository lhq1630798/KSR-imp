#pragma once
#include "gui/gl_object.h"
#include "cgal/cgal_object.h"

#include <CGAL/Linear_cell_complex_for_combinatorial_map.h>
using LLC_Traits = CGAL::Linear_cell_complex_traits<3, K>;

struct polyhedra_attributes {
	Polygons_3 polygons_3;
	Point_3 center;
};
struct Myitem
{
	template<class LCC>
	struct Dart_wrapper
	{
		using Vertex_attribute = CGAL::Cell_attribute_with_point< LCC >;
		using Polyhedra_attribute = CGAL::Cell_attribute< LCC, polyhedra_attributes>;
		using Attributes = std::tuple<Vertex_attribute, void, void, Polyhedra_attribute>;
	};
};
using LCC_3 = CGAL::Linear_cell_complex_for_combinatorial_map<3, 3, LLC_Traits, Myitem>;
using Polyhedra_attribute = LCC_3::Attribute_type<3>::type;

class BSP_Partition
{
public:
	BSP_Partition(Polygons_3 _polygons_3);
	void partition();
	void partition_next();

	std::unique_ptr<Polygon_Mesh> Get_mesh();

private:
	std::vector< LCC_3::Dart_handle > volumes;
	Polygons_3 polygons_3;
	LCC_3 lcc;
};