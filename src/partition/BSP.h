#pragma once
#include "gui/gl_object.h"
#include "cgal/cgal_object.h"

#include <CGAL/Linear_cell_complex_for_combinatorial_map.h>

namespace BSP {

using LLC_Traits = CGAL::Linear_cell_complex_traits<3, EC::K>;

struct vertex_attributes {
	std::array<size_t, 3> ID;
};
struct face_attributes {
	EC::FT area;
	EC::Plane_3 plane;
	EC::PWN_vector inline_points;
};
struct polyhedra_attributes {
	EC::Polygons_3 polygons_3;
	EC::Point_3 center;
	EC::FT volume = 0;
	int number = -1;
	bool is_ghost = false;
};
struct dart_info {
	EC::Direction_3 direction;
};
struct Myitem
{
	template<class LCC>
	struct Dart_wrapper
	{
		using Dart_info = dart_info;
		using Vertex_attribute = CGAL::Cell_attribute_with_point< LCC, vertex_attributes>;
		using Face_attributes = CGAL::Cell_attribute< LCC, face_attributes>;
		using Polyhedra_attribute = CGAL::Cell_attribute< LCC, polyhedra_attributes>;
		using Attributes = std::tuple<Vertex_attribute, void, Face_attributes, Polyhedra_attribute>;
	};
};
using LCC_3 = CGAL::Linear_cell_complex_for_combinatorial_map<3, 3, LLC_Traits, Myitem>;
using Face_attributes = LCC_3::Attribute_type<2>::type;
using Polyhedra_attribute = LCC_3::Attribute_type<3>::type;
using Dart_handle = LCC_3::Dart_handle;

class BSP_Partition
{
public:
	BSP_Partition(EC::Polygons_3 _polygons_3, float expand_scale = 0.1);
	void partition();
	void partition_next();

	std::unique_ptr<GL::Polygon_Mesh> Get_mesh();
	bool is_done() { return volumes.empty(); }
	LCC_3 lcc;
private:
	std::vector< LCC_3::Dart_handle > volumes;
	EC::Polygons_3 polygons_3;
	int count = 0;
};

template<typename Range>
std::vector<Dart_handle> collect(Range range) {
	std::vector<Dart_handle> dhs;
	for (auto dh = range.begin(); dh != range.end(); dh++)
		dhs.push_back(dh);
	return dhs;
}

}
