#include <CGAL/Polyhedron_3.h>
#include <fstream>
#include "fmt/core.h"
#include "cgal_object.h"

using Polyhedron = CGAL::Polyhedron_3<K>;
using HalfedgeDS = Polyhedron::HalfedgeDS;

// A modifier creating a triangle with the incremental builder.
template <class HDS>
class Build_triangle : public CGAL::Modifier_base<HDS> {
public:
	Build_triangle() {}
	void operator()(HDS& hds) {
		// Postcondition: hds is a valid polyhedral surface.
		CGAL::Polyhedron_incremental_builder_3<HDS> B(hds, true);
		B.begin_surface(3, 1, 6);
		typedef typename HDS::Vertex   Vertex;
		typedef typename Vertex::Point Point;
		B.add_vertex(Point(0, 0, 0));
		B.add_vertex(Point(1, 0, 0));
		B.add_vertex(Point(0, 1, 0));
		B.begin_facet();
		B.add_vertex_to_facet(0);
		B.add_vertex_to_facet(1);
		B.add_vertex_to_facet(2);
		B.end_facet();
		B.end_surface();
	}
};

void test_polyhedron() {
	Polyhedron surface;
	Build_triangle<HalfedgeDS> triangle;
	surface.delegate(triangle);
	std::ofstream file("src/output/outmesh.off");
	file << surface;
}

