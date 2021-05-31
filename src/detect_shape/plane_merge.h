#pragma once
#include <set>
#include <utility>
#include "gui/gl_object.h"
#include "cgal/cgal_object.h"

namespace Plane_Merge {

	struct Plane_region {
		IC::Plane_3 plane;
		//std::vector<IC::PWN> data;
		std::vector<IC::Triangle_3> triangles;
		std::set<Plane_region*> neighbors;
		GL::Vec3 color = GL::rand_color();
	};

	struct Edge {
		Edge(Plane_region* p1, Plane_region* p2) {
			pairs.first = p1 < p2 ? p1 : p2;
			pairs.second = p1 < p2 ? p2 : p1;
		}
		std::pair< Plane_region*, Plane_region*> pairs;
	};
	inline bool operator<(const Edge& edge1, const Edge& edge2)
	{
		return edge1.pairs < edge2.pairs;
	}

	double fitting(Edge, IC::Plane_3& out_plane);

	struct Merge_Event {
		explicit Merge_Event(Edge _edge) : edge(_edge) {
			cost = fitting(edge, plane);
		};
		Edge edge;
		double cost;
		IC::Plane_3 plane;
	};
	inline bool operator<(const Merge_Event& e1, const Merge_Event& e2)
	{
		if (e1.cost != e2.cost)
		{
			return e1.cost < e2.cost;
		}
		return e1.edge < e2.edge;
	}


	class Plane_Merger {
	public:
		using RG_Regions = std::vector<std::vector<std::size_t>>;
		using Mesh = IC::Surface_Mesh;

		Plane_Merger(const Mesh&, const RG_Regions&);
		~Plane_Merger();
		void merge_once();
		void merge_until(float cost = 0.5);
		std::unique_ptr<GL::Mesh> get_mesh();
		std::vector<EC::Detected_shape> detected_shape();

	private:
		std::size_t num_region = 0;
		//std::list<Plane_region> plane_regions;
		std::set<Plane_region*> plane_regions;
		std::set<Merge_Event> queue;
		std::map<Edge, Merge_Event> edge_event;
		const Merge_Event& top(void) const { return *(queue.begin()); }
		void pop(void) { queue.erase(queue.begin()); }
		void remove(const Edge& edge);
	};

	std::unique_ptr<Plane_Merge::Plane_Merger> init_plane_merge(IC::Surface_Mesh);

}