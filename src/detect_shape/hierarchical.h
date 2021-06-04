#pragma once
#include <CGAL/linear_least_squares_fitting_3.h>
#include <set>
#include <utility>
#include "gui/gl_object.h"
#include "cgal/cgal_object.h"

namespace Hierarchical {
	using Quadric_Matrix = CGAL::Eigen_diagonalize_traits<IC::FT>::Covariance_matrix;

	struct Plane_region {
		IC::Plane_3 plane{};
		std::vector<IC::face_descriptor> fds{}; // todo: shall we use hierarchical tree?
		std::set<Plane_region*> neighbors{};
		GL::Vec3 color = GL::rand_color();
		//double current_cost = 0;
		// we need QEM to advoid fitting the plane from scratch very time
		// <<Hierarchical Face Clustering on Polygonal Surfaces>>
		// dist quadirc
		Quadric_Matrix quadric_A{};
		IC::Vector_3 quadric_B{0,0,0};
		// quadric_C == fds.size()

		// normal quadirc
		Quadric_Matrix quadric_D{};
		IC::Vector_3 quadric_E{ 0,0,0 };
		// quadric_F == fds.size()
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


	class FaceQEM {
	public:
		using RG_Regions = std::vector<std::vector<std::size_t>>;
		using Mesh = IC::Surface_Mesh;
		using size_type = typename Mesh::size_type;
		using Face_index = typename Mesh::Face_index;

		FaceQEM(const Mesh&, const RG_Regions&);
		~FaceQEM();
		void merge_once(bool print = true);
		void merge_until(float cost = 0.5);
		void merge_until(std::size_t);
		void refine();
		std::unique_ptr<GL::Mesh> get_mesh();
		std::vector<EC::Detected_shape> detected_shape();

	private:
		const Mesh& mesh;
		std::set<Plane_region*> plane_regions; //TODO: use list
		std::set<Merge_Event> queue; //TODO: use priority queue
		std::map<Edge, Merge_Event> edge_event; //TODO: use unordered map?

		void init();
		const Merge_Event& top(void) const { return *(queue.begin()); }
		void pop(void) { queue.erase(queue.begin()); }
		void remove(const Edge& edge);
		void insert(const Edge& edge);
	};

	std::unique_ptr<Hierarchical::FaceQEM> bootstrap_face_qem(IC::Surface_Mesh&);

}