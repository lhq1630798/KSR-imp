#define CGAL_EIGEN3_ENABLED
#include <CGAL/linear_least_squares_fitting_3.h>
#include <fmt/core.h>
#include <execution>
#include <mutex>
#include "plane_merge.h"
#include "region_growing.h"
#include "shape_diameter.h"
#include "util/log.h"

namespace Plane_Merge {

	std::unique_ptr<Plane_Merger> init_plane_merge(IC::Surface_Mesh mesh) {

		Region_Growing::Regions regions;
		if (Config::Detection::get().shape_diameter)
			regions = Region_Growing::region_growing_sdf(mesh);
		else
			regions = Region_Growing::region_growing_on_mesh(mesh);

		return std::make_unique<Plane_Merger>(mesh, regions);

	}

	double fitting(Edge edge, IC::Plane_3& out_plane) {
		auto* p1 = edge.pairs.first;
		auto* p2 = edge.pairs.second;

		auto triangles = p1->triangles;
		triangles.insert(triangles.end(), p2->triangles.begin(), p2->triangles.end());
		IC::Point_3 centroid;
		auto quality = linear_least_squares_fitting_3(
			triangles.begin(), 
			triangles.end(), 
			out_plane,
			centroid,
			CGAL::Dimension_tag<2>(),
			IC::K(),
			CGAL::Eigen_diagonalize_traits<IC::FT>()
		);
		//double cost = 0;
		//for (const auto& tri : triangles) {
		//	//tri.squared_area
		//}
		double cost = (1 - quality) * triangles.size();
		//double cost = (1 - quality) * 1e3;
		return cost;
	}


	Plane_Merger::Plane_Merger(const Mesh& mesh, const RG_Regions& detected_regions)
	{
		using size_type = typename Mesh::size_type;
		using Face_index = typename Mesh::Face_index;

		std::map<std::size_t, Plane_region*> index_to_plane; //TODO::use vector?

		// insert node
		for (const auto& region : detected_regions) {

			//Plane_region* new_plane_region = &*plane_regions.insert(plane_regions.end(), Plane_region{});
			Plane_region* new_plane_region = new Plane_region;
			plane_regions.insert(new_plane_region);
			for (const auto index : region) {
				index_to_plane[index] = new_plane_region;

				std::vector<IC::Point_3> points;
				auto h = mesh.halfedge(Face_index(static_cast<size_type>(index)));
				for (auto vd : mesh.vertices_around_face(h)) {
					points.push_back(mesh.point(vd));
				}
				assert(points.size() == 3);
				new_plane_region->triangles.push_back(IC::Triangle_3{ points[0],points[1],points[2] });
			}

			linear_least_squares_fitting_3(
				new_plane_region->triangles.begin(),
				new_plane_region->triangles.end(),
				new_plane_region->plane,
				CGAL::Dimension_tag<2>()
			);
			num_region++;
		}
		fmt::print("insert node done\n");

		// build adjacency
		for (const auto& region : detected_regions) {
			Plane_region* plane_region = index_to_plane[region[0]];
			for (const auto index : region) {
				auto h = mesh.halfedge(Face_index(static_cast<size_type>(index)));
				for (auto fd : mesh.faces_around_face(h)) {
					auto index = static_cast<std::size_t>(fd); // what if -1?
					auto neighbor_region = index_to_plane.find(index);
					if (neighbor_region != index_to_plane.end() && //a discarded face in region growing
						neighbor_region->second != plane_region)
						plane_region->neighbors.insert(neighbor_region->second);
				}
			}
		}
		fmt::print("build adjacency done\n");

		// init queue
		//for (auto& plane_region : plane_regions) {
		//	for (Plane_region* neighbor : plane_region.neighbors) {
		//		Merge_Event e{ &plane_region, neighbor };
		//		queue.insert(e);
		//	}
		//}
		std::mutex queue_mutex;
		std::for_each(std::execution::par, plane_regions.begin(), plane_regions.end(),
			[&](Plane_region* plane_region) {
				std::vector<std::pair<Edge, Merge_Event>> events;
				for (Plane_region* neighbor : plane_region->neighbors) {
					assert(plane_region != neighbor);
					Edge edge{ plane_region, neighbor };
					Merge_Event event{ edge };
					events.push_back({ edge, event });
				}
				std::lock_guard<std::mutex> guard(queue_mutex);
				for (auto& [edge, event] : events) {
					if (edge_event.find(edge) == edge_event.end()) {
						queue.insert(event);
						edge_event.insert({ edge,event });
					}
				}
			}
		);

		fmt::print("init queue done\n");

	}


	bool need_update(Edge edge) {
		auto* small = edge.pairs.first;
		auto* large = edge.pairs.second;
		if (small->triangles.size() > large->triangles.size())
			std::swap(small, large);
		float rate = small->triangles.size() * 1.0 / large->triangles.size();
		if (rate < 0.1) return false;
		return true;
	}

	Plane_Merger::~Plane_Merger()
	{
		for (Plane_region* plane_region : plane_regions) {
			free(plane_region);
		}
	}

	void Plane_Merger::merge_once()
	{
		if (queue.empty())
			return;

		auto data_size = top().edge.pairs.first->triangles.size() + top().edge.pairs.second->triangles.size();
		auto neighbor_size = top().edge.pairs.first->neighbors.size() + top().edge.pairs.second->neighbors.size();
		fmt::print("\r merging {}..., data size {}, neighbor {},  queue size {}, cost {:4}   ", 
			num_region, data_size, neighbor_size, queue.size(), top().cost);

		auto event = top();
		auto [region1, region2] = event.edge.pairs;
		pop();
		bool update = need_update(event.edge);

		//Plane_region* new_plane_region = &*plane_regions.insert(plane_regions.end(), Plane_region{});
		Plane_region* new_plane_region = new Plane_region;
		plane_regions.insert(new_plane_region);

		new_plane_region->plane = event.plane;

		new_plane_region->triangles = std::move(region1->triangles);
		new_plane_region->triangles.insert(
			new_plane_region->triangles.end(),
			region2->triangles.begin(),
			region2->triangles.end()
		);



		if (update) {
			//remove queue
			for (Plane_region* neigh : region1->neighbors) {
				remove(Edge{ region1, neigh });
			}
			for (Plane_region* neigh : region2->neighbors) {
				if (neigh == region1) continue;
				remove(Edge{ region2, neigh });
			}

			// update graph
			new_plane_region->neighbors = region1->neighbors;
			new_plane_region->neighbors.insert(
				region2->neighbors.begin(),
				region2->neighbors.end()
			);
			new_plane_region->neighbors.erase(region1);
			new_plane_region->neighbors.erase(region2);

			for (Plane_region* neigh : region1->neighbors) {
				neigh->neighbors.erase(region1);
			}
			for (Plane_region* neigh : region2->neighbors) {
				neigh->neighbors.erase(region2);
			}
			for (Plane_region* neigh : new_plane_region->neighbors) {
				neigh->neighbors.insert(new_plane_region);
			}

			// update queue
			std::vector<std::pair<Edge, Merge_Event>> events;
			//for (Plane_region* neighbor : new_plane_region->neighbors) {
			//	Merge_Event e{ new_plane_region, neighbor };
			//	events.push_back(e);
			//}
			std::mutex mutex;
			std::for_each(std::execution::par, new_plane_region->neighbors.begin(), new_plane_region->neighbors.end(),
				[&](Plane_region* neighbor) {
				Edge edge{ new_plane_region, neighbor };
				Merge_Event event{ edge };
				assert(new_plane_region != neighbor);
				std::lock_guard<std::mutex> guard(mutex);
				events.push_back({ edge,event });
			}
			);
			for (auto& [edge, event] : events) {
				assert(edge_event.find(edge) == edge_event.end());
				queue.insert(event);
				edge_event.insert({ edge,event });
			}
		}
		else {
			auto* small = event.edge.pairs.first;
			auto* large = event.edge.pairs.second;
			if (small->triangles.size() > large->triangles.size())
				std::swap(small, large);

			// dont fit for the larger region's neighbor
			// because the new region's plane is close to the larger region's one
			for (Plane_region* neigh : large->neighbors) {
				auto event = edge_event.find(Edge{ large, neigh })->second;
				remove(Edge{ large, neigh });

				if (neigh == small) continue;
				Edge new_edge{ new_plane_region, neigh };
				event.edge = new_edge;
				queue.insert(event);
				edge_event.insert({ new_edge, event });
			}

			// fit for the smaller region's neighbors
			// because the new region's plane is far from the smaller region's one
			std::vector<Edge> new_edges;
			for (Plane_region* neigh : small->neighbors) {
				if (neigh == large) continue;
				remove(Edge{ small, neigh });

				assert(new_plane_region != neigh);
				Edge edge{ new_plane_region, neigh };
				if (edge_event.find(edge) == edge_event.end())
					new_edges.push_back(edge);
			}
			fmt::print("update neighbor {}   ",
				new_edges.size());
			//assert(new_edges.size() * 9 < large->neighbors.size());

			std::vector<std::pair<Edge, Merge_Event>> events;
			std::mutex mutex;
			std::for_each(std::execution::par, new_edges.begin(), new_edges.end(),
				[&](Edge& edge) {
				Merge_Event event{ edge };
				std::lock_guard<std::mutex> guard(mutex);
				events.push_back({ edge,event });
			}
			);
			for (auto& [edge, event] : events) {
				queue.insert(event);
				edge_event.insert({ edge,event });
			}

			// update graph
			new_plane_region->neighbors = region1->neighbors;
			new_plane_region->neighbors.insert(
				region2->neighbors.begin(),
				region2->neighbors.end()
			);
			new_plane_region->neighbors.erase(region1);
			new_plane_region->neighbors.erase(region2);

			for (Plane_region* neigh : region1->neighbors) {
				neigh->neighbors.erase(region1);
			}
			for (Plane_region* neigh : region2->neighbors) {
				neigh->neighbors.erase(region2);
			}
			for (Plane_region* neigh : new_plane_region->neighbors) {
				neigh->neighbors.insert(new_plane_region);

				Edge edge{ new_plane_region, neigh };
				assert(edge_event.find(edge) != edge_event.end());
				auto& event = edge_event.find(edge)->second;
				assert(queue.find(event) != queue.end());
			}
		}


		//
		plane_regions.erase(region1);
		plane_regions.erase(region2);
		free(region1);
		free(region2);

		num_region--;
	}

	void Plane_Merger::merge_until(float cost)
	{
		while (!queue.empty() && top().cost < cost) {
			merge_once();
		}
		fmt::print("\nmerge done, remain {} planes\n", plane_regions.size());
	}

	std::unique_ptr<GL::Mesh> Plane_Merger::get_mesh()
	{
		std::vector<GL::Vert> verts;
		std::vector<GL::Mesh::Index> idxs;
		int count = 0;
		for (Plane_region* plane_region : plane_regions) {
			auto color = plane_region->color;
			for (auto& triangle : plane_region->triangles)
			{
				auto p0 = plane_region->plane.projection(triangle.vertex(0));
				auto p1 = plane_region->plane.projection(triangle.vertex(1));
				auto p2 = plane_region->plane.projection(triangle.vertex(2));
				verts.push_back({ p0,color });
				verts.push_back({ p1,color });
				verts.push_back({ p2,color });
				idxs.push_back(count);
				idxs.push_back(count + 1);
				idxs.push_back(count + 2);
				count = count + 3;
			}
		}
		auto m = GL::Mesh{ verts,idxs };
		return std::make_unique<GL::Mesh>(m);

	}

	std::vector<EC::Detected_shape> Plane_Merger::detected_shape()
	{
		IK_to_EK to_EK;
		std::vector<EC::Detected_shape> detected_shapes;
		for (Plane_region* plane_region : plane_regions) {
			assert(!plane_region->triangles.empty());
			auto plane = plane_region->plane;
			std::vector<EC::PWN> pwns;
			IC::Vector_3 average_normal;
			for (auto& triangle : plane_region->triangles)
			{
				auto center = CGAL::centroid(triangle);
				auto normal = CGAL::unit_normal(triangle.vertex(0), triangle.vertex(1), triangle.vertex(2));
				average_normal += normal;
				//pwns.push_back({ to_EK(center), to_EK(normal) });
				pwns.push_back({ to_EK(triangle.vertex(0)), to_EK(normal) });
				pwns.push_back({ to_EK(triangle.vertex(1)), to_EK(normal) });
				pwns.push_back({ to_EK(triangle.vertex(2)), to_EK(normal) });
			}
			average_normal /= plane_region->triangles.size();
			if (plane.orthogonal_vector() * average_normal < 0)
				plane = plane.opposite();
			detected_shapes.push_back({ to_EK(plane) , pwns });
		}
		return detected_shapes;
	}

	inline void Plane_Merger::remove(const Edge& edge)
	{
		assert(edge_event.find(edge) != edge_event.end());
		auto& event = edge_event.find(edge)->second;
		if (queue.find(event) != queue.end())
		{
			queue.erase(event);
		}
		edge_event.erase(edge);
	}
}


