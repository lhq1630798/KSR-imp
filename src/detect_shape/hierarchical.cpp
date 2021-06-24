#define CGAL_EIGEN3_ENABLED
#include <CGAL/linear_least_squares_fitting_3.h>
#include <CGAL/Polygon_mesh_processing/shape_predicates.h>
#include <CGAL/Regularization/regularize_planes.h>
#include <fmt/core.h>
#include <execution>
#include <mutex>
#include "hierarchical.h"
#include "region_growing.h"
#include "shape_diameter.h"
#include "util/log.h"
#include "util/config.h"
#include <CGAL/PCA_util.h>

namespace Hierarchical {

	void save_regions_mesh(IC::Surface_Mesh mesh, std::vector<std::pair<std::vector<IC::face_descriptor>, GL::Vec3> >& regions, std::string method) {
		//save region growing result
		using Color = CGAL::Color;
		auto face_color = mesh.add_property_map<IC::Surface_Mesh::Face_index, Color>("f:color", Color(0, 0, 0)).first;
		// Iterate through all regions.
		for (const auto& region : regions) {
			// Generate a random color.
			const Color color(
				static_cast<unsigned char>(region.second[0] * 256),
				static_cast<unsigned char>(region.second[1] * 256),
				static_cast<unsigned char>(region.second[2] * 256));
			// Iterate through all region items.
			for (const auto index : region.first)
				face_color[index] = color;
		}
		std::string path = Config::read<std::string>("save_path") + method + "_regions_mesh.off";
		std::ofstream file(path, std::ios::binary);
		file << mesh;
		file.close();
	}

	void save_regions_point(const IC::PWN_vector& points, std::vector<std::pair<std::vector<size_t>, GL::Vec3> >& regions, std::string method) {
		//save region growing result
		IC::Surface_Mesh mesh;
		using Color = CGAL::Color;
		auto vertex_color = mesh.add_property_map<IC::Surface_Mesh::Vertex_index, Color>("v:color", Color(0, 0, 0)).first;
		// Iterate through all regions.
		for (const auto& region : regions) {
			// Generate a random color.
			const Color color(
				static_cast<unsigned char>(region.second[0] * 256),
				static_cast<unsigned char>(region.second[1] * 256),
				static_cast<unsigned char>(region.second[2] * 256));
			// Iterate through all region items.
			for (const auto index : region.first) {
				IC::vertex_descriptor v0 = mesh.add_vertex((*(points.begin() + index)).first);
				vertex_color[v0] = color;
			}
		}
		std::string path = Config::read<std::string>("save_path") + method + "_regions_point.off";
		std::ofstream file(path, std::ios::binary);
		file << mesh;
		file.close();
	}
	
	std::unique_ptr<FaceQEM> bootstrap_face_qem(IC::Surface_Mesh& mesh) {

		Region_Growing::Regions regions;
		if (Config::Detection::get().shape_diameter)
			regions = Region_Growing::region_growing_sdf(mesh);
		else
			regions = Region_Growing::region_growing_on_mesh(mesh);

		return std::make_unique<FaceQEM>(mesh, regions);

	}


	// covariance = A - bTb/c
	Quadric_Matrix qem_cova(const Quadric_Matrix& A, const IC::Vector_3& B, const IC::FT C) {
		Quadric_Matrix covariance;
		covariance = A;
		// Matrix numbering:
		// 0 1 2
		//   3 4
		//     5
		covariance[0] -= B.x() * B.x() / C;
		covariance[1] -= B.x() * B.y() / C;
		covariance[2] -= B.x() * B.z() / C;
		covariance[3] -= B.y() * B.y() / C;
		covariance[4] -= B.y() * B.z() / C;
		covariance[5] -= B.z() * B.z() / C;
		return covariance;
	}

	double qem_fit_plane_3(const Quadric_Matrix& A, const IC::Vector_3& B, const IC::FT C, IC::Plane_3& out_plane) {
		Quadric_Matrix covariance = qem_cova(A, B, C);
		auto center = CGAL::ORIGIN + B / C;
		auto quality = CGAL::internal::fitting_plane_3(
			covariance,
			center,
			out_plane,
			IC::K(),
			CGAL::Eigen_diagonalize_traits<IC::FT>());
		return quality;
	}

	//quadric
	double vTMv(const Quadric_Matrix& M, const IC::Vector_3& v) {
		IC::Vector_3 M_col0{ M[0],M[1],M[2] };
		IC::Vector_3 M_col1{ M[1],M[3],M[4] };
		IC::Vector_3 M_col2{ M[2],M[4],M[5] };
		double quadric = (v * v.x() * M_col0) + (v * v.y() * M_col1) + (v * v.z() * M_col2);
		return quadric;
	}

	Quadric_Matrix add_matrix(const Quadric_Matrix& M1, const Quadric_Matrix& M2) {
		Quadric_Matrix quadric_M{ {M1[0] + M2[0],M1[1] + M2[1],M1[2] + M2[2],M1[3] + M2[3],M1[4] + M2[4],M1[5] + M2[5]} };
		return quadric_M;
	}

	double dist_cost(const IC::Point_3& point, const IC::Plane_3& plane) {
		return CGAL::squared_distance(point, plane);
	}
	// dist_cost = (nTAn + 2BT(dn) + Cd2) / k
	double qem_dist_cost(const Quadric_Matrix& A, const IC::Vector_3& B, const IC::FT C, const IC::Plane_3& plane) {
		double cost = 0;

		auto n = plane.orthogonal_vector(); // {a,b,c}
		auto d = plane.d();
		//nTAn
		cost += vTMv(A,n);
		//2bT(dn)
		cost += 2 * B * d * n;
		//cd2
		cost += C * d * d;

		if (cost < 0) cost = 0;
		return cost;
	}

	double normal_cost(const IC::Vector_3& n, const IC::Plane_3& plane) {
		auto cost = 1 - n * plane.orthogonal_vector();
		return cost * cost;
	}
	// normal_cost = (nTDn + 2ETn + F) / k
	double qem_normal_cost(const Quadric_Matrix& D, const IC::Vector_3& E, const IC::FT F, const IC::Plane_3& plane) {
		double cost = 0;

		auto n = plane.orthogonal_vector(); // {a,b,c}
		// correct normal oritation before calculate normal cost!
		if (n * -E < 0)
			n = -n;
		//nTAn
		cost += vTMv(D, n);
		//2ETn
		cost += 2 * E * n;
		//cd2
		cost += F;

		if (cost < 0) cost = 0;
		return cost;
	}

	//bool near_orthogonal(const IC::Vector_3& n1, const IC::Vector_3& n2) {
	//	if (abs(n1 * n2) < 0.15)//81��
	//		return true;
	//	return false;
	//}
	double fitting_cost(const IC::Surface_Mesh& mesh, IC::face_descriptor fd, const IC::Plane_3& plane) {
		const auto& param = Config::Detection::get();
		std::vector<IC::Point_3> points;
		for (auto vd : mesh.vertices_around_face(mesh.halfedge(fd))) {
			points.push_back(mesh.point(vd));
		}
		assert(points.size() == 3);
		auto center = CGAL::centroid(points[0], points[1], points[2]);
		auto normal = CGAL::unit_normal(points[0], points[1], points[2]);
		auto cost = param.qem_a1 * dist_cost(center, plane) +
			param.qem_a2 * normal_cost(normal, plane);
		return cost;
	}
	double fitting_cost(
		const Quadric_Matrix& A,
		const IC::Vector_3& B,
		const IC::FT C,
		const Quadric_Matrix& D,
		const IC::Vector_3& E,
		const IC::FT F,
		const IC::Plane_3& plane) 
	{
		assert(C == F);
		const auto& param = Config::Detection::get();
		double dist_cost = qem_dist_cost(A, B, C, plane);
		double normal_cost = 0;
		if (C >= 3)
			normal_cost = qem_normal_cost(D, E, F, plane);
		return param.qem_a1 * dist_cost + param.qem_a2 * normal_cost;
	}

	double fitting(Edge edge, IC::Plane_3& out_plane) {
		auto* p1 = edge.pairs.first;
		auto* p2 = edge.pairs.second;

		//if(near_orthogonal(p1->plane.orthogonal_vector(), p2->plane.orthogonal_vector()))
		//	return 1e100;


		Quadric_Matrix quadric_A = add_matrix(p1->quadric_A, p2->quadric_A);
		auto quadric_B = p1->quadric_B + p2->quadric_B;
		auto quadric_C = p1->fds.size() + p2->fds.size();
		Quadric_Matrix quadric_D = add_matrix(p1->quadric_D, p2->quadric_D);
		auto quadric_E = p1->quadric_E + p2->quadric_E;
		auto quadric_F = p1->fds.size() + p2->fds.size();

		auto quality = qem_fit_plane_3(quadric_A, quadric_B, quadric_C, out_plane);
		if (out_plane.orthogonal_vector() * -quadric_E < 0)
			out_plane = out_plane.opposite();

		auto cost = fitting_cost(quadric_A, quadric_B, quadric_C, quadric_D, quadric_E, quadric_F, out_plane);
		cost = cost / (p1->fds.size() + p2->fds.size());
		assert(std::isfinite(cost));
		return cost;

		//double p1_cost = fitting_cost(
		//	p1->quadric_A,
		//	p1->quadric_B,
		//	p1->fds.size(),//C
		//	p1->quadric_D,
		//	p1->quadric_E,
		//	p1->fds.size(),//F
		//	out_plane
		//) / std::max(int(p1->fds.size()), 20);
		//double p2_cost = fitting_cost(
		//	p2->quadric_A,
		//	p2->quadric_B,
		//	p2->fds.size(),//C
		//	p2->quadric_D,
		//	p2->quadric_E,
		//	p2->fds.size(),//F
		//	out_plane
		//) / std::max(int(p2->fds.size()), 20);
		//return std::max(p1_cost,p2_cost);
	}


	FaceQEM::FaceQEM(const Mesh& _mesh, const RG_Regions& detected_regions)
		: mesh(_mesh)
	{

		// insert node
		for (const auto& region : detected_regions) {

			//Plane_region* new_plane_region = &*plane_regions.insert(plane_regions.end(), Plane_region{});
			Plane_region* new_plane_region = new Plane_region;
			plane_regions.insert(new_plane_region);
			for (const auto index : region) {
				auto fd = Face_index(static_cast<size_type>(index));
				new_plane_region->fds.push_back(fd);
			}
		}

		init();

		//save result
		std::vector<std::pair<std::vector<IC::face_descriptor>, GL::Vec3> > regions;
		for (Plane_region* plane_region : plane_regions) {
			std::vector<IC::face_descriptor> region;
			for (auto& fd : plane_region->fds)
			{
				region.push_back(fd);
			}
			regions.push_back(std::make_pair(region, plane_region->color));
		}
		const auto& params = Config::Detection::get();
		if (params.save_result) {
			save_regions_mesh(mesh, regions, "RG");
		}
	}

	void FaceQEM::init() {
		queue.clear();
		edge_event.clear();

		std::vector<Plane_region*> index_to_plane(mesh.num_faces(), nullptr);
		for (Plane_region* plane_region : plane_regions) {
			plane_region->quadric_A = Quadric_Matrix{ {0,0,0,0,0,0} };
			plane_region->quadric_B = IC::Vector_3{ 0,0,0 };
			plane_region->quadric_D = Quadric_Matrix{ {0,0,0,0,0,0} };
			plane_region->quadric_E = IC::Vector_3{ 0,0,0 };

			for (auto fd : plane_region->fds) {
				index_to_plane[fd] = plane_region;

				std::vector<IC::Point_3> points;
				auto h = mesh.halfedge(fd);
				for (auto vd : mesh.vertices_around_face(h)) {
					points.push_back(mesh.point(vd));
				}
				assert(points.size() == 3);
				auto center = CGAL::centroid(points[0], points[1], points[2]);

				// Matrix numbering:
				// 0 1 2
				//   3 4
				//     5
				auto& quadric_A = plane_region->quadric_A;
				quadric_A[0] += center.x() * center.x();
				quadric_A[1] += center.x() * center.y();
				quadric_A[2] += center.x() * center.z();
				quadric_A[3] += center.y() * center.y();
				quadric_A[4] += center.y() * center.z();
				quadric_A[5] += center.z() * center.z();
				plane_region->quadric_B += center - CGAL::ORIGIN;


				if (CGAL::Polygon_mesh_processing::is_degenerate_triangle_face(fd, mesh)) {
					//TODO:: use area weight
					continue;
				}
				auto normal = CGAL::unit_normal(points[0], points[1], points[2]);
				auto& quadric_D = plane_region->quadric_D;
				quadric_D[0] += normal.x() * normal.x();
				quadric_D[1] += normal.x() * normal.y();
				quadric_D[2] += normal.x() * normal.z();
				quadric_D[3] += normal.y() * normal.y();
				quadric_D[4] += normal.y() * normal.z();
				quadric_D[5] += normal.z() * normal.z();
				plane_region->quadric_E += -normal;
			}
			
			auto quality = qem_fit_plane_3(
				plane_region->quadric_A,
				plane_region->quadric_B,
				plane_region->fds.size(),
				plane_region->plane);
			if (plane_region->plane.orthogonal_vector() * -plane_region->quadric_E < 0)
				plane_region->plane = plane_region->plane.opposite();
			//new_plane_region->current_cost = fitting_cost(
			//	new_plane_region->quadric_A,
			//	new_plane_region->quadric_B,
			//	new_plane_region->fds.size(),//C
			//	new_plane_region->quadric_D,
			//	new_plane_region->quadric_E,
			//	new_plane_region->fds.size(),//F
			//	new_plane_region->plane
			//);

		}
		fmt::print("init node done\n");

		// build adjacency
		for (Plane_region* plane_region : plane_regions) {
			plane_region->neighbors.clear();
			for (auto fd : plane_region->fds) {
				auto h = mesh.halfedge(Face_index(static_cast<size_type>(fd)));
				for (auto fd : mesh.faces_around_face(h)) {
					if (fd == mesh.null_face()) continue;
					if (index_to_plane[fd] && //a discarded face in region growing
						index_to_plane[fd] != plane_region)
						plane_region->neighbors.insert(index_to_plane[fd]);
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
		{ // parallel version
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
		}
		fmt::print("init queue done\n");
	}


	FaceQEM::~FaceQEM()
	{
		for (Plane_region* plane_region : plane_regions) {
			free(plane_region);
		}
	}

	void FaceQEM::merge_once(bool print)
	{
		if (queue.empty())
			return;

		if (print || plane_regions.size() < 1e3 || plane_regions.size() % 1000 == 0) {
			fmt::print("\r merging {}...,  queue size {}, cost {:4}   ",
				plane_regions.size(), queue.size(), 1e5*top().cost);
		}
		//if (top().cost > 1e10) {
		//	fmt::print("error: cost{}   \n", top().cost);
		//	return;
		//}

		auto event = top();
		auto [region1, region2] = event.edge.pairs;
		pop();

		Plane_region* new_plane_region = new Plane_region;
		plane_regions.insert(new_plane_region);

		new_plane_region->quadric_A = add_matrix(region1->quadric_A, region2->quadric_A);
		new_plane_region->quadric_B = region1->quadric_B + region2->quadric_B;
		new_plane_region->quadric_D = add_matrix(region1->quadric_D, region2->quadric_D);
		new_plane_region->quadric_E = region1->quadric_E + region2->quadric_E;
		new_plane_region->plane = event.plane;
		new_plane_region->fds = std::move(region1->fds);
		new_plane_region->fds.insert(
			new_plane_region->fds.end(),
			region2->fds.begin(),
			region2->fds.end()
		);
		//new_plane_region->current_cost = fitting_cost(
		//	new_plane_region->quadric_A,
		//	new_plane_region->quadric_B,
		//	new_plane_region->fds.size(),//C
		//	new_plane_region->quadric_D,
		//	new_plane_region->quadric_E,
		//	new_plane_region->fds.size(),//F
		//	new_plane_region->plane
		//);

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
		for (Plane_region* neighbor : new_plane_region->neighbors) {
			insert(Edge{ new_plane_region, neighbor });
		}
		{ // parallel version
			//std::vector<std::pair<Edge, Merge_Event>> events;
			//std::mutex mutex;
			//std::for_each(std::execution::par, new_plane_region->neighbors.begin(), new_plane_region->neighbors.end(),
			//	[&](Plane_region* neighbor) {
			//	Edge edge{ new_plane_region, neighbor };
			//	Merge_Event event{ edge };
			//	assert(new_plane_region != neighbor);
			//	std::lock_guard<std::mutex> guard(mutex);
			//	events.push_back({ edge,event });
			//}
			//);
			//for (auto& [edge, event] : events) {
			//	assert(edge_event.find(edge) == edge_event.end());
			//	queue.insert(event);
			//	edge_event.insert({ edge,event });
			//}
		}


		plane_regions.erase(region1);
		plane_regions.erase(region2);
		free(region1);
		free(region2);

	}

	void FaceQEM::merge_until(float cost)
	{
		while (!queue.empty() && 1e5 * top().cost < cost) {
			merge_once(false);
		}
		fmt::print("\nmerge done, remain {} planes\n", plane_regions.size());
	}
	void FaceQEM::merge_until(std::size_t num) {
		while (plane_regions.size() > num) {
			merge_once(false);
		}
		fmt::print("\nmerge done, remain {} planes\n", plane_regions.size());
	}

	void FaceQEM::refine()
	{
		//multi-sources region growing
		std::vector<bool> visited(mesh.num_faces(), false);
		struct RG_Event {
			IC::face_descriptor fd;
			Plane_region* region;
			double cost;
		};
		auto cmp = [](const RG_Event& left, const RG_Event& right) {
			return left.cost > right.cost;
		};
		std::priority_queue<RG_Event, std::vector<RG_Event>, decltype(cmp)> RG_queue(cmp);

		// init queue
		for (Plane_region* plane_region : plane_regions) {
			IC::face_descriptor seed = plane_region->fds[0];
			double seed_cost = 1e100;
			auto plane = plane_region->plane;
			for (auto& fd : plane_region->fds) {
				auto cost = fitting_cost(mesh, fd, plane);
				if (cost < seed_cost) {
					seed = fd;
					seed_cost = cost;
				}
			}
			visited[seed] = true;
			for (auto fd : mesh.faces_around_face(mesh.halfedge(seed))) {
				if (fd == mesh.null_face()) continue;
				auto cost = fitting_cost(mesh, fd, plane);
				RG_queue.push(RG_Event{ fd,plane_region,cost });
			}
			auto color = plane_region->color;
			*plane_region = Plane_region{}; //clear
			plane_region->fds.push_back(seed);
			plane_region->plane = plane;//keep plane for region growing
			plane_region->color = color;
		}

		//growing
		while (!RG_queue.empty()) {
			auto event = RG_queue.top();
			RG_queue.pop();
			if (visited[event.fd])
				continue;
			visited[event.fd] = true;
			event.region->fds.push_back(event.fd);
			for (auto fd : mesh.faces_around_face(mesh.halfedge(event.fd))) {
				if (fd == mesh.null_face()) continue;
				if (visited[fd])
					continue;
				auto cost = fitting_cost(mesh, fd, event.region->plane);
				RG_queue.push(RG_Event{ fd, event.region, cost });
			}
		}

		init();


	}

	std::unique_ptr<GL::Mesh> FaceQEM::get_mesh()
	{
		// assert is triangle mesh
		std::vector<GL::Vert> verts;
		std::vector<GL::Mesh::Index> idxs;
		int count = 0;
		for (Plane_region* plane_region : plane_regions) {
			auto color = plane_region->color;
			for (auto& fd : plane_region->fds)
			{
				for (auto vd : mesh.vertices_around_face(mesh.halfedge(fd))) {
					verts.push_back({ mesh.point(vd),color });
					idxs.push_back(count++);
				}
			}
		}
		auto m = GL::Mesh{ verts,idxs };
		return std::make_unique<GL::Mesh>(m);

	}

	std::vector<EC::Detected_shape> FaceQEM::detected_shape()
	{
		//save MRG results
		std::vector<std::pair<std::vector<IC::face_descriptor>, GL::Vec3> > regions;
		for (Plane_region* plane_region : plane_regions) {
			std::vector<IC::face_descriptor> region;
			for (auto& fd : plane_region->fds)
			{
				region.push_back(fd);
			}
			regions.push_back(std::make_pair(region, plane_region->color));
		}
		const auto& params = Config::Detection::get();
		if (params.save_result) {
			save_regions_mesh(mesh, regions, "MRG");
		}

		if(Config::Detection::get().use_regularization){
			std::vector<IC::Plane_3> planes{};
			std::vector<IC::Point_3> points{};
			std::vector<int> point_plane_index_map{};
			for (Plane_region* plane_region : plane_regions) {
				for (auto& fd : plane_region->fds){
					point_plane_index_map.push_back(planes.size());

					std::vector<IC::Point_3> face_vertice;;
					for (auto vd : mesh.vertices_around_face(mesh.halfedge(fd))) {
						face_vertice.push_back(mesh.point(vd));
					}
					assert(face_vertice.size() == 3);
					auto center = CGAL::centroid(face_vertice[0], face_vertice[1], face_vertice[2]);
					points.push_back(center);
					}
				planes.push_back(plane_region->plane);
			}

			const auto& params = Config::Regularization::get();
			CGAL::regularize_planes(points,
				CGAL::Identity_property_map<IC::Point_3>(),
				planes,
				CGAL::Identity_property_map<IC::Plane_3>(),
				CGAL::make_property_map(point_plane_index_map),
				params.parallelism,  // regularize parallelism
				params.orthogonality,  // regularize orthogonality
				params.coplanarity, // regularize coplanarity
				params.Z_symmetry,  // regularize Z-symmetry 
				params.paral_degree,  // 1 degrees of tolerance for parallelism / orthogonality
				params.coplane_dist // tolerance coplanarity
			);

			int ind = 0;
			for (Plane_region* plane_region : plane_regions) {
				plane_region->plane = planes[ind++];
			}
		}

		IK_to_EK to_EK;
		std::vector<EC::Detected_shape> detected_shapes;
		for (Plane_region* plane_region : plane_regions) {
			assert(!plane_region->fds.empty());
			if (plane_region->fds.size() <= 3) continue;

			auto plane = plane_region->plane;
			std::vector<EC::PWN> pwns;
			IC::Vector_3 average_normal = CGAL::NULL_VECTOR;

			for (auto& fd : plane_region->fds)
			{
				std::vector<IC::Point_3> points;
				for (auto vd : mesh.vertices_around_face(mesh.halfedge(fd))) {
					points.push_back(mesh.point(vd));
				}
				assert(points.size() == 3);
				auto center = CGAL::centroid(points[0], points[1], points[2]);
				auto normal = CGAL::unit_normal(points[0], points[1], points[2]);
				average_normal += normal;
				//pwns.push_back({ to_EK(points[0]), to_EK(normal) });
				//pwns.push_back({ to_EK(points[1]), to_EK(normal) });
				//pwns.push_back({ to_EK(points[2]), to_EK(normal) });
				pwns.push_back({ to_EK(center), to_EK(normal) });
			}
			average_normal /= plane_region->fds.size();
			if (plane.orthogonal_vector() * average_normal < 0)
				plane = plane.opposite();
			detected_shapes.push_back({ to_EK(plane) , pwns });
		}

		return detected_shapes;
	}

	void FaceQEM::remove(const Edge& edge)
	{
		auto event_key = edge_event.find(edge);
		assert(event_key != edge_event.end());
		queue.erase(event_key->second);
		edge_event.erase(event_key);
	}
	void FaceQEM::insert(const Edge& edge)
	{
		Merge_Event event{ edge };
		queue.insert(event);
		edge_event.insert({ edge,event });
	}


	std::unique_ptr<PointQEM> bootstrap_points_qem(IC::PWN_vector& points) {

		Region_Growing::Regions regions;
		regions = Region_Growing::region_growing_on_points(points);

		return std::make_unique<PointQEM>(points, regions);
	}

	PointQEM::PointQEM(const Points& _points, const RG_Regions& detected_regions)
		: points(_points)
	{
		
		for (const auto& region : detected_regions) {

			Point_region* new_point_region = new Point_region;
			point_regions.insert(new_point_region);
			for (const auto index : region) {
				new_point_region->vds.push_back(index);
			}
		}
		init();

		//save result
		std::vector<std::pair<std::vector<size_t>, GL::Vec3> > regions;
		for (Point_region* point_region : point_regions) {
			std::vector<size_t> region;
			for (auto& vd : point_region->vds)
			{
				region.push_back(vd);
			}
			regions.push_back(std::make_pair(region,point_region->color));
		}
		const auto& params = Config::Detection::get();
		if (params.save_result) {
			save_regions_point(points, regions, "RG");
		}
	}

	void PointQEM::init() {
		
		for (Point_region* point_region : point_regions) 
		{
			std::vector<IC::Point_3> points_coord;
			IC::Vector_3 average_normal = CGAL::NULL_VECTOR;
			// Iterate through all region items.
			for (auto index : point_region->vds) {
				const IC::PWN& point = *(points.begin() + index);
				points_coord.push_back(point.first);
				average_normal += point.second;
			}
			average_normal /= point_region->vds.size();

			// The best fit plane will be a plane fitted to all region points with
			// its normal being perpendicular to the plane.
			IC::Plane_3 plane;
			//linear_least_squares_fitting_3(points_coord.begin(), points_coord.end(), plane, CGAL::Dimension_tag<0>());
			IC::Point_3 centroid;
			linear_least_squares_fitting_3(
				points_coord.begin(),
				points_coord.end(),
				plane,
				centroid,
				CGAL::Dimension_tag<0>(),
				IC::K(),
				CGAL::Eigen_diagonalize_traits<IC::FT>()
			);
			if (plane.orthogonal_vector() * average_normal < 0)
				plane = plane.opposite();

			point_region->plane = plane;
		}
	}

	PointQEM::~PointQEM()
	{
		for (Point_region* point_region : point_regions) {
			free(point_region);
		}
	}

	double fitting_cost(const IC::PWN_vector& points, size_t vd, const IC::Plane_3& plane) {
		const auto& param = Config::Detection::get();
		auto center = points[vd].first;
		auto normal = points[vd].second;
		double cost = 0;
		
		//normal��distance������
		double angle_threshold = std::cos(CGAL::to_double((param.max_accepted_angle * static_cast<IC::FT>(CGAL_PI)) / IC::FT(180)));
		double distance_threshold = param.max_distance_to_plane * param.max_distance_to_plane;

		if ((CGAL::abs(normal * plane.orthogonal_vector()) < angle_threshold) || (dist_cost(center, plane) > distance_threshold)) {
			cost = 1e100;
		}
		else {
			cost = param.qem_a1 * dist_cost(center, plane) + param.qem_a2 * normal_cost(normal, plane);
		}
		return cost;
	}

	void PointQEM::refine()
	{
		const auto& params = Config::Detection::get();
		const std::size_t k = params.neigbor_K;
		Neighbor_query neighbor_query(
			points,
			k,
			IC::Point_map());

		//multi-sources region growing
		std::vector<bool> visited(points.size(), false);
		struct RG_Event {
			size_t vd;
			Point_region* region;
			double cost;
		};
		auto cmp = [](const RG_Event& left, const RG_Event& right) {
			return left.cost > right.cost;
		};
		std::priority_queue<RG_Event, std::vector<RG_Event>, decltype(cmp)> RG_queue(cmp);

		// init queue
		for (Point_region* point_region : point_regions) {
			size_t seed = point_region->vds[0];
			double seed_cost = 1e100;
			auto plane = point_region->plane;
			for (auto vd : point_region->vds) {
				auto cost = fitting_cost(points, vd, plane);
				if (cost < seed_cost) {
					seed = vd;
					seed_cost = cost;
				}
			}
			//std::cout << "seed" << seed << std::endl;
			visited[seed] = true;

			std::vector<std::size_t> neighbors;
			neighbor_query(seed, neighbors);

			for (auto vd : neighbors) {
				
				auto cost = fitting_cost(points, vd, plane);
				RG_queue.push(RG_Event{ vd,point_region,cost });
			}
			auto color = point_region->color;
			*point_region = Point_region{}; //clear
			point_region->vds.push_back(seed);
			point_region->plane = plane;//keep plane for region growing
			point_region->color = color;
		}

		//growing
		while (!RG_queue.empty()) {
			auto event = RG_queue.top();
			RG_queue.pop();
			if (visited[event.vd])
				continue;

			//normal��distance������
			if (event.cost == 1e100) {
				visited[event.vd] = true;
				continue;
			}
			
			visited[event.vd] = true;
			event.region->vds.push_back(event.vd);

			std::vector<std::size_t> neighbors;
			neighbor_query(event.vd, neighbors);

			for (auto vd : neighbors) {
				if (visited[vd])
					continue;
				auto cost = fitting_cost(points, vd, event.region->plane);
				RG_queue.push(RG_Event{ vd, event.region, cost });
			}
		}

		init();

	}

	std::unique_ptr<GL::Points> PointQEM::get_points()
	{
		// assert is triangle mesh
		std::vector<GL::Vert> verts;
		for (Point_region* point_region : point_regions) {
			auto color = point_region->color;
			for (auto& vd : point_region->vds)
			{
				verts.push_back({ points[vd].first,color });
			}
		}
		return std::make_unique<GL::Points>(std::move(verts));
	}

	std::vector<EC::Detected_shape> PointQEM::detected_shape()
	{
		std::vector<std::pair<std::vector<size_t>, GL::Vec3> > regions;
		for (Point_region* point_region : point_regions) {
			std::vector<size_t> region;
			for (auto& vd : point_region->vds)
			{
				region.push_back(vd);
			}
			regions.push_back(std::make_pair(region, point_region->color));
		}
		const auto& params2 = Config::Detection::get();
		if (params2.save_result) {
			save_regions_point(points, regions, "MRG");
		}

		if(Config::Detection::get().use_regularization){
			std::vector<int> point_plane_index_map(points.size(), -1);
			std::vector<IC::Plane_3> planes{};
			for (Point_region* point_region : point_regions) {
				for (auto& vd : point_region->vds){
					point_plane_index_map[vd] = planes.size();
				}
				planes.push_back(point_region->plane);
			}

			const auto& params = Config::Regularization::get();
			CGAL::regularize_planes(points,
				IC::Point_map(),
				planes,
				CGAL::Identity_property_map<IC::Plane_3>(),
				CGAL::make_property_map(point_plane_index_map),
				params.parallelism,  // regularize parallelism
				params.orthogonality,  // regularize orthogonality
				params.coplanarity, // regularize coplanarity
				params.Z_symmetry,  // regularize Z-symmetry 
				params.paral_degree,  // 1 degrees of tolerance for parallelism / orthogonality
				params.coplane_dist // tolerance coplanarity
			);

			int ind = 0;
			for (Point_region* point_region : point_regions) {
				point_region->plane = planes[ind++];
			}
		}

		IK_to_EK to_EK;
		std::vector<EC::Detected_shape> detected_shapes;
		for (Point_region* point_region : point_regions) {
			auto plane = point_region->plane;
			std::vector<EC::PWN> pwns;
			IC::Vector_3 average_normal = CGAL::NULL_VECTOR;

			for (auto& vd : point_region->vds)
			{
				auto center = points[vd].first;
				auto normal = points[vd].second;
				average_normal += normal;
				pwns.push_back({ to_EK(center), to_EK(normal) });
			}
			average_normal /= point_region->vds.size();
			if (plane.orthogonal_vector() * average_normal < 0) {
				plane = plane.opposite();
			}

			detected_shapes.push_back({ to_EK(plane) , pwns });
		}
		return detected_shapes;
	}

}


