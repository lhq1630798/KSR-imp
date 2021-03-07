
#include "extract_surface.h"
#include "fmt/core.h"
//#include "log.h"

#include <iostream>
#include <cstdlib>
#include <map>
#include <string>
#include "face_graph.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include "build_map.h"
#include "label_polyhedron.h"

//output mesh
typedef CGAL::Exact_predicates_inexact_constructions_kernel  inexact_K;
typedef inexact_K::FT                                        in_FT;
typedef inexact_K::Point_3                                   in_Point;
typedef inexact_K::Vector_3                                  in_Vector;
typedef CGAL::Surface_mesh<in_Point>                         Surface_Mesh;
typedef Surface_Mesh::Vertex_index                           vertex_descriptor;
typedef Surface_Mesh::Face_index                             face_descriptor;
typedef CGAL::SM_Vertex_index                                Vertex_index;


struct myComp {
	bool operator()(const Plane_3& p1, const Plane_3& p2) const{
		if (to_double(p1.a()) != to_double(p2.a())) {
			return to_double(p1.a()) < to_double(p2.a());
		}
		else if (to_double(p1.b()) != to_double(p2.b())) {
			return to_double(p1.b()) < to_double(p2.b());
		}
		else if (to_double(p1.c()) != to_double(p2.c())) {
			return to_double(p1.c()) < to_double(p2.c());
		}
		else{
			return to_double(p1.d()) < to_double(p2.d());
		}
	}
};

/**************Merge functions**************/
//得到图的每条边连接的两个节点索引
std::vector<std::pair<int, int>> get_edges(std::map<int, std::vector<int>> faces) {

	std::map<std::pair<int, int>, int> edges;
	std::vector<std::pair<int, int>> graph_edges;
	auto it = faces.begin();
	auto itEnd = faces.end();
	while (it != itEnd) {
		std::vector<int> verts = it->second;
		for (int i = 0; i < verts.size() - 1; i++) {
			//auto itb = find(edges.begin(),edges.end(),std::make_pair(verts[i], verts[i + 1]));
			auto itb = edges.find(std::make_pair(verts[i + 1], verts[i]));
			if (itb != edges.end()) {
				graph_edges.push_back(std::make_pair(it->first, itb->second));
				edges.erase(itb);
			}
			/*else if (itb2 != edges.end()) {
				common_edges.push_back(*itb);
				common_edges.push_back(*itb2);
				edges.erase(itb2);

			}*/
			else {
				//graph_edges.push_back(*itb);
				edges[std::make_pair(verts[i], verts[i + 1])] = it->first;
			}
		}
		auto itb = edges.find(std::make_pair(verts[0], verts[verts.size() - 1]));
		if (itb != edges.end()) {
			graph_edges.push_back(std::make_pair(it->first, itb->second));
			edges.erase(itb);
		}
		else {
			edges[std::make_pair(verts[verts.size() - 1], verts[0])] = it->first;
		}

		it++;
	}
	return graph_edges;
}

void merge_item(std::list<std::vector<int>>& item) {
	/*for (auto it = item.begin(); it != item.end(); it++) {
		for (int j = 0; j < (*it).size(); j++) {
			std::cout << (*it)[j] << " ";
		}
		std::cout << std::endl;
	}*/
	auto it = item.begin();
	//std::vector<Vertex_index>::iterator it_erase;
	while (it != item.end()) {
		auto it2 = item.begin();
		while (it2 != item.end()) {

			if (*(it->end() - 1) == *(it2->begin())) {
				it->insert(it->end(), it2->begin() + 1, it2->end());
				it2 = item.erase(it2);
				it2 = item.begin();
				break;
			}

			it2++;
		}
		if (item.size() == 1) {
			break;
		}
		if (it2 == item.end()) {
			it++;
		}
	}
}

std::vector<std::vector<int>> get_boundary_edge(std::vector<int> unionField, std::map<int, std::vector<int>> faces, std::set<std::pair<int, int> > erased_edges) {

	std::vector<std::vector<int>> boundary_edges;
	for (int i = 0; i < unionField.size(); i++) {
		std::vector<int> verts = faces[unionField[i]];
		std::list<std::vector<int>> item;
		for (int j = 0; j < verts.size() - 1; j++) {
			auto itb = erased_edges.find(std::make_pair(verts[j], verts[j + 1]));
			auto itb2 = erased_edges.find(std::make_pair(verts[j + 1], verts[j]));

			if (itb == erased_edges.end() && itb2 == erased_edges.end()) {
				std::vector<int> temp{ verts[j], verts[j + 1] };
				item.push_back(temp);
			}
		}
		auto itb = find(erased_edges.begin(), erased_edges.end(), std::make_pair(verts[verts.size() - 1], verts[0]));
		auto itb2 = find(erased_edges.begin(), erased_edges.end(), std::make_pair(verts[0], verts[verts.size() - 1]));

		if (itb == erased_edges.end() && itb2 == erased_edges.end()) {
			std::vector<int> temp{ verts[verts.size() - 1], verts[0] };
			item.push_back(temp);
		}
		merge_item(item);
		for (auto it = item.begin(); it != item.end(); it++) {
			boundary_edges.push_back(*it);
		}
	}
	return boundary_edges;
}

void merge_face(std::vector<std::vector<int>>& boundary_edges) {

	/*for (auto it = boundary_edges.begin(); it != boundary_edges.end(); it++) {
		for (int j = 0; j < (*it).size(); j++) {
			std::cout << (*it)[j] << " ";
		}
		std::cout << std::endl;
	}*/

	auto it = boundary_edges.begin();
	//std::vector<Vertex_index>::iterator it_erase;
	while (it != boundary_edges.end()) {
		auto it2 = it + 1;
		while (it2 != boundary_edges.end()) {

			if (*(it->end() - 1) == *(it2->begin())) {
				it->insert(it->end(), it2->begin() + 1, it2->end());
				boundary_edges.erase(it2);
				break;
			}

			it2++;
		}
		if ((*it)[0] == (*it)[(*it).size() - 1]) {
			it++;
		}
	}

}

//std::optional<std::vector<Vec3>> merge(Surface_Mesh& m, std::map<int, std::vector<int>>& faces, std::map<int, in_Point> index_point) {
//	std::vector<std::pair<int, int>> edges = get_edges(faces);
//	/*for (int i = 0; i < edges.size(); i++) {
//		std::cout << edges[i].first << " " << edges[i].second << std::endl;
//	}*/
//
//	ALGraph* graph = new ALGraph();
//	graph->CreateGraph(faces.size(), edges.size(), edges);
//	//cout << graph->DFS_findUnion() << "个连通域" << endl;
//	//cout << "连通情况为：" << graph->isConnect(6, 7) << endl;
//
//	int unionSize = graph->DFS_findUnion();
//
//	std::set<std::pair<int, int> > erased_edges;
//	for (int i = 0; i < graph->erase_edges.size(); i++) {
//
//		std::vector<int> v1 = faces[graph->erase_edges[i].first];
//		std::vector<int> v2 = faces[graph->erase_edges[i].second];
//		std::sort(v1.begin(), v1.end());
//		std::sort(v2.begin(), v2.end());
//		std::vector<int> v_intersection;
//		std::set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(), std::back_inserter(v_intersection));
//		erased_edges.insert(std::make_pair(v_intersection[0], v_intersection[1]));
//	}
//
//	/*for (auto it = erased_edges.begin(); it != erased_edges.end(); it++) {
//		std::cout << it->first << " " << it->second << std::endl;
//	}*/
//
//	for (int i = 0; i < unionSize; i++) {
//		std::vector<int> unionField = graph->get_unionfield(i + 1);
//		std::vector<std::vector<int>> boundary_edges = get_boundary_edge(unionField, faces, erased_edges);
//		auto bk_boundary_edges = boundary_edges;
//		merge_face(boundary_edges);
//
//		std::list<int> vertList;
//		for (int i = 0; i < boundary_edges[0].size() - 1; i++) {
//			vertList.push_back(boundary_edges[0][i]);
//		}
//
//		if (vertList.empty()) fmt::print("empty vertlist\n");
//
//		for (auto it = vertList.begin(); it != vertList.end();) {
//			auto it2 = std::next(it);
//			if (it2 == vertList.end())
//				it2 = vertList.begin();
//			auto it3 = std::next(it2);
//			if (it3 == vertList.end())
//				it3 = vertList.begin();
//
//			if (*it != *it3) {
//				it++;
//			}
//			else {
//				vertList.erase(it2);
//				vertList.erase(it3);
//				if (it != vertList.begin()) it = std::prev(it);
//			}
//		}
//
//
//		std::vector<int> results;
//		for (auto it = vertList.begin(); it != vertList.end(); it++) {
//			//std::cout << *it << " ";
//			results.push_back(*it);
//		}
//		//std::cout << std::endl;
//		if (results.empty())
//		{
//			fmt::print("empty results list\n");
//		}
//
//		
//		////将空洞情况的共同点用不同的点索引表示
//		//std::vector<int>::iterator it, it2;
//		//for (it = results.begin(); it != results.end(); it++) {
//		//	it2 = find(results.begin(), it, *it);
//		//	if (it2 != it) {
//		//		int new_v = m.add_vertex(index_point[*it]);
//		//		*it = new_v;
//		//	}
//		//}
//
//		/*判断最后结果是否还存在ABA形式*/
//		/*for (auto it = results.begin(); it != results.end();) {
//			auto it2 = std::next(it);
//			if (it2 == results.end())
//				it2 = results.begin();
//			auto it3 = std::next(it2);
//			if (it3 == results.end())
//				it3 = results.begin();
//
//			if (*it != *it3) {
//				it++;
//			}
//			else {
//				std::cout << "ABA happens" << std::endl;
//			}
//		}*/
//
//
//		face_descriptor f = m.add_face(results);
//		//R_assert(f != Surface_Mesh::null_face());
//		if (f == Surface_Mesh::null_face()) {
//			std::cout << "face invalid" << std::endl;
//			std::vector<Vec3> end_points;
//			for (const auto &boundary : bk_boundary_edges) {
//				auto point_iter = boundary.begin();
//				point_iter++;
//				while (point_iter != boundary.end())
//				{
//					auto point = index_point[*(point_iter-1)];
//					end_points.push_back(Vec3{ point.x() ,point.y() ,point.z() });
//					point = index_point[*point_iter];
//					end_points.push_back(Vec3{ point.x() ,point.y() ,point.z() });
//					point_iter++;
//				}
//			}
//			return { end_points };
//			/*std::vector<vertex_descriptor> results2;
//			for (auto it = results.end(); it != results.begin(); it--) {
//				results2.push_back(*it);
//			}
//			f = m.add_face(results2);*/
//		}
//		std::cout << f << std::endl;
//		//m.add_face(boundary_edges[0]);
//	}
//	return {};
//}

std::vector<std::vector<std::vector<int>>> merge_without_holes(std::map<int, std::vector<int>>& faces, std::map<int, in_Point> index_point) {
	std::vector<std::pair<int, int>> edges = get_edges(faces);
	/*for (int i = 0; i < edges.size(); i++) {
		std::cout << edges[i].first << " " << edges[i].second << std::endl;
	}*/

	ALGraph* graph = new ALGraph();
	graph->CreateGraph(faces.size(), edges.size(), edges);

	int unionSize = graph->DFS_findUnion();

	std::set<std::pair<int, int> > erased_edges;
	for (int i = 0; i < edges.size(); i++) {

		std::vector<int> v1 = faces[edges[i].first];
		std::vector<int> v2 = faces[edges[i].second];
		std::sort(v1.begin(), v1.end());
		std::sort(v2.begin(), v2.end());
		std::vector<int> v_intersection;
		std::set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(), std::back_inserter(v_intersection));
		erased_edges.insert(std::make_pair(v_intersection[0], v_intersection[1]));
	}

	std::vector<std::vector<std::vector<int>>> all_results;

	for (int i = 0; i < unionSize; i++) {
		std::vector<int> unionField = graph->get_unionfield(i + 1);
		std::vector<std::vector<int>> boundary_edges = get_boundary_edge(unionField, faces, erased_edges);
		merge_face(boundary_edges);

		std::vector<std::vector<int>> union_results;

		for (int p = 0; p < boundary_edges.size(); p++) {
			std::vector<int> vertList;
			//去掉最后一个元素
			for (int i = 0; i < boundary_edges[p].size() - 1; i++) {
				vertList.push_back(boundary_edges[p][i]);
			}
			union_results.push_back(vertList);
		}
		all_results.push_back(union_results);
	}
	return all_results;
}
/********************************Merge end*****************/

std::map < Plane_3, std::vector<Dart_handle>, myComp > get_merge_face(CMap_3& cm, Neighbor N, GraphType* g) {
	//找到source和sink之间的面的dart d
	std::vector<Dart_handle> surface_darts;
	auto ite = N.begin();
	auto iteEnd = N.end();
	while (ite != iteEnd) {
		int d1 = ite->first.neighbors.first;
		int d2 = ite->first.neighbors.second;
		if (g->what_segment(d1) != g->what_segment(d2)) {
			NeighborDarts darts = ite->second;
			for (int i = 0; i < darts.size(); i++) {
				if (g->what_segment(cm.info<3>(darts[i].first).number) == GraphType::SINK) {
					surface_darts.push_back(darts[i].first);
				}
				else {
					surface_darts.push_back(cm.beta(darts[i].first, 3));
				}
			}
		}
		ite++;
	}

	//每个平面上的所有面
	std::map < Plane_3, std::vector<Dart_handle>, myComp > merge_face;
	for (int i = 0; i < surface_darts.size(); i++) {
		Plane_3 plane = cm.info<2>(surface_darts[i]).plane;
		Direction_3 p_normal = plane.orthogonal_direction();
		Direction_3 normal = cm.info(surface_darts[i]).first;
		if (normal == p_normal) {
			auto it = merge_face.find(plane);
			if (it != merge_face.end()) {
				std::vector<Dart_handle> darts = it->second;
				darts.push_back(surface_darts[i]);
				merge_face[plane] = darts;
			}
			else {
				std::vector<Dart_handle> darts;
				darts.push_back(surface_darts[i]);
				merge_face[plane] = darts;
			}
		}
		else {
			plane = plane.opposite();
			auto it = merge_face.find(plane);
			if (it != merge_face.end()) {
				std::vector<Dart_handle> darts = it->second;
				darts.push_back(surface_darts[i]);
				merge_face[plane] = darts;
			}
			else {
				std::vector<Dart_handle> darts;
				darts.push_back(surface_darts[i]);
				merge_face[plane] = darts;
			}
		}
	}

	return merge_face;
}

std::vector<std::vector<in_Point>> get_all_faces(CMap_3& cm, std::map < Plane_3, std::vector<Dart_handle>, myComp > merge_face) {
	
	auto itMerge = merge_face.begin();
	auto itMEnd = merge_face.end();

	//遍历每个平面
	//int num = 0;
	int v_index = 0;
	std::vector<std::vector<in_Point>> all_faces;
	while (itMerge != itMEnd) {

		std::vector<Dart_handle> plane_darts = itMerge->second;

		//Surface_Mesh m;

		//get faces
		std::map<in_Point, int> point_index;
		std::map<int, std::vector<int>> faces;
		for (int i = 0; i < plane_darts.size(); i++) {
			Dart_handle d = plane_darts[i];

			std::vector<int> v;

			for (CMap_3::One_dart_per_incident_cell_range<0, 2>::iterator it(cm.one_dart_per_incident_cell<0, 2>(d).begin()), itend(cm.one_dart_per_incident_cell<0, 2>(d).end()); it != itend; ++it)
			{
				Point_3 p3 = cm.info_of_attribute<0>(cm.attribute<0>(it)).pos_3;
				in_Point p = in_Point(
					CGAL::to_double(p3.x()),
					CGAL::to_double(p3.y()),
					CGAL::to_double(p3.z()));

				auto itp = point_index.find(p);
				if (itp != point_index.end()) {
					v.push_back(itp->second);
				}
				else {
					point_index[p] = v_index;
					v.push_back(v_index);
					v_index++;
				}
			}
			faces[i] = v;

		}
		std::map<int, in_Point> index_point;
		for (auto it = point_index.begin(); it != point_index.end(); it++) {
			index_point[it->second] = it->first;
		}

		std::vector<std::vector<std::vector<int>>> results = merge_without_holes(faces, index_point);
		//std::vector<std::vector<std::vector<in_Point>>> point_results;
		for (auto i : results) {
			for (auto j : i) {
				std::vector<in_Point> a_face;
				for (auto k : j) {
					in_Point v = index_point[k];
					a_face.push_back(v);
				}
				all_faces.push_back(a_face);
			}
		}
		itMerge++;
	}

	return all_faces;
}

Surface_Mesh get_surface(CMap_3& cm,GraphType* g,Neighbor N) {
	Surface_Mesh m;
	std::map < Plane_3, std::vector<Dart_handle>, myComp > merge_face = get_merge_face(cm, N, g);
	std::vector<std::vector<in_Point>> all_faces = get_all_faces(cm, merge_face);
	for (auto i : all_faces) {
		std::vector<vertex_descriptor> face_with_descriptor;
		for (auto j : i) {
			vertex_descriptor v = m.add_vertex(j);
			face_with_descriptor.push_back(v);
		}
		m.add_face(face_with_descriptor);
	}
	return m;
}

Surface_Mesh get_surface_outline(CMap_3& cm, GraphType* g, Neighbor N) {
	Surface_Mesh m;
	std::map < Plane_3, std::vector<Dart_handle>, myComp > merge_face = get_merge_face(cm, N, g);
	std::vector<std::vector<in_Point>> all_faces = get_all_faces(cm, merge_face);
	for (auto face : all_faces) {
		std::vector<vertex_descriptor> face_with_descriptor;
		vertex_descriptor v0 = m.add_vertex(face[0]);
		face_with_descriptor.push_back(v0);
		for (int j = 1; j < face.size(); j++) {
			vertex_descriptor v = m.add_vertex(face[j]);
			face_with_descriptor.push_back(v);
			m.add_edge(face_with_descriptor[j - 1], face_with_descriptor[j]);
		}
		m.add_edge(face_with_descriptor[face.size() - 1], face_with_descriptor[0]);
	}
	return m;
}

bool write_ply_pss(std::ostream& os, Surface_Mesh& sm)
{
	typedef Surface_Mesh SMesh;
	typedef typename SMesh::Vertex_index VIndex;
	typedef typename SMesh::Edge_index EIndex;

	os << "ply" << std::endl
		<< "format ascii 1.0" << std::endl
		<< "comment Generated by pss" << std::endl;

	os << "element vertex " << sm.number_of_vertices() << std::endl;
	os << "property double x" << std::endl;
	os << "property double y" << std::endl;
	os << "property double z" << std::endl;

	os << "element face " << sm.number_of_faces() << std::endl;
	os << "property list uchar int vertex_indices" << std::endl;

	os << "element edge " << sm.number_of_edges() << std::endl;
	os << "property int vertex1" << std::endl;
	os << "property int vertex2" << std::endl;

	os << "end_header" << std::endl;

	std::vector<int> reindex;
	reindex.resize(sm.num_vertices());
	int n = 0;
	for (VIndex vi : sm.vertices())
	{
		os << sm.point(vi).x() << " " << sm.point(vi).y() << " " << sm.point(vi).z() << std::endl;
		reindex[std::size_t(vi)] = n++;
	}


	for (EIndex ei : sm.edges())
	{
		int v0 = reindex[std::size_t(sm.vertex(ei, 0))];
		int v1 = reindex[std::size_t(sm.vertex(ei, 1))];
		os << v0 << " " << v1 << std::endl;
	}
	return true;
}

std::optional<std::vector<Vec3>> extract_surface(const KPolygons_SET& polygons_set, std::string filename, double lamda)
{
	CMap_3 cm;
	build_map(cm, polygons_set);

	//int ghost_num;

	//C:保存每个polyhedra的一个dart
	std::vector<Dart_handle> C = get_C(cm);
	//std::vector<Dart_handle> C = get_C_simple(cm);
	std::cout << "Polyhedron number: " << C.size() << std::endl;

	//F:保存每个face的一个dart
	std::vector<Dart_handle> F = get_F(cm);
	std::cout << "F number: " << F.size() << std::endl;

	//N:保存相邻的两个polyhedra的dart
	Neighbor N = get_N(cm, F);
	//Neighbor N = get_N_simple(cm, F,C);
	std::cout << "N number: " << N.size() << std::endl;

	//打印所有相邻的polyhedron
	/*for (auto n : N) {
		std::cout << n.first.neighbors.first << " " << n.first.neighbors.second << std::endl;
	}*/

	GraphType *g = label_polyhedron(cm,C,N,polygons_set, lamda);
	//GraphType *g = label_polyhedron_simple(cm, C, N, polygons_set);

	/*for (int i = 0; i < C.size(); i++) {
		if (g->what_segment(i) == GraphType::SOURCE) {
			
			Surface_Mesh m;
			std::vector<Dart_handle> face_darts;
			for (CMap_3::One_dart_per_incident_cell_range<2, 3>::iterator it(cm.one_dart_per_incident_cell<2, 3>(C[i]).begin()), itend(cm.one_dart_per_incident_cell<2, 3>(C[i]).end()); it != itend; it++) {
				face_darts.push_back(it);
			}
			for (auto d : face_darts) {
				std::vector<vertex_descriptor> v;

				for (CMap_3::One_dart_per_incident_cell_range<0, 2>::iterator it(cm.one_dart_per_incident_cell<0, 2>(d).begin()), itend(cm.one_dart_per_incident_cell<0, 2>(d).end()); it != itend; ++it)
				{
					Point_3 p3 = cm.info_of_attribute<0>(cm.attribute<0>(it)).pos_3;
					in_Point p = in_Point(
						CGAL::to_double(p3.x()),
						CGAL::to_double(p3.y()),
						CGAL::to_double(p3.z()));

					
					v.push_back(m.add_vertex(p));
					
				}
				m.add_face(v);
			}

			std::string file = "src/output/" + filename + "_polyhedron_"+std::to_string(i)+".ply";
			std::ofstream f(file);
			if (!CGAL::write_ply(f, m)) {
				std::cout << "write wrong" << std::endl;
			}
		}
	}*/



	/******************get surface mesh*************************/
	Surface_Mesh m=get_surface(cm, g, N);
	std::string file = "src/output/" + filename + "_surface.ply";
	std::ofstream f(file);
	if (!CGAL::write_ply(f, m)) {
		std::cout << "write wrong" << std::endl;
	}

	Surface_Mesh m_outline = get_surface_outline(cm, g, N);
	std::string file2 = "src/output/" + filename + "_surface_outline.ply";
	std::ofstream f2(file2);
	if (!write_ply_pss(f2, m_outline)) {
		std::cout << "write wrong" << std::endl;
	}
	/******************get surface mesh*************************/

	delete g;

	return {};
}


