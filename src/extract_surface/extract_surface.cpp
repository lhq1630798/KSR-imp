
#include "extract_surface/extract_surface.h"
#include "extract_surface/face_graph.h"
#include "extract_surface/build_map.h"
#include "extract_surface/label_polyhedron.h"
#include "detect_shape/detect_shape.h"
#include "cgal/cgal_object.h"
#include "util/config.h"

#include <fmt/core.h>
// #include "util/log.h"

#include <iostream>
#include <cstdlib>
#include <map>
#include <string>

#include <CGAL/Polygon_mesh_processing/repair_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
namespace PMP = CGAL::Polygon_mesh_processing;

/************* Compare Plane ********************/
struct myComp {
	bool operator()(const EC::Plane_3& p1, const EC::Plane_3& p2) const{
		if (p1.a() != p2.a()) {
			return p1.a() < p2.a();
		}
		else if (p1.b() != p2.b()) {
			return p1.b() < p2.b();
		}
		else if (p1.c() != p2.c()) {
			return p1.c() < p2.c();
		}
		else{
			return p1.d() < p2.d();
		}
	}
};


/******************* Merge functions *******************/

//得到 Graph 的每条边连接的两个节点索引
std::vector<std::pair<int, int>> get_edges(std::map<int, std::vector<int>> faces) {

	std::map<std::pair<int, int>, int> edges;
	std::vector<std::pair<int, int>> graph_edges;
	auto it = faces.begin();
	auto itEnd = faces.end();
	while (it != itEnd) {
		std::vector<int> verts = it->second;
		for (int i = 0; i < verts.size() - 1; i++) {
			auto itb = edges.find(std::make_pair(verts[i + 1], verts[i]));
			if (itb != edges.end()) {
				graph_edges.push_back(std::make_pair(it->first, itb->second));
				edges.erase(itb);
			}
			else {
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

//将每个子面属于边界的部分优先连接起来
void merge_item(std::list<std::vector<int>>& item) {
	/*for (auto it = item.begin(); it != item.end(); it++) {
		for (int j = 0; j < (*it).size(); j++) {
			std::cout << (*it)[j] << " ";
		}
		std::cout << std::endl;
	}*/
	auto it = item.begin();
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

//得到属于每个面的所有子边界
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

//将每个面的子边界进行合并
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

//将含有hole的面分成正反两个重叠的面
std::vector<std::vector<std::vector<int>>> merge_without_holes(std::map<int, std::vector<int>>& faces, std::map<int, IC::Point_3> index_point) {
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
/************************ Merge end *****************/


//将属于同一平面的子面聚类，得到 Map[Plane, faces_darts]
std::map < EC::Plane_3, std::vector<Dart_handle>, myComp > get_merge_face(CMap_3& cm, Neighbor N, GraphType* g) {
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
	std::map < EC::Plane_3, std::vector<Dart_handle>, myComp > merge_face;
	for (int i = 0; i < surface_darts.size(); i++) {
		EC::Plane_3 plane = cm.info<2>(surface_darts[i]).plane;
		EC::Direction_3 p_normal = plane.orthogonal_direction();
		EC::Direction_3 normal = cm.info(surface_darts[i]).direction;
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


//得到merge后的每个面
std::vector<std::vector<IC::Point_3>> get_all_faces(CMap_3& cm, std::map < EC::Plane_3, std::vector<Dart_handle>, myComp > merge_face) {
	
	auto itMerge = merge_face.begin();
	auto itMEnd = merge_face.end();

	//遍历每个平面
	//int num = 0;
	int v_index = 0;
	std::vector<std::vector<IC::Point_3>> all_faces;
	while (itMerge != itMEnd) {

		std::vector<Dart_handle> plane_darts = itMerge->second;

		//Surface_Mesh m;

		//get faces
		std::map<IC::Point_3, int> point_index;
		std::map<int, std::vector<int>> faces;
		for (int i = 0; i < plane_darts.size(); i++) {
			Dart_handle d = plane_darts[i];

			std::vector<int> v;

			for (CMap_3::One_dart_per_incident_cell_range<0, 2>::iterator it(cm.one_dart_per_incident_cell<0, 2>(d).begin()), itend(cm.one_dart_per_incident_cell<0, 2>(d).end()); it != itend; ++it)
			{
				EC::Point_3 p3 = cm.point(it);
				IC::Point_3 p = IC::Point_3(
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
		std::map<int, IC::Point_3> index_point;
		for (auto it = point_index.begin(); it != point_index.end(); it++) {
			index_point[it->second] = it->first;
		}

		std::vector<std::vector<std::vector<int>>> results = merge_without_holes(faces, index_point);
		//std::vector<std::vector<std::vector<IC::Point_3>>> point_results;
		for (auto i : results) {
			for (auto j : i) {
				std::vector<IC::Point_3> a_face;
				for (auto k : j) {
					IC::Point_3 v = index_point[k];
					a_face.push_back(v);
				}
				all_faces.push_back(a_face);
			}
		}
		itMerge++;
	}

	return all_faces;
}

/******************** Get Surface Mesh *************************/

//得到 merge 后的 Surface_mesh（holes由两个正反面重叠表示）
IC::Surface_Mesh get_surface(CMap_3& cm,GraphType* g,Neighbor N) {
	IC::Surface_Mesh m;
	auto merge_face = get_merge_face(cm, N, g);
	std::vector<std::vector<IC::Point_3>> all_faces = get_all_faces(cm, merge_face);
	for (auto i : all_faces) {
		std::vector<IC::vertex_descriptor> face_with_descriptor;
		for (auto j : i) {
			auto v = m.add_vertex(j);
			face_with_descriptor.push_back(v);
		}
		m.add_face(face_with_descriptor);
	}
	return m;
}

//得到最复杂版本的Surface_mesh，不进行合并
IC::Surface_Mesh get_surface_without_merge(CMap_3& cm, GraphType* g, Neighbor N) {
	EK_to_IK to_inexact;
	std::vector<IC::Point_3> points;
	std::vector<std::vector<size_t>> polygons;

	for (auto fdh : BSP::collect(cm.one_dart_per_cell<2>())) {
		auto n1 = cm.info<3>(fdh).number, n2 = cm.info<3>(cm.beta<3>(fdh)).number;
		if (g->what_segment(n1) == g->what_segment(n2))
			continue;
		if (g->what_segment(n1) == GraphType::SOURCE)
			fdh = cm.beta<3>(fdh);

		std::vector<size_t> polygon;
		for (auto pdh : BSP::collect(cm.darts_of_cell<2, 2>(fdh))) {
			auto p3 = cm.point(pdh);
			polygon.push_back(points.size());
			points.push_back(to_inexact(p3));
		}
		polygons.push_back(polygon);
	}

	PMP::repair_polygon_soup(points, polygons);
	PMP::orient_polygon_soup(points, polygons);

	IC::Surface_Mesh mesh;
	PMP::polygon_soup_to_polygon_mesh(points, polygons, mesh);
	mesh.is_valid();

	return mesh;
}

bool is_crease_edge(IC::Surface_Mesh& mesh, IC::edge_descriptor e) {
	auto h1 = e.halfedge();
	auto p1 = mesh.point(mesh.source(h1));
	auto p2 = mesh.point(mesh.source(mesh.next(h1)));
	auto p3 = mesh.point(mesh.source(mesh.next(mesh.next(h1))));
	auto n1 = CGAL::cross_product(p2 - p1, p3 - p2);
	
	auto h2 = mesh.opposite(h1);
	p1 = mesh.point(mesh.source(h2));
	p2 = mesh.point(mesh.source(mesh.next(h2)));
	p3 = mesh.point(mesh.source(mesh.next(mesh.next(h2))));
	auto n2 = CGAL::cross_product(p2 - p1, p3 - p2);

	auto angle_2 = (n1 * n2) * (n1 * n2) / (n1.squared_length() * n2.squared_length());
	bool is_crease = (angle_2 < 0.999);
	return is_crease;
}

IC::Surface_Mesh triangle_surface(IC::Surface_Mesh mesh) {
	PMP::triangulate_faces(CGAL::faces(mesh), mesh);

	//simplify
	//keep crease edge
	auto constrained_edge = mesh.add_property_map<IC::edge_descriptor, bool>("e:is_constrained", false).first;
	for (auto e : mesh.edges()) {
		if(is_crease_edge(mesh, e))
			constrained_edge[e] = true;
	}
	PMP::isotropic_remeshing(
		CGAL::faces(mesh),
		1e5, //a value larger than bbox
		mesh,
		PMP::parameters::edge_is_constrained_map(constrained_edge)
	);
	return mesh;
}

IC::Surface_Mesh get_surface_outline(IC::Surface_Mesh& surface) {
	IC::Surface_Mesh outline_mesh;
	for (auto e : surface.edges()) {
		if (!is_crease_edge(surface, e))
			continue;
		auto h = e.halfedge();
		auto v0 = outline_mesh.add_vertex(surface.point(surface.source(h)));
		auto v1 = outline_mesh.add_vertex(surface.point(surface.target(h)));
		outline_mesh.add_edge(v0, v1);
	}
	return outline_mesh;
}

// TODO : outline seems wrong, fix it 
//得到 merge 后的所有面的边界
//IC::Surface_Mesh get_surface_outline(CMap_3& cm, GraphType* g, Neighbor N) {
//	IC::Surface_Mesh m;
//	auto merge_face = get_merge_face(cm, N, g);
//	std::vector<std::vector<IC::Point_3>> all_faces = get_all_faces(cm, merge_face);
//	for (auto face : all_faces) {
//		std::vector<IC::vertex_descriptor> face_with_descriptor;
//		auto v0 = m.add_vertex(face[0]);
//		face_with_descriptor.push_back(v0);
//		for (int j = 1; j < face.size(); j++) {
//			auto v = m.add_vertex(face[j]);
//			face_with_descriptor.push_back(v);
//			m.add_edge(face_with_descriptor[j - 1], face_with_descriptor[j]);
//		}
//		m.add_edge(face_with_descriptor[face.size() - 1], face_with_descriptor[0]);
//	}
//	return m;
//}


/******************** Get Surface Mesh End*************************/


/******************** Draw Surface Mesh *************************/

//传参给GUI（未合并的表面）
std::unique_ptr<GL::Polygon_Mesh> draw_surface(IC::Surface_Mesh& surface) {
	std::vector<GL::Polygon> polys_3;
	for (auto f : surface.faces())
	{
		std::vector<GL::Vec3> verts;
		auto vs = surface.vertices_around_face(surface.halfedge(f));
		for (auto v : vs) {
			auto p3 = surface.point(v);
			GL::Vec3 p = GL::Vec3{
				CGAL::to_double(p3.x()),
				CGAL::to_double(p3.y()),
				CGAL::to_double(p3.z()) };
			verts.push_back(p);
		}
		polys_3.push_back(GL::Polygon(verts));
	}
	return std::make_unique<GL::Polygon_Mesh>(polys_3);

}

//传参给GUI（合并后的面的边界）
std::unique_ptr<GL::Lines> draw_surface_outline(IC::Surface_Mesh& outline) {
	std::vector<GL::Vec3> GL_edges;
	for (auto e : outline.edges()) {
		auto h = e.halfedge();
		auto p0 = outline.point(outline.source(h));
		auto p1 = outline.point(outline.target(h));
		GL_edges.push_back(GL::Vec3{ p0.x(), p0.y(), p0.z() });
		GL_edges.push_back(GL::Vec3{ p1.x(), p1.y(), p1.z() });
	}
	return std::make_unique<GL::Lines>(GL_edges);
}

/******************** Draw Surface Mesh End*************************/



//自定义的 write_ply 函数
bool write_ply_pss(std::ostream& os, IC::Surface_Mesh& sm)
{
	typedef IC::Surface_Mesh SMesh;
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


void to_origin(IC::Surface_Mesh& mesh, ExtractSurface_Params& ES_params) {
	for (auto vd : vertices(mesh)) {
		mesh.point(vd) = IC::Point_3{
			mesh.point(vd).x() * ES_params.scale - ES_params.translate.x(),
			mesh.point(vd).y() * ES_params.scale - ES_params.translate.y(),
			mesh.point(vd).z() * ES_params.scale - ES_params.translate.z() };
	}
}


//将表面保存成ply文件格式
std::tuple<std::unique_ptr<GL::Polygon_Mesh>, std::unique_ptr<GL::Lines>, int > Extract_Surface(BSP::LCC_3& cm, ExtractSurface_Params& ES_params)
{

	std::cout << "Extract Surface Begin" << std::endl;
	//C:保存每个polyhedra的一个dart
	auto C = BSP::collect(cm.one_dart_per_cell<3>());
	std::cout << "Polyhedron number: " << C.size() << std::endl;

	//F:保存每个face的一个dart
	auto F = BSP::collect(cm.one_dart_per_cell<2>());
	std::cout << "F number: " << F.size() << std::endl;

	//N:保存相邻的两个polyhedra的dart
	Neighbor N = get_N(cm, F);
	std::cout << "N number: " << N.size() << std::endl;

	//打印所有相邻的polyhedron
	/*for (auto n : N) {
		std::cout << n.first.neighbors.first << " " << n.first.neighbors.second << std::endl;
	}*/

	GraphType *g = label_polyhedron(cm, ES_params);


	//合并后的表面
	//auto m1 = get_surface(cm, g, N);

	//未合并的表面
	auto m2 = get_surface_without_merge(cm, g, N);

	auto tri_surface = triangle_surface(m2);

	//合并后的表面边界
	auto m_outline = get_surface_outline(tri_surface);


	/****************** draw surface mesh *************************/
	std::unique_ptr<GL::Polygon_Mesh> surface = draw_surface(tri_surface);
	std::unique_ptr<GL::Lines> surface_outline = draw_surface_outline(m_outline);
	

	/****************** write surface mesh *************************/
	auto save_path = Config::read<std::string>("save_path");
	to_origin(m2, ES_params);
	std::string file2 = save_path + ES_params.filename + "_surface_without_merge.ply";
	std::ofstream f2(file2, std::ios::binary);
	if (!CGAL::write_ply(f2, m2)) {
		std::cout << "write wrong" << std::endl;
	}
	f2.close();

	to_origin(tri_surface, ES_params);
	std::string tri_file_name = save_path + ES_params.filename + "_surface_tri.ply";
	std::ofstream tri_file(tri_file_name, std::ios::binary);
	if (!CGAL::write_ply(tri_file, tri_surface)) {
		std::cout << "write wrong" << std::endl;
	}
	tri_file.close();

	to_origin(m_outline, ES_params);
	std::string file3 = save_path + ES_params.filename + "_surface_outline.ply";
	std::ofstream f3(file3, std::ios::binary);
	if (!write_ply_pss(f3, m_outline)) {
		std::cout << "write wrong" << std::endl;
	}
	f3.close();

	delete g;

	return { std::move(surface), std::move(surface_outline), tri_surface.number_of_faces() };
}


