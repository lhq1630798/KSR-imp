
#include "extract_surface.h"

bool operator<(const EdgeKey& e1, const EdgeKey& e2) {
	if (e1.edge.first != e2.edge.first) {
		return e1.edge.first < e2.edge.first;
	}
	else {
		return e1.edge.second < e2.edge.second;
	}
}

bool operator<(const NeighborKey& n1, const NeighborKey& n2) {
	if (n1.neighbors.first != n2.neighbors.first) {
		return n1.neighbors.first < n2.neighbors.first;
	}
	else {
		return n1.neighbors.second < n2.neighbors.second;
	}
}

//n vertices and area of this face
Dart_handle make_polygon(CMap_3& amap, KPolygon_2& polygon, Plane_3 plane,bool is_ghost)
{
	//多边形的顶点
	auto id_points2 = polygon.id_polygon_2();

	std::vector<vertex_attributes> id_points;
	//Points_3 points;
	for (int i = 0; i < id_points2.size(); i++) {
		vertex_attributes v_att = { plane.to_3d(id_points2[i].pos_2), id_points2[i].ID };
		id_points.push_back(v_att);
	}
	PWN_E inline_points = polygon.inline_points;//属于多边形内部的输入点
	FT area = polygon.area();
	Direction_3 normal = cross_product((id_points[1].pos_3 - id_points[0].pos_3), (id_points[2].pos_3 - id_points[1].pos_3)).direction();
	//bool removed = false;
	face_attributes face_att = { area,plane,inline_points };

	std::vector<Dart_handle> darts1;
	//beta1:first half-face
	for (int i = 0; i < id_points.size(); i++)
	{
		Dart_handle d = amap.create_dart();
		darts1.push_back(d);
		if (amap.attribute<0>(d) == NULL)amap.set_attribute<0>(d, amap.create_attribute<0>());
		if (amap.attribute<2>(d) == NULL)amap.set_attribute<2>(d, amap.create_attribute<2>());
		if (amap.attribute<3>(d) == NULL)amap.set_attribute<3>(d, amap.create_attribute<3>());
		amap.info<0>(d) = id_points[i];
		amap.info(d) = std::make_pair(normal,is_ghost);
	}
	for (int i = 0; i < id_points.size() - 1; i++) {
		amap.link_beta<1>(darts1[i], darts1[i + 1]);
	}
	amap.link_beta<1>(darts1[id_points.size() - 1], darts1[0]);
	amap.info<2>(darts1[0]) = face_att;

	return darts1[0];
}

Dart_handle make_twins_polygon(CMap_3& amap, KPolygon_2& polygon, Plane_3 plane, bool is_ghost)
{
	//多边形的顶点
	auto id_points2 = polygon.id_polygon_2();

	std::vector<vertex_attributes> id_points;
	//Points_3 points;
	for (int i = 0; i < id_points2.size(); i++) {
		vertex_attributes v_att = { plane.to_3d(id_points2[i].pos_2), id_points2[i].ID };
		id_points.push_back(v_att);
	}
	PWN_E inline_points = polygon.inline_points;//属于多边形内部的输入点
	FT area = polygon.area();
	Direction_3 normal = cross_product((id_points[1].pos_3 - id_points[2].pos_3), (id_points[0].pos_3 - id_points[1].pos_3)).direction();
	//bool removed = false;
	face_attributes face_att = { area,plane,inline_points };

	//beta1:second half-face
	std::vector<Dart_handle> darts2;
	for (int i = 0; i < id_points.size(); i++)
	{
		Dart_handle d = amap.create_dart();
		darts2.push_back(d);
		if (amap.attribute<0>(d) == NULL)amap.set_attribute<0>(d, amap.create_attribute<0>());
		if (amap.attribute<2>(d) == NULL)amap.set_attribute<2>(d, amap.create_attribute<2>());
		if (amap.attribute<3>(d) == NULL)amap.set_attribute<3>(d, amap.create_attribute<3>());
		amap.info<0>(d) = id_points[id_points.size() - 1 - i];
		amap.info(d) = std::make_pair(normal, is_ghost);
	}
	for (int i = 0; i < id_points.size()-1; i++) {
		amap.link_beta<1>(darts2[i], darts2[i + 1]);
	}
	amap.link_beta<1>(darts2[id_points.size() - 1], darts2[0]);
	amap.info<2>(darts2[0]) = face_att;

	return darts2[0];
}

//Dart_handle make_boundary_polygon(CMap_3& amap, KPolygon_2& polygon, Plane_3 plane)
//{
//	//std::cout << plane << std::endl;
//	//多边形的顶点
//	Points_2 points2 = polygon.polygon_2().container();
//	Points_3 points;
//	for (int i = 0; i < points2.size(); i++) {
//		points.push_back(plane.to_3d(points2[i]));
//		//std::cout << plane.to_3d(points2[i])<< std::endl;
//	}
//	PWN_E inline_points = polygon.inline_points;//属于多边形内部的输入点
//	FT area = polygon.area();
//	Direction normal = cross_product((points[1] - points[0]), (points[2] - points[1])).direction();
//	face_attributes face_att = { area,plane,inline_points,normal };
//
//	std::vector<Dart_handle> darts1;
//	//beta1:first half-face
//	for (int i = 0; i < points.size(); i++)
//	{
//		Dart_handle d = amap.create_dart();
//		darts1.push_back(d);
//		if (amap.attribute<0>(d) == NULL)amap.set_attribute<0>(d, amap.create_attribute<0>());
//		if (amap.attribute<2>(d) == NULL)amap.set_attribute<2>(d, amap.create_attribute<2>());
//		if (amap.attribute<3>(d) == NULL)amap.set_attribute<3>(d, amap.create_attribute<3>());
//		amap.info<0>(d) = points[i];
//		//amap.set_attribute<0>(d, amap.create_attribute<0>(points[i]));
//		//amap.set_attribute<2>(d, amap.create_attribute<2>(face_att));
//	}
//	for (int i = 0; i < points.size() - 1; i++) {
//		amap.link_beta<1>(darts1[i], darts1[i + 1]);
//	}
//	amap.link_beta<1>(darts1[points.size() - 1], darts1[0]);
//	amap.info<2>(darts1[0]) = face_att;
//
//	return darts1[0];
//}


void build_edge_map(CMap_3& cm, Dart_handle it, Edge2darts& darts_per_edge, vertex_attributes p1, vertex_attributes p2) {
	
	EdgeKey edge1(std::make_pair(p1.ID, p2.ID));
	Edge2darts::iterator ite = darts_per_edge.find(edge1);
	if (ite != darts_per_edge.end()) {
		Plane_3 plane = Plane_3(p1.pos_3, (p2.pos_3 - p1.pos_3).direction());
		//Point_3 normal = CGAL::ORIGIN + cm.info_of_attribute<2>(cm.attribute<2>(it)).normal.vector();
		Point_3 normal = p1.pos_3 + cm.info(it).first.vector();
		Direction_2 direction2 = (plane.to_2d(normal) - plane.to_2d(p1.pos_3)).direction();
		//std::cout << plane << " normal"<<normal <<" direction"<< direction2 << std::endl;

		Darts darts = ite->second;
		int n = darts.size();
		int i;
		if (n < 2) {
			darts.push_back(std::make_pair(direction2, it));
			darts_per_edge[ite->first] = darts;
		}
		else {
			for (i = 0; i < n - 1; i++) {
				if (direction2.counterclockwise_in_between(darts[i].first, darts[i + 1].first)) {
					darts.insert(darts.begin() + i + 1, std::make_pair(direction2, it));
					darts_per_edge[ite->first] = darts;
					break;
				}
			}
			if (i == n - 1) {
				darts.push_back(std::make_pair(direction2, it));
				darts_per_edge[ite->first] = darts;
			}
		}
	}
	else {
		Plane_3 plane = Plane_3(p1.pos_3, (p2.pos_3 - p1.pos_3).direction());
		//Point_3 normal = CGAL::ORIGIN + cm.info_of_attribute<2>(cm.attribute<2>(it)).normal.vector();
		Point_3 normal = p1.pos_3 + cm.info(it).first.vector();
		Direction_2 direction2 = (plane.to_2d(normal) - plane.to_2d(p1.pos_3)).direction();
		//std::cout << plane << " normal" << normal << " direction" << direction2 << std::endl;

		Darts darts;
		darts.push_back(std::make_pair(direction2, it));
		darts_per_edge[edge1] = darts;
	}
}

void build_map(CMap_3& cm, KPolygons_SET& polygons_set)
{
	size_type amark;
	try
	{
		amark = cm.get_new_mark();
	}
	catch (CMap_3::Exception_no_more_available_mark)
	{
		std::cerr << "No more free mark, exit." << std::endl;
		exit(-1);
	}

	//int polynum = 0;
	for (auto polygons = polygons_set._kpolygons_set.begin(); polygons != std::prev(polygons_set._kpolygons_set.end(), 6); polygons++) {
		Plane_3 plane = polygons->plane();
		//std::cout << plane << std::endl;
		for (auto &kpoly : polygons->_kpolygons_2) {
			Dart_handle d1 = make_polygon(cm, kpoly, plane,false);
			Dart_handle d2 = make_twins_polygon(cm, kpoly, plane,false);
			cm.sew<3>(d1, d2);
		}
	}
	cm.display_characteristics(std::cout) << ",valid=" << cm.is_valid() << std::endl;
	
	for (auto polygons = std::prev(polygons_set._kpolygons_set.end(), 6); polygons != polygons_set._kpolygons_set.end(); polygons++){
		Plane_3 plane = polygons->plane();
		for (auto &kpoly : polygons->_kpolygons_2) {
			Dart_handle d1 = make_polygon(cm, kpoly, plane,false);
			Dart_handle d2 = make_twins_polygon(cm, kpoly, plane,true);
			cm.sew<3>(d1, d2);
		}
	}
	cm.display_characteristics(std::cout) << ",valid=" << cm.is_valid() << std::endl;

	/*for (auto polygons = std::prev(polygons_set._kpolygons_set.end(), 6); polygons != polygons_set._kpolygons_set.end(); polygons++) {
		Plane_3 plane = polygons->plane();
		for (auto &kpoly : polygons->_kpolygons_2) {
			make_boundary_polygon(cm, kpoly, plane);
			
		}
	}
	cm.display_characteristics(std::cout) << ",valid=" << cm.is_valid() << std::endl;*/

	//build edge-darts table
	Edge2darts darts_per_edge;
	for (CMap_3::Dart_range::iterator it(cm.darts().begin()), itend(cm.darts().end()); it != itend; it++) {
		if (cm.is_marked(it, amark))
			continue;

		auto p1 = cm.info_of_attribute<0>(cm.attribute<0>(it));
		auto p2 = cm.info_of_attribute<0>(cm.attribute<0>(cm.beta(it, 1)));
		//std::cout << p1.ID[0] << p1.ID[1] << p1.ID[2] << " " << p2.ID[0] << p2.ID[1] << p2.ID[2] << std::endl;
		
		if (p1.ID < p2.ID) {
			build_edge_map(cm, it, darts_per_edge, p1, p2);
		}
		else {
			build_edge_map(cm, cm.beta(it,3), darts_per_edge, p2, p1);
		}

		cm.mark(it,amark);
		cm.mark(cm.beta(it, 3), amark);
	}
	cm.unmark_all(amark);

	/*Edge2darts::iterator it = darts_per_edge.begin();
	Edge2darts::iterator itEnd = darts_per_edge.end();
	while (it != itEnd) {
		std::cout << "edge:"<< it->first.edge.first[0] << it->first.edge.first[1] << it->first.edge.first[2] << " " << it->first.edge.second[0] << it->first.edge.second[1] << it->first.edge.second[2] << std::endl;
		std::cout << "direction:" << std::endl;
		if (it->second.size() == 1) {
			std::cout << cm.info_of_attribute<0>(cm.attribute<0>(it->second[0].second)).pos_3 << std::endl;
		}
		for (int i = 0; i < it->second.size(); i++) {
			std::cout << it->second[i].first << " " << cm.info(it->second[i].second) << std::endl;
		}
		std::cout<<std::endl;
		it++;
	}*/

	//beta2
	Edge2darts::iterator it = darts_per_edge.begin();
	Edge2darts::iterator itEnd = darts_per_edge.end();
	//it = darts_per_edge.begin();
	//itEnd = darts_per_edge.end();
	while (it != itEnd) {
		Darts darts = it->second;
		for (int i = 0; i < darts.size()-1; i++) {
			cm.sew<2>(darts[i].second, cm.beta(darts[i+1].second, 3));
		}
		cm.sew<2>(darts[darts.size()-1].second, cm.beta(darts[0].second, 3));
		it++;
	}

	//polyhedra_attribute
	int num = 0;
	for (CMap_3::One_dart_per_cell_range<3>::iterator it(cm.one_dart_per_cell<3>().begin()), itend(cm.one_dart_per_cell<3>().end()); it != itend; it++) {
		
		Point_3 center;
		Vector_3 center_V = CGAL::NULL_VECTOR;
		int size = 0;
		//找到it所在的polyhedra的所有点
		for (CMap_3::One_dart_per_incident_cell_range<0, 3>::iterator it2(cm.one_dart_per_incident_cell<0, 3>(it).begin()), itend2(cm.one_dart_per_incident_cell<0, 3>(it).end()); it2 != itend2; it2++) {
			center_V += cm.info_of_attribute<0>(cm.attribute<0>(it2)).pos_3 - CGAL::ORIGIN;
			size++;
		}
		
		center_V = center_V / size;
		center = CGAL::ORIGIN + center_V;
		polyhedra_attributes poly_att = { num, center };
		//将属于同一3-cell的dart的polyhedra——attribute设为一致的attribute
		cm.info<3>(it) = poly_att;
		num++;
	}
	cm.display_characteristics(std::cout) << ",valid="<<cm.is_valid()<<std::endl;

	
}

//0:out  1:in
int D(CMap_3& cm, PWN_E polyhedra_points, Point_3 center,int status) {

	int sum_d = 0;
	if (status == 0) {
		for (int i = 0; i < polyhedra_points.size(); i++) {
			Vector_3 u = center - polyhedra_points[i].first;
			Vector_3 n = polyhedra_points[i].second;
			if (n*u < 0) {
				sum_d++;
			}
		}
		return sum_d;
	}
	if (status == 1) {
		for (int i = 0; i < polyhedra_points.size(); i++) {
			Vector_3 u = center - polyhedra_points[i].first;
			Vector_3 n = polyhedra_points[i].second;
			if (n*u > 0) {
				sum_d++;
			}
		}
		return sum_d;
	}
}

std::vector<Dart_handle> get_C(CMap_3& cm) {
	//C:保存每个polyhedra的一个dart
	std::vector<Dart_handle> C;
	for (CMap_3::One_dart_per_cell_range<3>::iterator it(cm.one_dart_per_cell<3>().begin()), itend(cm.one_dart_per_cell<3>().end()); it != itend; it++) {
		//if (cm.info(it).second == true) { continue; }
		C.push_back(it);
	}
	int C_num = C.size();
	std::cout << "polyhedra number" << C_num << std::endl;
	return C;
}

std::vector<Dart_handle> get_F(CMap_3& cm) {
	//F:保存每个face的一个dart
	std::vector<Dart_handle> F;
	for (CMap_3::One_dart_per_cell_range<2>::iterator it(cm.one_dart_per_cell<2>().begin()), itend(cm.one_dart_per_cell<2>().end()); it != itend; it++) {
		if (cm.info(it).second == true) {
			F.push_back(cm.beta(it, 3));
		}
		else {
			F.push_back(it);
		}
	}
	std::cout << "F number" << F.size() << std::endl;
	return F;
}

Neighbor get_N(CMap_3& cm, std::vector<Dart_handle> F) {
	//N:保存相邻的两个polyhedra的darts
	Neighbor N;
	//std::vector<std::pair<Dart_handle, Dart_handle> > N;
	for (int i = 0; i < F.size(); i++) {
		/*if (cm.info(cm.beta(F[i], 3)).second == true) {
			continue;
		}*/
		int poly_number1 = cm.info_of_attribute<3>(cm.attribute<3>(F[i])).number;
		int poly_number2 = cm.info_of_attribute<3>(cm.attribute<3>(cm.beta(F[i], 3))).number;
		if (poly_number1 < poly_number2) {
			NeighborKey key(std::make_pair(poly_number1, poly_number2));
			Neighbor::iterator ite = N.find(key);
			if (ite != N.end()) {
				NeighborDarts darts = ite->second;
				darts.push_back(std::make_pair(F[i], cm.beta(F[i], 3)));
				N[key] = darts;
			}
			else {
				NeighborDarts darts;
				darts.push_back(std::make_pair(F[i], cm.beta(F[i], 3)));
				N[key] = darts;
			}
		}
		else {
			NeighborKey key(std::make_pair(poly_number2, poly_number1));
			Neighbor::iterator ite = N.find(key);
			if (ite != N.end()) {
				NeighborDarts darts = ite->second;
				darts.push_back(std::make_pair(F[i], cm.beta(F[i], 3)));
				N[key] = darts;
			}
			else {
				NeighborDarts darts;
				darts.push_back(std::make_pair(F[i], cm.beta(F[i], 3)));
				N[key] = darts;
			}
		}
	}
	return N;
}

Polygon_Mesh get_mesh(CMap_3& cm, std::vector<Dart_handle> face_darts) {
	//-----get surface mesh---------

	Polygons_3 polys_3;

	for (int i = 0; i < face_darts.size(); i++) {
		Dart_handle d = face_darts[i];

		Plane_3 plane = cm.info_of_attribute<2>(cm.attribute<2>(d)).plane;
		Points_2 points2;
		for (CMap_3::One_dart_per_incident_cell_range<0, 2>::iterator it(cm.one_dart_per_incident_cell<0, 2>(d).begin()), itend(cm.one_dart_per_incident_cell<0, 2>(d).end()); it != itend; ++it)
		{
			Point_3 p3 = cm.info_of_attribute<0>(cm.attribute<0>(it)).pos_3;
			Point_2 p2 = plane.to_2d(p3);
			points2.push_back(p2);
		}
		Polygon_3 poly(plane, points2);
		polys_3.push_back(poly);
	}

	return Polygon_Mesh{ std::vector<Polygon_GL>(polys_3.begin(), polys_3.end()) };
}

struct cmpDirection  //自定义比较规则
{
	bool operator() (const Direction_3& d1, const Direction_3& d2) const
	{
		if (d1.dx() != d2.dx()) {
			return d1.dx() < d2.dx();
		}
		else {
			if (d1.dy() != d2.dy()) {
				return d1.dy() < d2.dy();
			}
			else {
				return d1.dz() < d2.dz();
			}
		}
	}
};

CMap_3 merge_face( KPolygons_SET& polygons_set) {
	CMap_3 cm;
	build_map(cm, polygons_set);

	//C:保存每个polyhedra的一个dart
	std::vector<Dart_handle> C = get_C(cm);
	int C_num = C.size();
	std::cout << "polyhedra number" << C_num << std::endl;

	//F:保存每个face的一个dart
	std::vector<Dart_handle> F = get_F(cm);
	std::cout << "F number" << F.size() << std::endl;

	Neighbor N = get_N(cm, F);
	int N_num = N.size();
	std::cout << "N number" << N_num << std::endl;

	Neighbor::iterator ite = N.begin();
	Neighbor::iterator iteEnd = N.end();
	while (ite != iteEnd) {
		NeighborDarts darts = ite->second;
		std::map<Direction_3,std::set<Dart_handle>, cmpDirection> comface_darts;
		for (auto pair_dart : darts) {
			Direction_3 normal = cm.info<2>(pair_dart.first).plane.orthogonal_direction();
			std::map<Direction_3, std::set<Dart_handle>, cmpDirection>::iterator itd = comface_darts.find(normal);
			if (itd != comface_darts.end()) {
				std::set<Dart_handle> darts=itd->second;
				darts.insert(pair_dart.first);
				Dart_handle d = cm.beta(pair_dart.first, 1);
				while (d != pair_dart.first) {
					darts.insert(d);
					d = cm.beta(d, 1);
				}
				comface_darts[normal] = darts;
			}
			else {
				std::set<Dart_handle> darts;
				darts.insert(pair_dart.first);
				Dart_handle d = cm.beta(pair_dart.first, 1);
				while (d != pair_dart.first) {
					darts.insert(d);
					d = cm.beta(d, 1);
				}
				comface_darts[normal] = darts;
			}	
		}
		
		std::map<Direction_3, std::set<Dart_handle>, cmpDirection>::iterator itmap=comface_darts.begin();
		while (itmap != comface_darts.end()) {
			std::vector<Dart_handle> remove_darts;
			std::set<Dart_handle> darts=itmap->second;
			std::set<Dart_handle>::iterator it = darts.begin();
			std::set<Dart_handle>::iterator it2;
			while (it != darts.end()) {
				if ((it2 = darts.find(cm.beta(*it, 2))) != darts.end())
				{
					remove_darts.push_back(*it);
					darts.erase(it2);
				}
				it++;
			}

			for (int i = 0; i < remove_darts.size(); i++) {
				cm.remove_cell<1>(remove_darts[i]);
			}
			itmap++;
		}
		ite++;
	}
	return cm;
}

Polygon_Mesh get_merged_mesh(CMap_3& cm) {
	std::vector<Dart_handle> face_darts = get_F(cm);
	Polygon_Mesh merged_mesh = get_mesh(cm,face_darts);
	return merged_mesh;
}

Polygon_Mesh extract_surface(CMap_3& cm, KPolygons_SET& polygons_set)
{

	//CMap_3 cm = merge_face(polygons_set);
	//build_map(cm, polygons_set);
	

	//C:保存每个polyhedra的一个dart
	std::vector<Dart_handle> C = get_C(cm);
	int C_num = C.size();
	std::cout << "polyhedra number" << C_num << std::endl;

	//F:保存每个face的一个dart
	std::vector<Dart_handle> F = get_F(cm);
	std::cout << "F number" << F.size() << std::endl;

	Neighbor N = get_N(cm, F);
	int N_num = N.size();
	std::cout << "N number" << N_num << std::endl;

	typedef Graph<int, int, int> GraphType;
	GraphType *g = new GraphType(/*estimated # of nodes*/ C_num, /*estimated # of edges*/ N_num);

	for (int i = 0; i < C_num; i++) {
		g->add_node();
	}

	//|I|
	int points_count = 0;
	for (auto polygons = polygons_set._kpolygons_set.begin(); polygons != polygons_set._kpolygons_set.end(); polygons++) {
		for (auto &kpoly : polygons->_kpolygons_2) {
			points_count += kpoly.inline_points.size();
		}
	}
	points_count = points_count * 2;
	std::cout << "points_count:" << points_count << std::endl;

	//A
	double sum_area = 0;
	for (auto polygons = polygons_set._kpolygons_set.begin(); polygons != std::prev(polygons_set._kpolygons_set.end(), 6); polygons++) {
		Plane_3 plane = polygons->plane();
		for (auto &kpoly : polygons->_kpolygons_2) {
			sum_area += CGAL::to_double(kpoly.area());
		}
	}
	std::cout << "sum_area:" << sum_area << std::endl;


	//{i,S,T}
	for (int i = 0; i < C_num; i++) {
		//g->add_tweights(cm.info_of_attribute<3>(cm.attribute<3>(C[i])).number, D(cm, C[i], 0) / points_count, D(cm, C[i], 1) / points_count);
		Point_3 center = cm.info_of_attribute<3>(cm.attribute<3>(C[i])).center;
		std::vector<Dart_handle> face_darts;
		PWN_E polyhedra_points;
		//找到d所在的polyhedra的所有face上的dart
		for (CMap_3::One_dart_per_incident_cell_range<2, 3>::iterator it(cm.one_dart_per_incident_cell<2, 3>(C[i]).begin()), itend(cm.one_dart_per_incident_cell<2, 3>(C[i]).end()); it != itend; it++) {
			face_darts.push_back(it);
		}
		for (auto dart : face_darts) {
			for (auto p : cm.info_of_attribute<2>(cm.attribute<2>(dart)).inline_points) {
				polyhedra_points.push_back(p);
			}
		}
		std::cout << "Polyhedra "<< i << " points num:"<<polyhedra_points.size()<<std::endl;

		int d_out = D(cm, polyhedra_points, center, 0);
		int d_in = D(cm, polyhedra_points, center, 1);
		g->add_tweights(cm.info_of_attribute<3>(cm.attribute<3>(C[i])).number, d_out * sum_area, d_in * sum_area);
		//std::cout << "node :"<<cm.info_of_attribute<3>(cm.attribute<3>(C[i])).number << " i,S:"<< d_out * sum_area << " i,T:"<< d_in * sum_area << std::endl;
	}

	//{i,j}
	double lamda = 0.5;
	Neighbor::iterator ite = N.begin();
	Neighbor::iterator iteEnd = N.end();
	while (ite != iteEnd) {
		NeighborDarts darts = ite->second;
		double area=0;
		for (auto pair_dart : darts) {
			area += CGAL::to_double(cm.info_of_attribute<2>(cm.attribute<2>(pair_dart.first)).area);
		}
		g->add_edge(ite->first.neighbors.first, ite->first.neighbors.second, lamda * area * points_count, lamda * area * points_count);
		ite++;
	}

	int flow = g->maxflow();

	printf("Flow = %d\n", flow);
	printf("Minimum cut:\n");
	for (int i = 0; i < C_num; i++) {
		if (g->what_segment(i) == GraphType::SOURCE)
			printf("node%d is in the INSIDE set\n", i);
		else
			printf("node%d is in the OUTSIDE set\n", i);
	}

	//找到source和sink之间的面的dart d
	std::vector<Dart_handle> surface_darts;
	ite = N.begin();
	iteEnd = N.end();
	while (ite != iteEnd) {
		int d1 = ite->first.neighbors.first;
		int d2 = ite->first.neighbors.second;
		if (g->what_segment(d1) != g->what_segment(d2)) {
			NeighborDarts darts = ite->second;
			for (int i = 0; i < darts.size(); i++) {
				surface_darts.push_back(darts[i].first);
			}
		}
		ite++;
	}

	Polygon_Mesh surface_mesh = get_mesh(cm, surface_darts);

	delete g;

	return surface_mesh;
}

