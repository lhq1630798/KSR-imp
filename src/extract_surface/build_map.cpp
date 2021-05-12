#include"build_map.h"

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

////n vertices and area of this face
//Dart_handle make_polygon(CMap_3& amap, const KPolygon_2& polygon, Plane_3 plane)
//{
//	//����εĶ���
//	auto id_points2 = polygon.id_polygon_2();
//
//	std::vector<vertex_attributes> id_points;
//	//Points_3 points;
//	for (int i = 0; i < id_points2.size(); i++) {
//		vertex_attributes v_att = { plane.to_3d(id_points2[i].pos_2), id_points2[i].ID };
//		id_points.push_back(v_att);
//	}
//	IC::PWN_vector inline_points = polygon.inline_points;//���ڶ�����ڲ��������
//	FT area = polygon.area();
//	Direction_3 normal = cross_product((id_points[1].pos_3 - id_points[0].pos_3), (id_points[2].pos_3 - id_points[1].pos_3)).direction();
//	//bool removed = false;
//	face_attributes face_att = { area,plane,inline_points };
//
//	std::vector<Dart_handle> darts1;
//	//beta1:first half-face
//	for (int i = 0; i < id_points.size(); i++)
//	{
//		Dart_handle d = amap.create_dart();
//		darts1.push_back(d);
//		if (amap.attribute<0>(d) == NULL)amap.set_attribute<0>(d, amap.create_attribute<0>());
//		if (amap.attribute<2>(d) == NULL)amap.set_attribute<2>(d, amap.create_attribute<2>());
//		if (amap.attribute<3>(d) == NULL)amap.set_attribute<3>(d, amap.create_attribute<3>());
//		amap.info<0>(d) = id_points[i];
//		amap.info(d) = std::make_pair(normal, false);
//	}
//	for (int i = 0; i < id_points.size() - 1; i++) {
//		amap.link_beta<1>(darts1[i], darts1[i + 1]);
//	}
//	amap.link_beta<1>(darts1[id_points.size() - 1], darts1[0]);
//	amap.info<2>(darts1[0]) = face_att;
//
//	return darts1[0];
//}
//
//Dart_handle make_twins_polygon(CMap_3& amap, const KPolygon_2& polygon, Plane_3 plane, bool is_ghost)
//{
//	//����εĶ���
//	auto id_points2 = polygon.id_polygon_2();
//
//	std::vector<vertex_attributes> id_points;
//	//Points_3 points;
//	for (int i = 0; i < id_points2.size(); i++) {
//		vertex_attributes v_att = { plane.to_3d(id_points2[i].pos_2), id_points2[i].ID };
//		id_points.push_back(v_att);
//	}
//	IC::PWN_vector inline_points = polygon.inline_points;//���ڶ�����ڲ��������
//	FT area = polygon.area();
//	Direction_3 normal = cross_product((id_points[1].pos_3 - id_points[2].pos_3), (id_points[0].pos_3 - id_points[1].pos_3)).direction();
//	//bool removed = false;
//	face_attributes face_att = { area,plane,inline_points };
//
//	//beta1:second half-face
//	std::vector<Dart_handle> darts2;
//	for (int i = 0; i < id_points.size(); i++)
//	{
//		Dart_handle d = amap.create_dart();
//		darts2.push_back(d);
//		if (amap.attribute<0>(d) == NULL)amap.set_attribute<0>(d, amap.create_attribute<0>());
//		if (amap.attribute<2>(d) == NULL)amap.set_attribute<2>(d, amap.create_attribute<2>());
//		if (amap.attribute<3>(d) == NULL)amap.set_attribute<3>(d, amap.create_attribute<3>());
//		amap.info<0>(d) = id_points[id_points.size() - 1 - i];
//		amap.info(d) = std::make_pair(normal, is_ghost);
//	}
//	for (int i = 0; i < id_points.size() - 1; i++) {
//		amap.link_beta<1>(darts2[i], darts2[i + 1]);
//	}
//	amap.link_beta<1>(darts2[id_points.size() - 1], darts2[0]);
//	amap.info<2>(darts2[0]) = face_att;
//
//	return darts2[0];
//}
//
//
//void build_edge_map(CMap_3& cm, Dart_handle it, Edge2darts& darts_per_edge, vertex_attributes p1, vertex_attributes p2) {
//
//	EdgeKey edge1(std::make_pair(p1.ID, p2.ID));
//	Edge2darts::iterator ite = darts_per_edge.find(edge1);
//	if (ite != darts_per_edge.end()) {
//		Plane_3 plane = Plane_3(p1.pos_3, (p2.pos_3 - p1.pos_3).direction());
//		//Point_3 normal = CGAL::ORIGIN + cm.info_of_attribute<2>(cm.attribute<2>(it)).normal.vector();
//		Point_3 normal = p1.pos_3 + cm.info(it).first.vector();
//		Direction_2 direction2 = (plane.to_2d(normal) - plane.to_2d(p1.pos_3)).direction();
//		//std::cout << plane << " normal"<<normal <<" direction"<< direction2 << std::endl;
//
//		Darts darts = ite->second;
//		int n = darts.size();
//		int i;
//		if (n < 2) {
//			darts.push_back(std::make_pair(direction2, it));
//			darts_per_edge[ite->first] = darts;
//		}
//		else {
//			for (i = 0; i < n - 1; i++) {
//				if (direction2.counterclockwise_in_between(darts[i].first, darts[i + 1].first)) {
//					darts.insert(darts.begin() + i + 1, std::make_pair(direction2, it));
//					darts_per_edge[ite->first] = darts;
//					break;
//				}
//			}
//			if (i == n - 1) {
//				darts.push_back(std::make_pair(direction2, it));
//				darts_per_edge[ite->first] = darts;
//			}
//		}
//	}
//	else {
//		Plane_3 plane = Plane_3(p1.pos_3, (p2.pos_3 - p1.pos_3).direction());
//		//Point_3 normal = CGAL::ORIGIN + cm.info_of_attribute<2>(cm.attribute<2>(it)).normal.vector();
//		Point_3 normal = p1.pos_3 + cm.info(it).first.vector();
//		Direction_2 direction2 = (plane.to_2d(normal) - plane.to_2d(p1.pos_3)).direction();
//		//std::cout << plane << " normal" << normal << " direction" << direction2 << std::endl;
//
//		Darts darts;
//		darts.push_back(std::make_pair(direction2, it));
//		darts_per_edge[edge1] = darts;
//	}
//}
//
//void build_map(CMap_3& cm, const KPolygons_SET& polygons_set)
//{
//	size_type amark;
//	try
//	{
//		amark = cm.get_new_mark();
//	}
//	catch (CMap_3::Exception_no_more_available_mark)
//	{
//		std::cerr << "No more free mark, exit." << std::endl;
//		exit(-1);
//	}
//
//	//int polynum = 0;
//	for (auto polygons = polygons_set._kpolygons_set.begin(); polygons != polygons_set._kpolygons_set.end(); polygons++) {
//		Plane_3 plane = polygons->plane();
//		//std::cout << plane << std::endl;
//		bool is_ghost = polygons->is_bbox;
//		for (auto &kpoly : polygons->_kpolygons_2) {
//			Dart_handle d1 = make_polygon(cm, kpoly, plane);
//			Dart_handle d2 = make_twins_polygon(cm, kpoly, plane, is_ghost);
//			//polynum++;
//			cm.sew<3>(d1, d2);
//			//cm.display_characteristics(std::cout) << ",valid=" << cm.is_valid() << std::endl;
//		}
//	}
//	//std::cout << polynum << std::endl;
//	//cm.display_characteristics(std::cout) << ",valid=" << cm.is_valid() << std::endl;
//
//	//build edge-darts table
//	Edge2darts darts_per_edge;
//	for (CMap_3::Dart_range::iterator it(cm.darts().begin()), itend(cm.darts().end()); it != itend; it++) {
//		if (cm.is_marked(it, amark))
//			continue;
//
//		auto p1 = cm.info_of_attribute<0>(cm.attribute<0>(it));
//		auto p2 = cm.info_of_attribute<0>(cm.attribute<0>(cm.beta(it, 1)));
//		//std::cout << p1.ID[0] << p1.ID[1] << p1.ID[2] << " " << p2.ID[0] << p2.ID[1] << p2.ID[2] << std::endl;
//
//		if (p1.ID < p2.ID) {
//			build_edge_map(cm, it, darts_per_edge, p1, p2);
//		}
//		else {
//			build_edge_map(cm, cm.beta(it, 3), darts_per_edge, p2, p1);
//		}
//
//		cm.mark(it, amark);
//		cm.mark(cm.beta(it, 3), amark);
//	}
//	cm.unmark_all(amark);
//
//	/*Edge2darts::iterator it = darts_per_edge.begin();
//	Edge2darts::iterator itEnd = darts_per_edge.end();
//	while (it != itEnd) {
//		std::cout << "edge:"<< it->first.edge.first[0] << it->first.edge.first[1] << it->first.edge.first[2] << " " << it->first.edge.second[0] << it->first.edge.second[1] << it->first.edge.second[2] << std::endl;
//		std::cout << "direction:" << std::endl;
//		if (it->second.size() == 1) {
//			std::cout << cm.info_of_attribute<0>(cm.attribute<0>(it->second[0].second)).pos_3 << std::endl;
//		}
//		for (int i = 0; i < it->second.size(); i++) {
//			std::cout << it->second[i].first << " " << cm.info(it->second[i].second) << std::endl;
//		}
//		std::cout<<std::endl;
//		it++;
//	}*/
//
//	//beta2
//	Edge2darts::iterator it = darts_per_edge.begin();
//	Edge2darts::iterator itEnd = darts_per_edge.end();
//	//it = darts_per_edge.begin();
//	//itEnd = darts_per_edge.end();
//	while (it != itEnd) {
//		Darts darts = it->second;
//		for (int i = 0; i < darts.size() - 1; i++) {
//			cm.sew<2>(darts[i].second, cm.beta(darts[i + 1].second, 3));
//		}
//		cm.sew<2>(darts[darts.size() - 1].second, cm.beta(darts[0].second, 3));
//		it++;
//	}
//
//	//polyhedra_attribute
//	int num = 0;
//	for (CMap_3::One_dart_per_cell_range<3>::iterator it(cm.one_dart_per_cell<3>().begin()), itend(cm.one_dart_per_cell<3>().end()); it != itend; it++) {
//
//		Point_3 center;
//		Vector_3 center_V = CGAL::NULL_VECTOR;
//		int size = 0;
//		//�ҵ�it���ڵ�polyhedra�����е�
//		for (CMap_3::One_dart_per_incident_cell_range<0, 3>::iterator it2(cm.one_dart_per_incident_cell<0, 3>(it).begin()), itend2(cm.one_dart_per_incident_cell<0, 3>(it).end()); it2 != itend2; it2++) {
//			center_V += cm.info_of_attribute<0>(cm.attribute<0>(it2)).pos_3 - CGAL::ORIGIN;
//			size++;
//		}
//
//		center_V = center_V / size;
//		center = CGAL::ORIGIN + center_V;
//		polyhedra_attributes poly_att = { num, center };
//		cm.info<3>(it) = poly_att;
//		num++;
//	}
//	std::cout << "The Combinatorial Map Characteristics:" << std::endl;
//	cm.display_characteristics(std::cout) << ",valid=" << cm.is_valid() << std::endl;
//
//}

std::vector<Dart_handle> get_C(CMap_3& cm) {
	std::vector<Dart_handle> C;
	for (CMap_3::One_dart_per_cell_range<3>::iterator it(cm.one_dart_per_cell<3>().begin()), itend(cm.one_dart_per_cell<3>().end()); it != itend; it++) {
		/*if (cm.info(it).second) {
			std::cout << "ghost num:" << cm.info<3>(it).number << std::endl;
		}*/
		C.push_back(it);
	}
	return C;
}

std::vector<Dart_handle> get_F(CMap_3& cm) {
	std::vector<Dart_handle> F;
	for (CMap_3::One_dart_per_cell_range<2>::iterator it(cm.one_dart_per_cell<2>().begin()), itend(cm.one_dart_per_cell<2>().end()); it != itend; it++) {
		F.push_back(it);
	}
	return F;
}

Neighbor get_N(CMap_3& cm, std::vector<Dart_handle> F) {
	Neighbor N;
	for (int i = 0; i < F.size(); i++) {
		int poly_number1 = cm.info_of_attribute<3>(cm.attribute<3>(F[i])).number;
		int poly_number2 = cm.info_of_attribute<3>(cm.attribute<3>(cm.beta(F[i], 3))).number;
		/*if (cm.info(F[i]).second || cm.info(cm.beta(F[i],3)).second) {
			continue;
		}*/
		if(poly_number1>poly_number2){
			int temp = poly_number2;
			poly_number2 = poly_number1;
			poly_number1 = temp;
		}
		
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
	return N;
}