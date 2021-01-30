
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
Dart_handle make_polygon(CMap_3& amap, KPolygon_2& polygon, Plane_3 plane)
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
		amap.info(d) = normal;
	}
	for (int i = 0; i < id_points.size() - 1; i++) {
		amap.link_beta<1>(darts1[i], darts1[i + 1]);
	}
	amap.link_beta<1>(darts1[id_points.size() - 1], darts1[0]);
	amap.info<2>(darts1[0]) = face_att;

	return darts1[0];
}

Dart_handle make_twins_polygon(CMap_3& amap, KPolygon_2& polygon, Plane_3 plane)
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
		amap.info(d) = normal;
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
		Point_3 normal = p1.pos_3 + cm.info(it).vector();
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
		Point_3 normal = p1.pos_3 + cm.info(it).vector();
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
	for (auto polygons = polygons_set._kpolygons_set.begin(); polygons != polygons_set._kpolygons_set.end(); polygons++) {
		Plane_3 plane = polygons->plane();
		//std::cout << plane << std::endl;
		for (auto &kpoly : polygons->_kpolygons_2) {
			Dart_handle d1 = make_polygon(cm, kpoly, plane);
			Dart_handle d2 = make_twins_polygon(cm, kpoly, plane);
			//polynum++;
			cm.sew<3>(d1, d2);
			//cm.display_characteristics(std::cout) << ",valid=" << cm.is_valid() << std::endl;
		}
	}
	//std::cout << polynum << std::endl;
	cm.display_characteristics(std::cout) << ",valid=" << cm.is_valid() << std::endl;
	
	//for (auto polygons = polygons_set._kpolygons_set.begin(); polygons != std::prev(polygons_set._kpolygons_set.end(), 6); polygons++){
	//	Plane_3 plane = polygons->plane();
	//	for (auto &kpoly : polygons->_kpolygons_2) {
	//		Dart_handle d1 = make_polygon(cm, kpoly, plane);
	//		Dart_handle d2 = make_twins_polygon(cm, kpoly, plane);
	//		cm.sew<3>(d1, d2);
	//		
	//		//cm.display_characteristics(std::cout) << ",valid=" << cm.is_valid() << std::endl;
	//	}
	//}
	//cm.display_characteristics(std::cout) << ",valid=" << cm.is_valid() << std::endl;

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

		//EdgeKey edge1(std::make_pair(p1.ID, p2.ID));
		//EdgeKey edge2(std::make_pair(p2.ID, p1.ID));
		//Edge2darts::iterator ite = darts_per_edge.find(edge1);
		//if (ite != darts_per_edge.end()) {
		//	Plane_3 plane = Plane_3(p1.pos_3, (p2.pos_3 - p1.pos_3).direction());
		//	//Point_3 normal = CGAL::ORIGIN + cm.info_of_attribute<2>(cm.attribute<2>(it)).normal.vector();
		//	Point_3 normal = p1.pos_3 + cm.info(it).vector();
		//	Direction_2 direction2 = (plane.to_2d(normal) - plane.to_2d(p1.pos_3)).direction();
		//	//std::cout << plane << " normal"<<normal <<" direction"<< direction2 << std::endl;
		//	
		//	Darts darts = ite->second;
		//	int n = darts.size();
		//	int i;
		//	if (n < 2) {
		//		darts.push_back(std::make_pair(direction2, it));
		//		darts_per_edge[ite->first] = darts;
		//	}
		//	else {
		//		for (i = 0; i < n-1; i++) {
		//			if (direction2.counterclockwise_in_between(darts[i].first, darts[i + 1].first)) {
		//				darts.insert(darts.begin()+i+1, std::make_pair(direction2, it));
		//				darts_per_edge[ite->first] = darts;
		//				break;
		//			}
		//		}
		//		if(i==n-1) {
		//			darts.push_back(std::make_pair(direction2, it));
		//			darts_per_edge[ite->first] = darts;
		//		}
		//	}
		//}
		//else {			
		//	Edge2darts::iterator ite2 = darts_per_edge.find(edge2);
		//	if (ite2 != darts_per_edge.end()) {
		//		Plane_3 plane = Plane_3(p2.pos_3, (p1.pos_3 - p2.pos_3).direction());
		//		//Point_3 normal = CGAL::ORIGIN + cm.info_of_attribute<2>(cm.attribute<2>(cm.beta(it,3))).normal.vector();
		//		Point_3 normal = p2.pos_3 + cm.info(cm.beta(it,3)).vector();
		//		Direction_2 direction2 = (plane.to_2d(normal) - plane.to_2d(p2.pos_3)).direction();
		//		//std::cout << plane << " normal " << normal << " direction" << direction2 << std::endl;
		//		Darts darts = ite2->second;
		//		int n = darts.size();
		//		int i;
		//		if (n < 2) {
		//			darts.push_back(std::make_pair(direction2, cm.beta(it,3)));
		//			darts_per_edge[ite2->first] = darts;
		//		}
		//		else {
		//			for (i = 0; i < n - 1; i++) {
		//				if (direction2.counterclockwise_in_between(darts[i].first, darts[i+1].first)) {
		//					darts.insert(darts.begin() + i + 1, std::make_pair(direction2, cm.beta(it, 3)));
		//					darts_per_edge[ite2->first] = darts;
		//					break;
		//				}
		//			}
		//			if (i == n-1) {
		//				darts.push_back(std::make_pair(direction2, cm.beta(it, 3)));
		//				darts_per_edge[ite2->first] = darts;
		//			}
		//		}
		//	}
		//	else {
		//		Plane_3 plane = Plane_3(p1.pos_3, (p2.pos_3 - p1.pos_3).direction());
		//		//Point_3 normal = CGAL::ORIGIN + cm.info_of_attribute<2>(cm.attribute<2>(it)).normal.vector();
		//		Point_3 normal = p1.pos_3 + cm.info(it).vector();
		//		Direction_2 direction2 = (plane.to_2d(normal) - plane.to_2d(p1.pos_3)).direction();
		//		//std::cout << plane << " normal" << normal << " direction" << direction2 << std::endl;
		//		Darts darts;
		//		darts.push_back(std::make_pair(direction2, it));
		//		darts_per_edge[edge1] = darts;
		//	}
		//}
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
	
	// it = darts_per_edge.begin();
	// itEnd = darts_per_edge.end();
	//Edge2darts::iterator iterErase;
	//while (it != itEnd) {
	//	Darts darts = it->second;
	//	if (darts.size() == 1/* && cm.is_free<3>(darts[0].second)*/) {
	//		face_attributes face_att = cm.info_of_attribute<2>(cm.attribute<2>(darts[0].second));
	//		face_att.removed = true;
	//		cm.info<2>(darts[0].second) = face_att;
	//		iterErase = it;
	//		it++;
	//		darts_per_edge.erase(it);
	//	}
	//	else {
	//		it++;
	//	}
	//}

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


	////beta2
	////for each dart
	//for (CMap_3::Dart_range::iterator it(cm.darts().begin()), itend(cm.darts().end()); it != itend; it++) {
	//	if (cm.is_marked(it, amark))
	//		continue;
	//	Plane_3 plane = cm.info_of_attribute<2>(cm.attribute<2>(it)).plane;
	//	Direction plane_normal = plane.orthogonal_direction();
	//	//std::cout << plane_normal << std::endl;
	//	Point_3 p1 = cm.info_of_attribute<0>(cm.attribute<0>(it));
	//	Point_3 p2 = cm.info_of_attribute<0>(cm.attribute<0>(cm.beta(it, 1)));
	//	Point_3 p6 = cm.info_of_attribute<0>(cm.attribute<0>(cm.beta(it, 0)));
	//	//std::cout << p1 << p2 << p6 << std::endl;
	//	
	//	Direction normal = cross_product((p1 - p6),(p2 - p1)).direction();
	//	//std::cout << (p1 - p6)<<(p2 - p1) << std::endl;
	//	//std::cout << normal << std::endl;
	//	if (plane_normal != normal){
	//		plane = plane.opposite();
	//	}
	//	CMap_3::Dart_range::iterator it2, itend2;
	//	for (it2=cm.darts().begin(), itend2=cm.darts().end(); it2 != itend2; it2++) {
	//		if (cm.is_marked(it, amark))
	//			continue;
	//		//找到it2满足it2的下下个dart的点---ax+by+cz+d>0且it2的点==it的下一个dart的点
	//		Point_3 p3 = cm.info_of_attribute<0>(cm.attribute<0>(it2));
	//		Point_3 p4 = cm.info_of_attribute<0>(cm.attribute<0>(cm.beta(it2, 1)));
	//		Point_3 p5 = cm.info_of_attribute<0>(cm.attribute<0>(cm.beta(it2, 1, 1)));
	//		
	//		if ((p1 == p4) && (p2==p3) && (p5!=p6)) {
	//			if (plane.has_on_positive_side(p5)) {
	//				cm.sew<2>(it, it2);
	//				//std::cout << p3 << p4 << p5 << std::endl;
	//				cm.mark(it, amark);
	//				cm.mark(it2, amark);
	//				break;
	//			}
	//		}
	//	}
	//	if (it2 == itend2) { it2 = cm.darts().begin(); }
	//	for (it2 = cm.darts().begin(), itend2 = cm.darts().end(); it2 != itend2; it2++) {
	//		if (cm.is_marked(it, amark))
	//			continue;
	//		//找到it2满足it2的下下个dart的点---ax+by+cz+d>0且it2的点==it的下一个dart的点
	//		Point_3 p3 = cm.info_of_attribute<0>(cm.attribute<0>(it2));
	//		Point_3 p4 = cm.info_of_attribute<0>(cm.attribute<0>(cm.beta(it2, 1)));
	//		Point_3 p5 = cm.info_of_attribute<0>(cm.attribute<0>(cm.beta(it2, 1, 1)));
	//		if ((p1 == p4) && (p2 == p3) && (p5 != p6)) {
	//			if (plane.has_on(p5)) {
	//				cm.sew<2>(it, it2);
	//				//std::cout << p3 << p4 << p5 << std::endl;
	//				cm.mark(it, amark);
	//				cm.mark(it2, amark);
	//				break;
	//			}
	//		}
	//	}
	//	
	//}
	//cm.unmark_all(amark);
	////cm.free_mark(amark);
	//cm.display_characteristics(std::cout) << ",valid=" << cm.is_valid() << std::endl;

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

	////beta3
	////for each dart
	//for (CMap_3::Dart_range::iterator it(cm.darts().begin()), itend(cm.darts().end()); it != itend; it++) {
	//	if (cm.is_marked(it, amark))
	//		continue;
	//	Point_3 p1 = cm.info_of_attribute<0>(cm.attribute<0>(it));
	//	Point_3 p2 = cm.info_of_attribute<0>(cm.attribute<0>(cm.beta(it, 1)));
	//	Point_3 p6 = cm.info_of_attribute<0>(cm.attribute<0>(cm.beta(it, 0)));
	//	for (CMap_3::Dart_range::iterator it2(cm.darts().begin()), itend2(cm.darts().end()); it2 != itend2; it2++) {
	//		//找到it2满足it2的下下个dart的点---ax+by+cz+d>0且it2的点==it的下一个dart的点
	//		Point_3 p3 = cm.info_of_attribute<0>(cm.attribute<0>(it2));
	//		Point_3 p4 = cm.info_of_attribute<0>(cm.attribute<0>(cm.beta(it2, 1)));
	//		Point_3 p5 = cm.info_of_attribute<0>(cm.attribute<0>(cm.beta(it2, 1, 1)));
	//		if ((p1 == p4) && (p2 == p3) && (p5==p6)) {
	//			cm.sew<3>(it, it2);
	//			for (CMap_3::Dart_of_cell_range<2>::iterator it3(cm.darts_of_cell<2>(it).begin()), itend3(cm.darts_of_cell<2>(it).end()); it3 != itend3; it3++) {
	//				cm.mark(it3, amark);
	//			}
	//			for (CMap_3::Dart_of_cell_range<2>::iterator it3(cm.darts_of_cell<2>(it2).begin()), itend3(cm.darts_of_cell<2>(it2).end()); it3 != itend3; it3++) {
	//				cm.mark(it3, amark);
	//			}
	//			break;
	//		}
	//	}
	//}
	////cm.unmark_all(amark);
	//cm.free_mark(amark);
	//cm.display_characteristics(std::cout) << ",valid=" << cm.is_valid() << std::endl;

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

//struct ID_Edge {
//	ID_Edge(std::array<size_t, 3> _p1, std::array<size_t, 3> _p2)
//		: p1(_p1), p2(_p2)
//	{
//		if (p1 > p2) std::swap(p1, p2);
//		assert(p1 != p2);
//	}
//	bool is_bbox() {
//
//	}
//	std::array<size_t, 3> p1;
//	std::array<size_t, 3> p2;
//	static std::set<size_t> bbox_id;
//};
//bool operator<(const ID_Edge& e1, const ID_Edge& e2) {
//	if (e1.p1 != e2.p1) return e1.p1 < e2.p1;
//	return e1.p2 < e2.p2;
//}

void extract_surface(KPolygons_SET& polygons_set)
{
	/*std::map<ID_Edge, std::vector<KPoly_Ref>> Edge_Polygon_Map;
	for (auto& kpolys_2 : polygons_set._kpolygons_set) {
		for (auto kpoly = kpolys_2._kpolygons_2.begin(); kpoly != kpolys_2._kpolygons_2.end(); kpoly++) {
			auto vert = kpoly->vert_circulator(), end = vert;
			CGAL_For_all(vert, end)
				Edge_Polygon_Map[ID_Edge{ vert->T_ID(), std::next(vert)->T_ID() }].push_back(kpoly);
		}
	}
	for (auto[key, value] : Edge_Polygon_Map) {
		if (value.size() == 1) {
			std::cout << "size=1" << std::endl;
		}
	}*/

	CMap_3 cm;
	build_map(cm, polygons_set);

	//C:保存每个polyhedra的一个dart
	std::vector<Dart_handle> C;
	for (CMap_3::One_dart_per_cell_range<3>::iterator it(cm.one_dart_per_cell<3>().begin()), itend(cm.one_dart_per_cell<3>().end()); it != itend; it++) {
		C.push_back(it);
	}
	int C_num = C.size();
	std::cout << "polyhedra number"<<C_num<<std::endl;

	//F:保存每个face的一个dart
	std::vector<Dart_handle> F;
	for (CMap_3::One_dart_per_cell_range<2>::iterator it(cm.one_dart_per_cell<2>().begin()), itend(cm.one_dart_per_cell<2>().end()); it != itend; it++) {
		F.push_back(it);
	}
	std::cout << "F number" << F.size() << std::endl;

	//N:保存相邻的两个polyhedra的dart,和两个polyhedra之间的共面总面积
	Neighbor N;
	//std::vector<std::pair<Dart_handle, Dart_handle> > N;
	for (int i = 0; i < F.size();i++) {
		int poly_number1 = cm.info_of_attribute<3>(cm.attribute<3>(F[i])).number;
		int poly_number2 = cm.info_of_attribute<3>(cm.attribute<3>(cm.beta(F[i],3))).number;
		if (poly_number1 < poly_number2) {
			NeighborKey key(std::make_pair(poly_number1, poly_number2));
			Neighbor::iterator ite = N.find(key);
			if (ite != N.end()) {
				NeighborDarts darts = ite->second;
				darts.push_back(std::make_pair(F[i], cm.beta(F[i],3)));
				N[key] = darts;
			}
			else {
				NeighborDarts darts;
				darts.push_back(std::make_pair(F[i], cm.beta(F[i],3)));
				N[key] = darts;
			}
		}
		else{
			NeighborKey key(std::make_pair(poly_number2, poly_number1));
			Neighbor::iterator ite = N.find(key);
			if (ite != N.end()) {
				NeighborDarts darts = ite->second;
				darts.push_back(std::make_pair(F[i], cm.beta(F[i],3)));
				N[key] = darts;
			}
			else {
				NeighborDarts darts;
				darts.push_back(std::make_pair(F[i], cm.beta(F[i],3)));
				N[key] = darts;
			}
		}
	}
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
	//for (int i = 0; i < N_num; i++) {
	//	double area = CGAL::to_double(cm.info_of_attribute<2>(cm.attribute<2>(N[i].first)).area);
	//	//std::cout << area << std::endl;
	//	//g->add_edge(cm.info_of_attribute<3>(cm.attribute<3>(N[i].first)).number, cm.info_of_attribute<3>(cm.attribute<3>(N[i].second)).number, lamda * area / sum_area, lamda * area / sum_area);
	//	g->add_edge(cm.info_of_attribute<3>(cm.attribute<3>(N[i].first)).number, cm.info_of_attribute<3>(cm.attribute<3>(N[i].second)).number, lamda * area * points_count, lamda * area * points_count);
	//	//std::cout << "node :" << cm.info_of_attribute<3>(cm.attribute<3>(N[i].first)).number << " node :" << cm.info_of_attribute<3>(cm.attribute<3>(N[i].second)).number << " weight:" << lamda * area * points_count << std::endl;
	//}
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
			printf("node%d is in the SOURCE set\n", i);
		else
			printf("node%d is in the SINK set\n", i);
	}


	//-----get surface mesh---------
	Surface_Mesh m;

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

	/*for (int i = 0; i < N_num; i++) {
		int d1 = cm.info_of_attribute<3>(cm.attribute<3>(N[i].first)).number;
		int d2 = cm.info_of_attribute<3>(cm.attribute<3>(N[i].second)).number;
		if (g->what_segment(d1) != g->what_segment(d2)) {
			surface_darts.push_back(N[i].first);
		}
	}*/
	for (int i = 0; i < surface_darts.size();i++) {
		Dart_handle d = surface_darts[i];

		std::vector<Vertex_index> v;

		for (CMap_3::One_dart_per_incident_cell_range<0,2>::iterator it(cm.one_dart_per_incident_cell<0,2>(d).begin()), itend(cm.one_dart_per_incident_cell<0,2>(d).end()); it != itend; ++it)
		{
			Point_3 p3 = cm.info_of_attribute<0>(cm.attribute<0>(it)).pos_3;
			in_Point p = in_Point(
				CGAL::to_double(p3.x()), 
				CGAL::to_double(p3.y()), 
				CGAL::to_double(p3.z()));
			vertex_descriptor u = m.add_vertex(p);
			v.push_back(u);
		}
		m.add_face(v);
	}
	
	//输出convex mesh
	std::ofstream f("src/output/outmesh.off");
	if (!CGAL::write_off(f, m)) {
		std::cout << "write wrong" << std::endl;
	}

	delete g;

}