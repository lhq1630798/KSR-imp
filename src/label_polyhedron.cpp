#include"label_polyhedron.h"

//0:out  1:in
int D(CMap_3& cm, PWN_vector polyhedra_points, Point_3 center, int status) {

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

GraphType* label_polyhedron(CMap_3& cm, std::vector<Dart_handle> C, Neighbor N, KPolygons_SET& polygons_set) {
	int C_num = C.size();
	int N_num = N.size();
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
		PWN_vector polyhedra_points;
		//找到d所在的polyhedra的所有face上的dart
		for (CMap_3::One_dart_per_incident_cell_range<2, 3>::iterator it(cm.one_dart_per_incident_cell<2, 3>(C[i]).begin()), itend(cm.one_dart_per_incident_cell<2, 3>(C[i]).end()); it != itend; it++) {
			face_darts.push_back(it);
		}
		for (auto dart : face_darts) {
			for (auto p : cm.info_of_attribute<2>(cm.attribute<2>(dart)).inline_points) {
				polyhedra_points.push_back(p);
			}
		}
		std::cout << "Polyhedra " << cm.info_of_attribute<3>(cm.attribute<3>(C[i])).number << " points num:" << polyhedra_points.size() << std::endl;

		int d_out = D(cm, polyhedra_points, center, 0);
		int d_in = D(cm, polyhedra_points, center, 1);
		/*if (cm.info_of_attribute<3>(cm.attribute<3>(C[i])).number == C_num) {
			continue;
		}*/
		
		g->add_tweights(cm.info_of_attribute<3>(cm.attribute<3>(C[i])).number, d_out * sum_area, d_in * sum_area);
		//std::cout << "node " << cm.info_of_attribute<3>(cm.attribute<3>(C[i])).number << " -->S:" << d_out * sum_area << " -->T:" << d_in * sum_area << std::endl;
	}

	//{i,j}
	double lamda = 0.5;
	Neighbor::iterator ite = N.begin();
	Neighbor::iterator iteEnd = N.end();
	while (ite != iteEnd) {
		NeighborDarts darts = ite->second;
		double area = 0;
		for (auto pair_dart : darts) {
			area += CGAL::to_double(cm.info_of_attribute<2>(cm.attribute<2>(pair_dart.first)).area);
		}
		//area = area * 2;
		g->add_edge(ite->first.neighbors.first, ite->first.neighbors.second, lamda * area * points_count, lamda * area * points_count);
		//std::cout << "node " << ite->first.neighbors.first << " -->node " << ite->first.neighbors.second << ": " << lamda * area * points_count << std::endl;
		ite++;
	}

	int flow = g->maxflow();

	printf("Flow = %d\n", flow);
	printf("Minimum cut:\n");
	for (int i = 0; i < C_num; i++) {
		if (g->what_segment(i) == GraphType::SOURCE) {
			printf("node%d is in the INSIDE set\n", i);
		}
		/*else {
			printf("node%d is in the OUTSIDE set\n", i);
		}*/
	}
	return g;

}