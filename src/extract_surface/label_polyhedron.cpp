#include"label_polyhedron.h"
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <math.h>
//#include "detect_shape/detect_shape.h"

typedef std::list<EPIC::in_Triangle>::iterator Iterator;
typedef CGAL::AABB_triangle_primitive<EPIC::EPIC_K, Iterator> Primitive;
typedef CGAL::AABB_traits<EPIC::EPIC_K, Primitive> AABB_triangle_traits;
typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;

//0:out  1:in
/****** Data term 1 polygedra_with_points *******/
int D(PWN_vector polyhedra_points, Point_3 center, int status) {

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

/****** Data term 2 faces_with_points *******/
int Dp(std::vector<std::pair<Direction_3, PWN_vector> > faces_with_points, int status) {

	int sum_d = 0;
	if (status == 0) {
		for (int i = 0; i < faces_with_points.size(); i++) {
			Vector_3 u = faces_with_points[i].first.to_vector();
			PWN_vector points = faces_with_points[i].second;
			for (int j = 0; j < points.size(); j++) {
				Vector_3 n = points[j].second;
				if (n*u < 0) {
					sum_d++;
				}
			}	
		}
		return sum_d;
	}
	if (status == 1) {
		for (int i = 0; i < faces_with_points.size(); i++) {
			Vector_3 u = faces_with_points[i].first.to_vector();
			PWN_vector points = faces_with_points[i].second;
			for (int j = 0; j < points.size(); j++) {
				Vector_3 n = points[j].second;
				if (n*u > 0) {
					sum_d++;
				}
			}
		}
		return sum_d;
	}
}

/****** Data term 3 ray_intersection *******/
float Dray(EPIC::in_Point center, std::list<EPIC::in_Triangle>& triangles, std::vector<EPIC::in_Vector>& rays, int status) {
	float c = 0;
	float r = 0;
	Tree tree(triangles.begin(), triangles.end());
	for (int i = 0; i < rays.size(); i++) {
		EPIC::in_Ray3 ray_query(center, center + rays[i]);
		int num_of_intersection = tree.number_of_intersected_primitives(ray_query);
		if (num_of_intersection % 2 == 0) {
			r++;
		}
	}
	r = r / rays.size();
	c = ((2 * r - 1)*(2 * r - 1)*(2 * r - 1) + 1) / 2;
	if (status == 0) {
		return 1 - c;
	}
	if (status == 1) {
		return c;
	}
}

/*********** Todo: Data term 4 ************/


//rays direction
std::vector<EPIC::in_Vector> get_rays() {
	std::vector<EPIC::in_Vector> rays;
	double pi = 2 * acos(0.0);
	//std::cout << pi << std::endl;
	double gold = 3 - sqrt(5);
	//std::cout << gold << std::endl;
	int samples = 50;
	for (int i = 0; i < samples; i++) {
		double z = 1 - (double(i) / double(samples-1)) * 2;
		double theta = pi * i * gold;
		double x = cos(theta) * sqrt(1 - z * z);
		double y = sin(theta) * sqrt(1 - z * z);
		//std::cout << x <<" "<< y <<" "<< z << std::endl;
		rays.push_back(EPIC::in_Vector(x, y, z));
	}
	return rays;
}


GraphType* label_polyhedron(CMap_3& cm, std::vector<Dart_handle> C, Neighbor N, const KPolygons_SET& polygons_set, ExtractSurface_Params& ES_params) {
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
	//std::list<EPIC::in_Triangle> triangles = ES_params.alpha_triangles;
	std::vector<EPIC::in_Vector> rays = get_rays();
	for (int i = 0; i < C_num; i++) {
		
		//Point_3 center = cm.info_of_attribute<3>(cm.attribute<3>(C[i])).center;
		EPIC::EK_to_IK to_inexact;
		Point_3 center = cm.info_of_attribute<3>(cm.attribute<3>(C[i])).center;
		EPIC::in_Point in_center = to_inexact(center);
		
		float d_out, d_in;

		//compute d_out, d_in
		if (ES_params.GC_term == 1 || ES_params.GC_term == 2) {
			std::vector<Dart_handle> face_darts;
			for (CMap_3::One_dart_per_incident_cell_range<2, 3>::iterator it(cm.one_dart_per_incident_cell<2, 3>(C[i]).begin()), itend(cm.one_dart_per_incident_cell<2, 3>(C[i]).end()); it != itend; it++) {
				face_darts.push_back(it);
			}
			if (ES_params.GC_term == 1) {
				/***** Data term 1 ******/
				PWN_vector polyhedra_points;
				for (auto dart : face_darts) {
					for (auto p : cm.info_of_attribute<2>(cm.attribute<2>(dart)).inline_points) {
						polyhedra_points.push_back(p);
					}
				}
				//std::cout << "Polyhedra " << cm.info_of_attribute<3>(cm.attribute<3>(C[i])).number << " points num:" << polyhedra_points.size() << std::endl;
				d_out = D(polyhedra_points, center, 0);
				d_in = D(polyhedra_points, center, 1);
			}
			else if (ES_params.GC_term == 2) {
				/******** Data term 2 *********/
				std::vector<std::pair<Direction_3, PWN_vector> > faces_with_points;
				for (auto dart : face_darts) {
					Direction_3 normal = cm.info(dart).first;
					PWN_vector face_points;
					for (auto p : cm.info_of_attribute<2>(cm.attribute<2>(dart)).inline_points) {
						face_points.push_back(p);
					}
					faces_with_points.push_back(std::make_pair(normal, face_points));

				}
				d_out = Dp(faces_with_points,0);
				d_in = Dp(faces_with_points,1);
			}
		}
		else if (ES_params.GC_term == 3) {
			/******* Data term 3 ***********/
			d_out = Dray(in_center, ES_params.alpha_triangles, rays, 0);
			d_in = Dray(in_center, ES_params.alpha_triangles, rays, 1);
		}


		//std::cout << d_out * sum_area << " " << d_in * sum_area << std::endl;
		if (i == C_num - 1) {
			g->add_tweights(cm.info_of_attribute<3>(cm.attribute<3>(C[i])).number, 0, sum_area);
		}
		else {
			g->add_tweights(cm.info_of_attribute<3>(cm.attribute<3>(C[i])).number, d_out * sum_area, d_in * sum_area);
			//std::cout << d_out * sum_area << " " << d_in * sum_area << std::endl;
		}

	}

	//{i,j}
	Neighbor::iterator ite = N.begin();
	Neighbor::iterator iteEnd = N.end();
	while (ite != iteEnd) {
		NeighborDarts darts = ite->second;
		double area = 0;
		for (auto pair_dart : darts) {
			area += CGAL::to_double(cm.info_of_attribute<2>(cm.attribute<2>(pair_dart.first)).area);
		}
		
		if (ES_params.GC_term == 1 || ES_params.GC_term == 2) {
			g->add_edge(ite->first.neighbors.first, ite->first.neighbors.second, ES_params.lamda * area * points_count, ES_params.lamda * area * points_count);
			//std::cout << "node " << ite->first.neighbors.first << " -->node " << ite->first.neighbors.second << ": " << lamda * area * points_count << std::endl;
		}
		else if (ES_params.GC_term == 3) {
			g->add_edge(ite->first.neighbors.first, ite->first.neighbors.second, ES_params.lamda * area, ES_params.lamda * area);
			//std::cout << "node " << ite->first.neighbors.first << " -->node " << ite->first.neighbors.second << ": " << lamda * area << std::endl;
		}

		ite++;
	}

	//min-cut
	int flow = g->maxflow();

	printf("Flow = %d\n", flow);
	printf("Minimum cut:\n");
	for (int i = 0; i < C_num; i++) {
		if (g->what_segment(i) == GraphType::SOURCE) {
			printf("node%d is in the INSIDE set\n", i);
		}
		else {
			printf("node%d is in the OUTSIDE set\n", i);
		}
	}
	return g;

}