#include"label_polyhedron.h"
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <math.h>
//#include "detect_shape/detect_shape.h"

#include <execution>
#include <mutex>

typedef std::list<IC::Triangle_3>::iterator Iterator;
typedef CGAL::AABB_triangle_primitive<IC::K, Iterator> Primitive;
typedef CGAL::AABB_traits<IC::K, Primitive> AABB_triangle_traits;
typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;

//0:out  1:in
/****** Data term 1 polygedra_with_points *******/
int D(EC::PWN_vector polyhedra_points, EC::Point_3 center, int status) {

	int sum_d = 0;
	if (status == 0) {
		for (int i = 0; i < polyhedra_points.size(); i++) {
			auto u = center - polyhedra_points[i].first;
			auto n = polyhedra_points[i].second;
			if (n*u < 0) {
				sum_d++;
			}
		}
		return sum_d;
	}
	if (status == 1) {
		for (int i = 0; i < polyhedra_points.size(); i++) {
			auto u = center - polyhedra_points[i].first;
			auto n = polyhedra_points[i].second;
			if (n*u > 0) {
				sum_d++;
			}
		}
		return sum_d;
	}
}

/****** Data term 2 faces_with_points *******/
float Dp(std::vector<std::pair<EC::Direction_3, EC::PWN_vector> > faces_with_points, int status) {

	float sum_d = 0;
	if (status == 0) {
		for (int i = 0; i < faces_with_points.size(); i++) {
			auto u = faces_with_points[i].first.to_vector();
			auto norm = std::sqrt(CGAL::to_double(u * u));
			EC::PWN_vector points = faces_with_points[i].second;
			for (int j = 0; j < points.size(); j++) {
				auto n = points[j].second;
				if (n*u < 0) {
					sum_d++;
					//sum_d += -CGAL::to_double(n * u) / norm;
				}
			}
		}
		return sum_d;
	}
	if (status == 1) {
		for (int i = 0; i < faces_with_points.size(); i++) {
			auto u = faces_with_points[i].first.to_vector();
			auto norm = std::sqrt(CGAL::to_double(u * u));
			EC::PWN_vector points = faces_with_points[i].second;
			for (int j = 0; j < points.size(); j++) {
				auto n = points[j].second;
				if (n*u > 0) {
					sum_d++;
					//sum_d += CGAL::to_double(n * u) / norm;
				}
			}
		}
		return sum_d;
	}
}

/****** Data term 3 ray_intersection *******/
float Dray(std::vector <IC::Point_3> centers, Tree& tree, std::vector<IC::Vector_3>& rays, int status) {
	float c = 0;
	float r = 0;
	int valid_ray_count = 0;
	//Tree tree(triangles.begin(), triangles.end());
	for (int i = 0; i < rays.size(); i++) {
		for (auto &center : centers) {
			IC::Ray_3 ray_query(center, center + rays[i]);
			//int num_of_intersection = tree.number_of_intersected_primitives(ray_query);

			if (auto res = tree.first_intersected_primitive(ray_query)) {
				valid_ray_count++;
				Iterator triangle = *res;

				if (triangle->supporting_plane().has_on_negative_side(center)) {
					if (status == 0) r++;
				}
				else {
					if (status == 1) r++;
				}
			}
			else if (rays[i].z() > 0) {
				//due to ground missing, if ray point to ground, we dont count it effective
				valid_ray_count++;
				if (status == 1) r++;
			}
		}
	}
	r = r / valid_ray_count;
	r = ((2 * r - 1)*(2 * r - 1)*(2 * r - 1) + 1) / 2;
	return r;

}

/*********** Todo: Data term 4 ************/


//rays direction
std::vector<IC::Vector_3> get_rays() {
	std::vector<IC::Vector_3> rays;
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
		rays.push_back(IC::Vector_3(x, y, z));
	}
	return rays;
}

// bool inside_polyhedron(CMap_3& cm, Dart_handle dh, IC::Point_3 p) {
// 	EK_to_IK to_inexact;
// 	auto frange = cm.one_dart_per_incident_cell<2, 3>(dh);

// 	for (auto fdh = frange.begin(); fdh != frange.end(); fdh++) {
// 		auto pdhs = BSP::collect(cm.darts_of_cell<2, 2>(fdh));
// 		auto plane = IC::Plane_3{
// 			to_inexact(cm.point(pdhs[2])),
// 			to_inexact(cm.point(pdhs[1])),
// 			to_inexact(cm.point(pdhs[0]))
// 		};
// 		if (plane.has_on_positive_side(to_inexact(cm.info<3>(dh).center)))
// 			plane = plane.opposite();
// 		if (plane.has_on_positive_side(p))
// 			return false;
// 	}
// 	return true;
// }

GraphType* label_polyhedron(CMap_3& cm, ExtractSurface_Params& ES_params) {

	auto C = BSP::collect(cm.one_dart_per_cell<3>());
	auto F = BSP::collect(cm.one_dart_per_cell<2>());
	auto N = get_N(cm, F);
	int C_num = C.size();
	int N_num = N.size();
	GraphType *g = new GraphType(/*estimated # of nodes*/ C_num, /*estimated # of edges*/ N_num);

	for (int i = 0; i < C_num; i++) {
		g->add_node();
	}

	//|I|
	int points_count = 0;
	for (auto fdh : F) {
		points_count += cm.info<2>(fdh).inline_points.size();
	}
	points_count = points_count * 2;
	std::cout << "points_count:" << points_count << std::endl;

	//A
	double sum_area = 0;
	for (auto fdh : F) {
		sum_area += CGAL::to_double(cm.info<2>(fdh).area);
	} //bbox?
	std::cout << "sum_area:" << sum_area << std::endl;


	//{i,S,T}
	EK_to_IK to_inexact;
	if (ES_params.GC_term == 3) {
		/******* Data term 3 ***********/

		auto rand = CGAL::Random(0);
		//std::list<IC::Triangle_3> triangles = ES_params.alpha_triangles;
		Tree tree(ES_params.alpha_triangles.begin(), ES_params.alpha_triangles.end());

		std::vector<IC::Vector_3> rays = get_rays();
		std::vector<int> ids;
		std::vector<std::vector<IC::Point_3>> all_centers;
		std::vector<float> d_outs(C_num), d_ins(C_num);
		for (int i = 0; i < C_num; i++)
		{
			ids.push_back(i);

			std::vector<IC::Point_3> centers;
			//std::vector<IC::Point_3> vertices;
			//auto prange = cm.one_dart_per_incident_cell<0, 3>(C[i]);
			//for (auto dh = prange.begin(); dh != prange.end(); dh++)
			//	vertices.push_back(to_inexact(cm.point(dh)));
			//auto box = CGAL::bbox_3(vertices.begin(), vertices.end());

			//int resolution = 3;
			//auto step_x = (box.xmax() - box.xmin()) / resolution;
			//auto step_y = (box.ymax() - box.ymin()) / resolution;
			//auto step_z = (box.zmax() - box.zmin()) / resolution;
			//while (centers.size() < 27)
			//for (int x_ind = 0; x_ind < resolution; x_ind++)
			//	for (int y_ind = 0; y_ind < resolution; y_ind++)
			//		for (int z_ind = 0; z_ind < resolution; z_ind++) {
			//			auto x = box.xmin() + step_x * x_ind + rand.get_double(0, step_x);
			//			auto y = box.ymin() + step_y * y_ind + rand.get_double(0, step_y);
			//			auto z = box.zmin() + step_z * z_ind + rand.get_double(0, step_z);
			//			IC::Point_3 center{ x,y,z };
			//			if (inside_polyhedron(cm, C[i], center))
			//				centers.push_back(center);
			//		}
			centers.push_back(to_inexact(cm.info<3>(C[i]).center));
			all_centers.push_back(centers);
		}
		std::for_each(std::execution::par, ids.begin(), ids.end(),
			[&](int i) {
			d_outs[i] = Dray(all_centers[i], tree, rays, 0);
			d_ins[i] = Dray(all_centers[i], tree, rays, 1);
		}
		);
		for (int i = 0; i < C_num; i++)
		{
			if (cm.info<3>(C[i]).is_ghost)
				g->add_tweights(cm.info_of_attribute<3>(cm.attribute<3>(C[i])).number, 0, 999 * sum_area);
			else {
				float volume = CGAL::to_double(cm.info<3>(C[i]).volume);
				g->add_tweights(cm.info_of_attribute<3>(cm.attribute<3>(C[i])).number, d_outs[i] * volume * sum_area, d_ins[i] * volume * sum_area);
			}
		}
	}
	else {
		for (int i = 0; i < C_num; i++) {

			//Point_3 center = cm.info_of_attribute<3>(cm.attribute<3>(C[i])).center;
			auto center = cm.info_of_attribute<3>(cm.attribute<3>(C[i])).center;
			IC::Point_3 in_center = to_inexact(center);

			float d_out, d_in;

			//compute d_out, d_in

			std::vector<Dart_handle> face_darts;
			for (auto it(cm.one_dart_per_incident_cell<2, 3>(C[i]).begin()), itend(cm.one_dart_per_incident_cell<2, 3>(C[i]).end()); it != itend; it++) {
				face_darts.push_back(it);
			}
			if (ES_params.GC_term == 1) {
				/***** Data term 1 ******/
				EC::PWN_vector polyhedra_points;
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
				std::vector<std::pair<EC::Direction_3, EC::PWN_vector> > faces_with_points;
				for (auto dart : face_darts) {
					auto normal = cm.info(dart).direction;
					EC::PWN_vector face_points;
					for (auto p : cm.info_of_attribute<2>(cm.attribute<2>(dart)).inline_points) {
						face_points.push_back(p);
					}
					faces_with_points.push_back(std::make_pair(normal, face_points));

				}
				d_out = Dp(faces_with_points, 0);
				d_in = Dp(faces_with_points, 1);
			}


			//std::cout << d_out * sum_area << " " << d_in * sum_area << std::endl;
			//if (i == C_num - 1) {
			//	g->add_tweights(cm.info_of_attribute<3>(cm.attribute<3>(C[i])).number, 0, sum_area);
			//}
			//else {
			//	g->add_tweights(cm.info_of_attribute<3>(cm.attribute<3>(C[i])).number, d_out * sum_area, d_in * sum_area);
			//	//std::cout << d_out * sum_area << " " << d_in * sum_area << std::endl;
			//}

			if (cm.info<3>(C[i]).is_ghost)
				g->add_tweights(cm.info_of_attribute<3>(cm.attribute<3>(C[i])).number, 0, points_count * sum_area);
			else
				g->add_tweights(cm.info_of_attribute<3>(cm.attribute<3>(C[i])).number, d_out * sum_area, d_in * sum_area);

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