#include "BSP.h"

#include "util/log.h"
#undef assert
#define assert(expr) R_assert(expr)

//#include <CGAL/Combinatorial_map.h>
//using CMap_3 = CGAL::Combinatorial_map<3>;
//#include <CGAL/Polygonal_surface_reconstruction.h>

using namespace BSP;



// Functor called when one polyhedra is split in two.
struct Polyhedra_Split_functor
{
	Polyhedra_Split_functor(LCC_3& alcc) : lcc(alcc)
	{}
	// operator() automatically called after a split.
	void operator()(Polyhedra_attribute& ca1, Polyhedra_attribute& ca2)
	{
		auto center1 = lcc.barycenter<3>(ca1.dart());
		auto center2 = lcc.barycenter<3>(ca2.dart());

		ca1.info().center = center1;
		ca2.info().center = center2;

		//std::cout << "After on split faces: center of polyhera1=" << ca1.info().center << ", center of polyhera1=" << ca2.info().center << std::endl;
	}
private:
	LCC_3& lcc;
};

struct Face_Split_functor
{
	Face_Split_functor(LCC_3& alcc) : lcc(alcc)
	{}
	// operator() automatically called after a split.
	void operator()(Face_attributes& ca1, Face_attributes& ca2)
	{
		//split inline points
		auto inline_points = std::move(ca1.info().inline_points);
		ca2.info().inline_points.clear();

		Polygon_2 poly1{ }, poly2{ };
		auto plane = ca1.info().plane;
		for (auto pdh : collect(lcc.darts_of_cell<2, 2>(ca1.dart()))) {
			assert(plane.has_on(lcc.point(pdh)));
			poly1.insert(poly1.end(), plane.to_2d(lcc.point(pdh)));
		}
		for (auto pdh : collect(lcc.darts_of_cell<2, 2>(ca2.dart()))) {
			assert(plane.has_on(lcc.point(pdh)));
			poly2.insert(poly2.end(), plane.to_2d(lcc.point(pdh)));
		}

		for (auto& [point_3, normal] : inline_points)
		{
			auto point_2 = plane.to_2d(point_3);
			if (poly1.has_on_bounded_side(point_2))
			{
				assert(!poly2.has_on_bounded_side(point_2));
				ca1.info().inline_points.emplace_back(point_3, normal);
			}
			else if (poly2.has_on_bounded_side(point_2))
			{
				ca2.info().inline_points.emplace_back(point_3, normal);
			}
		}

	}
private:
	LCC_3& lcc;
};

BSP_Partition::BSP_Partition(Polygons_3 _polygons_3) : polygons_3(_polygons_3) {
	lcc.onsplit_functor<3>() = Polyhedra_Split_functor(lcc);
	lcc.onsplit_functor<2>() = Face_Split_functor(lcc);

	// small first
	std::sort(polygons_3.begin(), polygons_3.end(), [](Polygon_3& poly_3_a, Polygon_3& poly_3_b) {
		return poly_3_a.inline_points.size() < poly_3_b.inline_points.size();
	});
	//// large first
	//std::sort(polygons_3.begin(), polygons_3.end(), [](Polygon_3& poly_3_a, Polygon_3& poly_3_b) {
	//	return poly_3_a.inline_points.size() > poly_3_b.inline_points.size();
	//});

	//expand polygon
	for (auto& poly : polygons_3) {
		auto& polygon_2 = poly.polygon_2();
		// center
		Vector_2 center_V = CGAL::NULL_VECTOR;
		for (const auto& point_2 : polygon_2.container())
			center_V += point_2 - CGAL::ORIGIN;
		center_V = center_V / polygon_2.size();
		auto center_P = CGAL::ORIGIN + center_V;
		//expand
		for (auto& point_2 : polygon_2.container()) {
			point_2 += 0.1*(point_2 - center_P);
		}
		poly.update_points_3();
	}

	// bounding box
	/*
	*       4----7
	*      /|   /|
	*     5----6 |
	*     | 3--|-2
	*     |/   |/
	*     0----1
	*
	*     ^ y
	*     |
	*     |
	*     o---->x
	*    /
	*   /z
	*  v
	*/
	auto box = CGAL::Bbox_3{};
	for (const auto& poly_3 : polygons_3)
		box += CGAL::bbox_3(poly_3.points_3().begin(), poly_3.points_3().end());
	box.dilate(1e3);

	Point_3 p0 = { box.xmin(), box.ymin(), box.zmax() };
	Point_3 p1 = { box.xmax(), box.ymin(), box.zmax() };
	Point_3 p2 = { box.xmax(), box.ymin(), box.zmin() };
	Point_3 p3 = { box.xmin(), box.ymin(), box.zmin() };
	Point_3 p4 = { box.xmin(), box.ymax(), box.zmin() };
	Point_3 p5 = { box.xmin(), box.ymax(), box.zmax() };
	Point_3 p6 = { box.xmax(), box.ymax(), box.zmax() };
	Point_3 p7 = { box.xmax(), box.ymax(), box.zmin() };
	auto dh = lcc.make_hexahedron(p0, p1, p2, p3, p4, p5, p6, p7);

	lcc.set_attribute<3>(dh, lcc.create_attribute<3>());
	for (const auto& poly : polygons_3)
		lcc.info<3>(dh).polygons_3.push_back(poly);
	lcc.info<3>(dh).center = lcc.barycenter<3>(dh);
	for (auto fdh : collect(lcc.one_dart_per_cell<2>())) {
		lcc.set_attribute<2>(fdh, lcc.create_attribute<2>());
		auto p_range = collect(lcc.darts_of_cell<2, 2>(fdh));
		auto plane = Plane_3{ lcc.point(p_range[0]),lcc.point(p_range[1]),lcc.point(p_range[2]) };
		lcc.info<2>(fdh).plane = plane;
	}

	volumes.push_back(dh);

	auto ghost = lcc.make_combinatorial_hexahedron();
	lcc.set_attribute<3>(ghost, lcc.create_attribute<3>());
	lcc.info<3>(ghost).center = CGAL::ORIGIN;
	lcc.info<3>(ghost).is_ghost = true;
	lcc.sew<3>(dh, ghost);
	lcc.sew<3>(lcc.beta<2>(dh), lcc.beta<2>(ghost));
	lcc.sew<3>(lcc.beta<0, 2>(dh), lcc.beta<1, 2>(ghost));
	lcc.sew<3>(lcc.beta<1, 2>(dh), lcc.beta<0, 2>(ghost));
	lcc.sew<3>(lcc.beta<1, 1, 2>(dh), lcc.beta<1, 1, 2>(ghost));
	lcc.sew<3>(lcc.beta<2, 1, 1, 2>(dh), lcc.beta<2, 1, 1, 2>(ghost));
}

void BSP_Partition::partition()
{
	int count = 0;
	while (!volumes.empty()) {
		std::cout << "\rsplit volume " << count;
		count++;
		partition_next();
	}
	lcc.display_characteristics(std::cout) << ",valid=" << lcc.is_valid() << std::endl;

	// set attribute 
	for (auto fdh : collect(lcc.one_dart_per_cell<2>())) {
		Polygon_2 poly{ };
		auto plane = lcc.info<2>(fdh).plane;
		for (auto pdh : collect(lcc.darts_of_cell<2, 2>(fdh))) {
			assert(plane.has_on(lcc.point(pdh)));
			poly.insert(poly.end(), plane.to_2d(lcc.point(pdh)));
		}
		lcc.info<2>(fdh).area = CGAL::abs(poly.area());
	}
	for (auto fdh : collect(lcc.one_dart_per_cell<2>())) {
		auto pdhs = collect(lcc.darts_of_cell<2, 2>(fdh));
		auto vec01 = lcc.point(pdhs[1]) - lcc.point(pdhs[0]);
		auto vec12 = lcc.point(pdhs[2]) - lcc.point(pdhs[1]);
		auto normal = -CGAL::cross_product(vec01, vec12); //make it point to outside
		auto out_dir = lcc.point(pdhs[0]) - lcc.info<3>(fdh).center;
		if (normal * out_dir < 0) {
			assert(lcc.info<3>(fdh).is_ghost);
			// todo: ghost normal?
		}

		for (auto pdh : pdhs) {
			lcc.info(pdh).direction = -normal.direction();
		}
		for (auto pdh : collect(lcc.darts_of_cell<2, 2>(lcc.beta<3>(fdh)))) {
			lcc.info(pdh).direction = normal.direction();
		}
	}
	int num = 0;
	for (auto cdh : collect(lcc.one_dart_per_cell<3>())) {
		lcc.info<3>(cdh).number = num;
		num++;
	}

	// count attribute
	size_t inliner_num = 0;
	for (auto fdh : collect(lcc.one_dart_per_cell<2>())) {
		inliner_num += lcc.info<2>(fdh).inline_points.size();
	}
	std::cout << "inliner_num = " << inliner_num << std::endl;


}




void BSP_Partition::partition_next()
{
	if (volumes.empty())
		return;
	auto dh = volumes.back();
	volumes.pop_back();

	auto polys = std::move(lcc.info<3>(dh).polygons_3);
	if (polys.empty())
		return;
	auto largest_poly = polys.back(); // the largest polygon
	polys.pop_back();

	// collect all faces handle first in case of iterator invalidation
	auto faces = collect(lcc.one_dart_per_incident_cell<2, 3>(dh));

	LCC_3::Dart_handle new_fdh = lcc.null_dart_handle;
	for (auto fdh : faces) {
		//std::cout << "==========================" << std::endl;
		bool cut_edge = false;
		auto new_dhs = std::vector<LCC_3::Dart_handle>{};

		auto pdhs = collect(lcc.darts_of_cell<2, 2>(fdh)); //one side
		assert(pdhs.size() >= 3);
		for (auto pdh : pdhs) {
			auto point1 = lcc.point(pdh);
			auto point2 = lcc.point(lcc.next(pdh));
			//std::cout << point1.x() << " " << point1.y() << " " << point1.z() << std::endl;
			//std::cout << point2.x() << " " << point2.y() << " " << point2.z() << std::endl;
			//std::cout << std::endl;
			auto edge = Segment_3{ point1, point2 };
			if (auto result = CGAL::intersection(edge, largest_poly.plane()))
			{
				if (auto* inters_seg = boost::get<Segment_3>(&*result)) {
					//std::cout << "cutting a edge\n";
					cut_edge = true;
					new_fdh = pdh;
				}
				else {
					auto* inters_point = boost::get<Point_3>(&*result);
					// todo::comparision by ID
					if (largest_poly.plane().has_on(point1)) { //already intersection point
						new_dhs.push_back({ pdh });
						continue;
					}
					if (largest_poly.plane().has_on(point2))
						continue;
					auto new_pdh = lcc.insert_point_in_cell<1>(pdh, *inters_point);
					new_dhs.push_back({ new_pdh });
				}
			}
		}
		if (!cut_edge && !new_dhs.empty()) {
			assert(new_dhs.size() == 2);
			new_fdh = lcc.insert_cell_1_in_cell_2(new_dhs[0], new_dhs[1]);
		}

	}


	// split volume
	assert(new_fdh != lcc.null_dart_handle); //?
	std::vector< LCC_3::Dart_handle > new_face_path;
	auto next_dh = new_fdh;
	do {
		// find the next dart
		auto out_dhs = collect(lcc.darts_of_cell<0, 2>(lcc.next(next_dh)));
		for (auto dh : out_dhs) {
			if (lcc.beta<2>(dh) == next_dh) continue;
			if (largest_poly.plane().has_on(lcc.point(lcc.next(dh)))) {
				new_face_path.push_back(dh);
				break;
			}
		}
		assert(next_dh != new_face_path.back());
		next_dh = new_face_path.back();
	} while (next_dh != new_fdh);
	assert(new_face_path.size() >= 3);
	if (!lcc.is_insertable_cell_2_in_cell_3(new_face_path.begin(), new_face_path.end())) {
		log("fatal error\n");
		return;
	}
	auto new_volume = lcc.insert_cell_2_in_cell_3(new_face_path.begin(), new_face_path.end());
	lcc.set_attribute<2>(new_volume, lcc.create_attribute<2>());
	lcc.info<2>(new_volume).plane = largest_poly.plane();
	lcc.info<2>(new_volume).inline_points = std::move(largest_poly.inline_points);




	// distribute polygons
	assert(lcc.info<3>(new_volume).polygons_3.empty());
	assert(lcc.info<3>(lcc.beta<3>(new_volume)).polygons_3.empty());
	assert(!largest_poly.plane().has_on(lcc.info<3>(new_volume).center));
	bool positive_side = largest_poly.plane().has_on_positive_side(lcc.info<3>(new_volume).center);
	if (!positive_side) new_volume = lcc.beta<3>(new_volume);

	for (auto poly : polys) {

		if (auto res = poly.split_by_plane(largest_poly.plane())) {
			auto [new_poly1, new_poly2] = *res;
			assert(!largest_poly.plane().has_on(new_poly1.points_3()[0]));
			if (largest_poly.plane().has_on_positive_side(new_poly1.points_3()[0])) {
				lcc.info<3>(new_volume).polygons_3.push_back(new_poly1);
				lcc.info<3>(lcc.beta<3>(new_volume)).polygons_3.push_back(new_poly2);
			}
			else {
				lcc.info<3>(new_volume).polygons_3.push_back(new_poly2);
				lcc.info<3>(lcc.beta<3>(new_volume)).polygons_3.push_back(new_poly1);
			}
		}
		else {
			assert(!largest_poly.plane().has_on(poly.points_3()[0]));
			if (largest_poly.plane().has_on_positive_side(poly.points_3()[0])) {
				lcc.info<3>(new_volume).polygons_3.push_back(poly);
			}
			else {
				lcc.info<3>(lcc.beta<3>(new_volume)).polygons_3.push_back(poly);
			}
		}

	}

	////todo: sort!!!
	//{
	//	auto& polys = lcc.info<3>(new_volume).polygons_3;
	//	std::sort(polys.begin(), polys.end(), [](Polygon_3& poly_3_a, Polygon_3& poly_3_b) {
	//		return poly_3_a.inline_points.size() < poly_3_b.inline_points.size();
	//	});
	//}
	//{
	//	auto& polys = lcc.info<3>(lcc.beta<3>(new_volume)).polygons_3;
	//	std::sort(polys.begin(), polys.end(), [](Polygon_3& poly_3_a, Polygon_3& poly_3_b) {
	//		return poly_3_a.inline_points.size() < poly_3_b.inline_points.size();
	//	});
	//}

	if (!lcc.info<3>(new_volume).polygons_3.empty())
		volumes.push_back(new_volume);
	if (!lcc.info<3>(lcc.beta<3>(new_volume)).polygons_3.empty())
		volumes.push_back(lcc.beta<3>(new_volume));

}

std::unique_ptr<Polygon_Mesh> BSP_Partition::Get_mesh()
{
	std::vector<Polygon_GL> polys;
	auto faces = lcc.one_dart_per_cell<2>();
	for (auto f = faces.begin(); f != faces.end(); f++) {

		std::vector<Vec3> verts;
		auto f_darts = lcc.darts_of_cell<2, 2>(f); //one side
		for (auto p = f_darts.begin(); p != f_darts.end(); p++) {
			auto point = lcc.point(p);
			verts.push_back(Vec3{
				(float)CGAL::to_double(point.x()),
				(float)CGAL::to_double(point.y()),
				(float)CGAL::to_double(point.z())
				});
		}

		polys.push_back(Polygon_GL{ std::move(verts), rand_color() });
	}
	return std::make_unique<Polygon_Mesh>(std::move(polys));
}