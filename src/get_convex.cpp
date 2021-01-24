#include "cgal_object.h"
#include "region_growing.h"
//#include "log.h"

// region_growing.h does not need to be included in cgal_object.h

Polygon_2 simplify_convex(const Polygon_2& polygon);


Polygons_3 detect_shape(std::string path)
{
	std::vector<Detected_shape> detected_shape = region_growing(path);
	Polygons_3 results;
	for (auto& [plane, pwn] : detected_shape)
	{

		auto plane_3 = Plane_3{
			FT{plane.a()},
			FT{plane.b()},
			FT{plane.c()},
			FT{plane.d()} };

		PWN_E inline_points;
		std::vector<Point_2> projected_points;
		for (auto [v, n] : pwn)
		{
			auto point_3 = Point_3{ FT{v.x()}, FT{v.y()}, FT{v.z()} };
			auto normal = Vector_3{ FT{n.x()}, FT{n.y()}, FT{n.z()} };
			inline_points.push_back(std::make_pair(point_3, normal));
			Point_3 projected = plane_3.projection(point_3);
			projected_points.push_back(plane_3.to_2d(projected));
		}

		 //auto polygon2 = get_convex(projected_points.begin(), projected_points.end());
		 auto polygon2 = simplify_convex(get_convex(projected_points.begin(), projected_points.end()));
		//for (const auto& p : projected_points)
		//	assert(!polygon2.has_on_unbounded_side(p));

		auto poly3 = Polygon_3{ plane_3, std::move(polygon2) };
		poly3.set_inline_points(std::move(inline_points));
		results.push_back(std::move(poly3));
	}
	return results;
}

Polygon_2 get_convex(Points_2::const_iterator begin, Points_2::const_iterator end) {
	//get convex point
	std::vector<Point_2> convex_points;
	CGAL::convex_hull_2(begin, end, std::back_inserter(convex_points));
	Polygon_2 polygon2 = Polygon_2(convex_points.begin(), convex_points.end());
	assert(polygon2.is_simple());
	assert(polygon2.is_convex());
	return polygon2;
}





// --------------- simplify convex ------------------------

size_t next_id = 0;
class Point_2_id : public Point_2 {
public:
	explicit Point_2_id(Point_2 p) :Point_2(p) {}
	Point_2_id& operator=(const Point_2& point)
	{
		Point_2::operator=(point);
		return *this;
	}
	Point_2_id& operator=(Point_2&& point)
	{
		Point_2::operator=(std::move(point));
		return *this;
	}
	size_t id = next_id++;
};

using P_Circ = CGAL::Circulator_from_container<std::list<Point_2_id>>;
class Event
{
public:
	FT cost;
	P_Circ p = P_Circ{};
	Event() = default;
	Event(FT _cost, P_Circ _p)
		: cost(_cost), p(_p)
	{
		assert(cost > 0);
	}
};
class Pri_queue
{
public:
	void try_insert(P_Circ p) {
		auto prev = std::prev(p);
		auto next = std::next(p);
		auto p3 = Point_3{ p->x(),p->y(), 0 };
		auto prev_p3 = Point_3{ prev->x(),prev->y(), 0 };
		auto next_p3 = Point_3{ next->x(),next->y(), 0 };
		FT degree = CGAL::approximate_angle(p3-prev_p3, next_p3 - p3);
		std::cout << degree << std::endl;
		if (degree < 10) {
			auto e = Event{ degree, p };
			queue.insert(e);
			id_event[p->id] = e;
			
		}
	}
	void remove(size_t id) { 
		remove(id_event[id]); 
		id_event.erase(id);
	}

	const Event& top(void) const { return *(queue.begin()); }
	void pop(void) { queue.erase(queue.begin()); }
	size_t size() { return queue.size(); }
private:
	void insert(const Event& event) { queue.insert(event); }
	void remove(const Event& event)
	{
		if (queue.find(event) != queue.end())
		{
			queue.erase(event);
		}
	}
	std::set<Event> queue;
	std::unordered_map<size_t, Event> id_event;

};

/* Comparison operator for Event so std::set will properly order them */
inline bool operator<(const Event& r1, const Event& r2)
{
	if (r1.cost != r2.cost)
	{
		return r1.cost < r2.cost;
	}
	return &*r1.p < &*r2.p;
}

Polygon_2 simplify_convex(const Polygon_2& polygon) {

	std::list<Point_2_id> simplified{ polygon.begin(), polygon.end() };

	Vector_2 center_V = CGAL::NULL_VECTOR;
	for (const auto& point_2 : polygon.container())
		center_V += point_2 - CGAL::ORIGIN;
	center_V = center_V / polygon.size();
	Point_2 center_P = CGAL::ORIGIN + center_V;
	assert(polygon.has_on_bounded_side(center_P));


	Pri_queue queue;
	auto p = P_Circ{ &simplified }, end = p;
	CGAL_For_all(p, end) queue.try_insert(p);

	while (queue.size() > 0) {
		auto p = queue.top().p;
		auto next_p = std::next(p);
		auto prev_p = std::prev(p);
		queue.pop();

		// auto ave = ((*p  - *std::prev(p)) + (*std::next(p) - *p)) / 2;
		// assert(ave != CGAL::NULL_VECTOR);

		// auto res = CGAL::intersection(Ray_2{ center_P ,*std::next(p) - center_P }, Line_2{*p, ave });
		// assert(res);
		// *next_p = *boost::get<Point_2>(&*res);

		// res = CGAL::intersection(Ray_2{ center_P ,*std::prev(p) - center_P }, Line_2{ *p, ave });
		// assert(res);
		// *prev_p = *boost::get<Point_2>(&*res);


		queue.remove(prev_p->id);
		queue.remove(next_p->id);
		simplified.erase(p.current_iterator());
		queue.try_insert(prev_p);
		queue.try_insert(next_p);
	}
	return Polygon_2{ simplified.begin(), simplified.end()};
}
