#include "kinetic.h"
#include <limits>
#include <algorithm>
//#include "log.h"
#undef max
#undef min

inline void KPoint_2::set_collide_ray(std::shared_ptr<Collide_Ray> _collide_ray) {
    collide_ray = _collide_ray;
    ray_reverse = collide_ray->is_reverse(*this);
}
inline void KPoint_2::set_sliding_line(KLine_Ref kline)
{
    sliding_line = kline;
    set_collide_ray(kline->collide_ray);
}


KPolygon_2::KPolygon_2(KPolygons_2 *_parent, Polygon_2 poly_2)
    : parent(_parent), _polygon_2(std::move(poly_2))
{
    dirty = false;
    // center
    Vector_2 center_V = CGAL::NULL_VECTOR;
    for (const auto &point_2 : _polygon_2.container())
        center_V += point_2 - CGAL::ORIGIN;
    center_V = center_V / _polygon_2.size();
    Point_2 center_P = CGAL::ORIGIN + center_V;
    assert(_polygon_2.has_on_bounded_side(center_P));
    // insert vertices
    for (const auto &point_2 : _polygon_2.container())
    {
        auto kp = parent->new_KP(KPoint_2{point_2, point_2 - center_P, Mode::Normal});
        steal_kp(BACK, kp);
    }
    // insert edge
    auto vert = vert_circulator(), end = vert;
    CGAL_For_all(vert, end)
        vert->set_edge(std::make_shared<Edge>(vert, std::next(vert)));
}


Collide_Ray::Collide_Ray(Ray_2 _ray, KLine_Ref begin, KLine_Ref end) :
    ray(_ray), start(_ray.start()), vec(_ray.to_vector())
{
    auto line = ray.supporting_line();
    for (auto kline_2 = begin; kline_2 != end; kline_2++)
        if (auto res = CGAL::intersection(line, kline_2->_line_2)) {
            if (auto point_2_p = boost::get<Point_2>(&*res))
            {
                auto diff = *point_2_p - start;
                records.push_back(Record{ diff * vec, kline_2, *point_2_p });
            }
        }
    std::sort(records.begin(), records.end(), [](Record a, Record b) {
        return a.dot < b.dot;
    });
}
std::vector<Collide_Ray::Record> Collide_Ray::next_hit(KP_Ref kp) {
    bool reverse = kp->ray_reverse;
    auto start_diff = kp->point() - start;
    if (!reverse) {
        auto next_record = std::upper_bound(records.begin(), records.end(), start_diff * vec, [](FT dot, Record rec) {
            return dot < rec.dot;
        });
        return std::vector<Record>{next_record, records.end()};
    }

    auto next_record = std::lower_bound(records.begin(), records.end(), start_diff * vec, [](Record rec, FT dot) {
        return rec.dot < dot;
    });
    auto ret = std::vector<Record>{ records.begin(), next_record };
    std::reverse(ret.begin(), ret.end());
    return ret;

}


std::pair<Vert_Circ, Vert_Circ> get_vert(const Event &event1, const Event &event2)
{
    auto vert1 = event1.kp->vertex;
    auto vert2 = event2.kp->vertex;
    auto vert1_twin = vert1->twin();
    auto vert2_twin = vert2->twin();

    if (vert1->face->id == vert2->face->id)
        return {vert1, vert2};
    if (vert1_twin->face->id == vert2->face->id)
        return {vert1_twin, vert2};
    if (vert1->face->id == vert2_twin->face->id)
        return {vert1, vert2_twin};
    assert(vert1_twin->face->id == vert2_twin->face->id);
    return {vert1_twin, vert2_twin};
}

std::vector<KP_Ref> Kinetic_queue::type_c(Event event1, Event event2)
{
    assert(event1.pos == event2.pos);

    auto vert1 = event1.kp->vertex;
    auto vert2 = event2.kp->vertex;
    auto parent = vert1->face->parent;
    assert(vert1->kp->_status == Mode::Sliding);
    assert(vert2->kp->_status == Mode::Sliding);

    *vert1->kp = event1.pos;
    *vert2->kp = event2.pos;
    assert(event1.kline->_line_2.has_on(vert1->kp->point()));
    assert(event2.kline->_line_2.has_on(vert2->kp->point()));
    event1.kline->move_to_t(event1.t);
    event2.kline->move_to_t(event1.t);

    if (!vert1->has_twin() && !vert2->has_twin())
    {
        //std::cout << last_t << ": type c\n";
        assert(std::next(vert1) == vert2 || std::next(vert2) == vert1);
        erase_kp(parent, vert1->kp);
        vert1->face->erase(vert1);
        vert2->kp->frozen();
        return {vert2->kp};
    }

    if (vert1->has_twin() != vert2->has_twin())
    {
        //std::cout << last_t << ": one type c + one type b\n";

        if (vert1->has_twin())
            std::swap(event1, event2);
        vert1 = event1.kp->vertex;
        vert2 = event2.kp->vertex;
        assert(!vert1->has_twin() && vert2->has_twin());
        if (vert1->face->id == vert2->twin()->face->id)
            vert2 = vert2->twin();
        assert(vert1->face->id == vert2->face->id);
        assert(std::next(vert1) == vert2 || std::prev(vert1) == vert2);
        auto face = vert2->face;
        auto twin = vert2->twin();

        vert1->face->steal_kp(vert1, parent->new_KP(KPoint_2{ vert1->kp->point(), CGAL::NULL_VECTOR, Mode::Frozen }));
        erase_kp(vert1->face->parent, vert1->kp);
        // vert2->kp belong to type_b 
        face->erase(vert1);
        face->erase(vert2);

        return type_b(twin, event2.kline); //todo: let type b extend??
    }

    { // one type c + two type b
        //std::cout << last_t << ": one type c + two type b\n";
        auto [vert1, vert2] = get_vert(event1, event2);
        if (std::prev(vert1) == vert2)
        {
            std::swap(vert1, vert2);
            std::swap(event1, event2);
        }
        assert(std::next(vert1) == vert2);

        auto face = vert1->face;
        auto vert1_twin = vert1->twin();
        auto vert1_twin_face = vert1_twin->face;
        auto vert2_twin = vert2->twin();
        auto vert2_twin_face = vert2_twin->face;
        auto kp1 = vert1->kp;
        auto kp2 = vert2->kp;
        auto frozen_kp = parent->new_KP(KPoint_2{ kp1->point(), CGAL::NULL_VECTOR, Mode::Frozen });
        auto edge = vert2_twin->edge;
        // from now we can erase Vert_Circ

        // type c
        face->steal_kp(vert1, frozen_kp);
        face->erase(vert1);
        face->erase(vert2);
        // type b
        auto extend_vert2 = vert1_twin_face->steal_kp(vert1_twin, kp2);
        vert1_twin_face->steal_kp(vert1_twin, frozen_kp);
        vert1_twin_face->erase(vert1_twin);
        // type b
        vert2_twin_face->steal_kp(vert2_twin, frozen_kp);
        auto extend_vert1 = vert2_twin_face->steal_kp(vert2_twin, kp1);
        extend_vert1->set_edge(edge);
        vert2_twin_face->erase(vert2_twin);
        // extend
        auto new_face = parent->insert_kpoly_2();
        new_face->steal_kp(BACK, frozen_kp);
        auto tmp = new_face->steal_kp(BACK, kp2);
        set_twin(tmp, extend_vert2);
        tmp->set_edge(edge);
        set_twin(new_face->steal_kp(BACK, kp1), extend_vert1);

        return {};
    }
}

std::vector<KP_Ref> Kinetic_queue::type_b(Vert_Circ vert, KLine_Ref kline)
{
    const auto& line_2 = kline->_line_2;
    auto face = vert->face;
    auto kp = vert->kp;
    auto parent = face->parent;

    assert(kp->_status == Mode::Sliding);

    auto frozen_kp = parent->new_KP(KPoint_2{ kp->point(), CGAL::NULL_VECTOR, Mode::Frozen });

    Vert_Circ new_vert1, new_vert2;
    if (std::prev(vert)->edge)
    {
        //std::cout << last_t << ": type b Sliding_Prev\n";
        auto new_kp = parent->new_sliding_KP(*std::prev(vert)->edge, kline, kp->point());
        new_vert1 = face->steal_kp(vert, new_kp);
        new_vert2 = face->steal_kp(vert, frozen_kp);
    }
    else
    {
        assert(vert->edge);
        //std::cout << last_t << ": type b Sliding_Next\n";
        new_vert1 = face->steal_kp(vert, frozen_kp);
        auto new_kp = parent->new_sliding_KP(*vert->edge, kline, kp->point());
        new_vert2 = face->steal_kp(vert, new_kp);
        new_vert2->set_edge(vert->edge);
    }
    face->erase(vert);

    if (stop_extend(kline, kp))
    { // stop extend
        erase_kp(parent, kp);
        return { new_vert1->kp, new_vert2->kp };
    }

    //extend
    kline->add_seg_twin(new_vert1, new_vert2);
    auto new_face = parent->insert_kpoly_2();
    auto extend_vert = new_face->steal_kp(BACK, kp);
    auto extend_vert2 = new_face->steal_kp(BACK, new_vert2->kp);
    auto extend_vert1 = new_face->steal_kp(BACK, new_vert1->kp);
    if (new_vert1->kp->_status == Mode::Sliding)
        extend_vert1->set_edge(std::prev(new_vert1)->edge);
    if (new_vert2->kp->_status == Mode::Sliding)
        extend_vert->set_edge(new_vert2->edge);
    set_twin(new_vert2, extend_vert2);
    set_twin(new_vert1, extend_vert1);

    return { extend_vert1->kp, extend_vert2->kp, extend_vert->kp };
}

std::vector<KP_Ref> Kinetic_queue::type_a(const Event &event)
{
    auto kline = event.kline;
    const auto &line_2 = kline->_line_2;
    auto kp = event.kp;
    auto vert = kp->vertex;
    auto prev_vert = std::prev(vert);
    auto next_vert = std::next(vert);
    auto face = vert->face;
    auto parent = face->parent;

    *kp = event.pos;
    assert(line_2.has_on(kp->point()));
    kline->move_to_t(event.t);

    if (prev_vert->kp->_status == Mode::Sliding && prev_vert->kp->sliding_line == kline)
    {
        //std::cout << last_t << ": vert collision\n";
        assert(!vert->has_twin());
        auto sliding_vert = prev_vert;
        bool has_twin = sliding_vert->has_twin();

        auto new_speed = vert->edge->sliding_speed(line_2);
        kline->twin->move_to_t(event.t);
        sliding_vert->kp->sliding_speed(new_speed);
        *sliding_vert->kp = kp->point();
        sliding_vert->set_edge(vert->edge);

        if (has_twin)
        {
            auto twin_vert = sliding_vert->twin();
            twin_vert->face->steal_kp(twin_vert, kp)->set_edge(vert->edge);
        }
        else
            erase_kp(parent, kp);

        face->erase(vert);
        return {sliding_vert->kp};
    }
    else if (next_vert->kp->_status == Mode::Sliding && next_vert->kp->sliding_line == kline)
    {
        //std::cout << last_t << ": vert collision\n";
        assert(!vert->has_twin());
        auto sliding_vert = next_vert;
        bool has_twin = sliding_vert->has_twin();

        auto new_speed = std::prev(vert)->edge->sliding_speed(line_2);
        kline->twin->move_to_t(event.t);
        sliding_vert->kp->sliding_speed(new_speed);
        *sliding_vert->kp = kp->point();

        if (has_twin)
        {
            auto sliding_twin = sliding_vert->twin();
            assert(vert->edge == sliding_twin->edge);
            sliding_twin->face->steal_kp(std::next(sliding_twin), kp)->set_edge(vert->edge);
            sliding_twin->set_edge(prev_vert->edge);
        }
        else
            erase_kp(parent, kp);

        face->erase(vert);
        return {sliding_vert->kp};
    }

    {
        //std::cout << last_t << ": type a\n";
        assert(prev_vert->kp->_status != Mode::Frozen);
        assert(next_vert->kp->_status != Mode::Frozen);
        auto sliding_prev_KP = face->parent->new_sliding_KP(*std::prev(vert)->edge, kline, kp->point());
        auto sliding_next_KP = face->parent->new_sliding_KP(*vert->edge, kline, kp->point());
        auto next_edge = vert->edge;
        auto prev_edge = std::prev(vert)->edge;
        assert(-sliding_prev_KP->_speed.direction() == sliding_next_KP->_speed.direction());

        auto new_vert1 = face->steal_kp(vert, sliding_prev_KP);
        auto new_vert2 = face->steal_kp(vert, sliding_next_KP);
        new_vert2->set_edge(next_edge);

        if (stop_extend(kline, vert->kp))
        { // stop extend
            erase_kp(parent, kp);
        }
        else
        { //extend
            kline->add_seg_twin(new_vert1, new_vert2);
            auto triangle = face->parent->insert_kpoly_2();
            triangle->steal_kp(BACK, kp)->set_edge(next_edge);
            auto tri_vert2 = triangle->steal_kp(BACK, sliding_next_KP);
            auto tri_vert1 = triangle->steal_kp(BACK, sliding_prev_KP);
            tri_vert1->set_edge(prev_edge);
            set_twin(new_vert2, tri_vert2);
            set_twin(new_vert1, tri_vert1);
        }
        
        face->erase(vert);
        return {new_vert1->kp, new_vert2->kp};
    }
}

void Kinetic_queue::update_certificate()
{
    auto event = top();
    pop();
    auto dt = event.t - last_t;
    last_t = event.t;
    assert(dt > 0);
    static float accumulated = 0;
    accumulated += CGAL::to_double(dt);
    if (accumulated > 0.1) {
        accumulated = 0;
        std::cout << "kinetic time : " << CGAL::to_double(last_t) << std::endl;
    }
    auto next_event = top();

    if (event.kp->_status == Mode::Normal)
    {
        assert((next_event.t != event.t));
        need_update = type_a(event);
    }

    else if (next_event.t == event.t)
    {
        pop();
        if (top().t == next_event.t) std::exit(__LINE__);//Simultaneous collision
        need_update = type_c(event, next_event);
    }
    else {
        assert(!event.kp->vertex->has_twin());
        *event.kp = event.pos;
        event.kline->move_to_t(event.t);
        need_update = type_b(event.kp->vertex, event.kline);
    }


    for (auto kp : need_update)
    {
        remove_events(kp);
        kp_collide(kp);
    }
    //if (event_num[event.kp_id] == 0 && id_events[event.kp_id].size() > 0)
    //    kp_collide(event.kp);
    //if (event_num[next_event.kp_id] == 0 && id_events[next_event.kp_id].size() > 0)
    //    kp_collide(next_event.kp);
}

Kinetic_queue::Kinetic_queue(KPolygons_SET &kpolygons_set, bool exhausted)
    : kpolygons_set(kpolygons_set), exhausted(exhausted)
{
    std::cout << last_t << ": num of polygons set " << kpolygons_set.size() << std::endl;
    for (auto &kpolys_2 : kpolygons_set._kpolygons_set)
        for (auto kp = kpolys_2.active_KP.begin(); kp != kpolys_2.active_KP.end(); kp++)
            kp_collide(kp);

    std::cout << last_t << ": queue size " << queue.size() << std::endl;
}

void Kinetic_queue::kp_collide(KP_Ref kp)
{
    if (kp->_status == Mode::Frozen)
        return;

    auto next_records = kp->collide_ray->next_hit(kp);
    size_t max_event = (size_t)-1;
    for (const auto& rec : next_records) {
        if (max_event-- == 0)
            break;
        auto diff = rec.pos - *kp;
        auto t = last_t;
        t += Vec_div(diff, kp->_speed);
        insert(Event{t, kp, rec.kline_2, rec.pos});
    }

}

FT Vec_div(Vector_2 v1, Vector_2 v2)
{
    if (v1 == CGAL::NULL_VECTOR)
        return 0;
    assert(v1.direction() == v2.direction() || -v1.direction() == v2.direction());
    if (v2.x() != 0)
    {
        return v1.x() / v2.x();
    }
    else
    {
        assert(v2.y() != 0);
        return v1.y() / v2.y();
    }
}

void Kinetic_queue::erase_kp(KPolygons_2 *parent, KP_Ref kp)
{
    kp->frozen();
    remove_events(kp);
    // erase AFTER we update queue
    parent->erase_kp(kp);
}
FT Kinetic_queue::next_time()
{
    if (!queue.empty())
        return top().t;
    return INF;
}

FT Kinetic_queue::to_next_event()
{
    if (!queue.empty())
    {

        update_certificate();

    }
    return last_t;
}

size_t max_id = 0;
size_t next_id()
{
    auto next_id = max_id++;
    if (next_id == 2)
        std::cout << "debug" << std::endl;
    return next_id;
}

FT Kinetic_queue::move_to_time(FT t)
{
    assert(t >= last_t);
    auto next_t = next_time();
    if (t < next_t)
    {
        kpolygons_set.move_dt(t - last_t);
        last_t = t;
        return last_t;
    }
    else {
        kpolygons_set.move_dt(next_t - last_t);
        return to_next_event();
    }
}

void Kinetic_queue::done(){
    while (!queue.empty()) to_next_event();
}

KPolygons_SET::KPolygons_SET(Polygons_3 polygons_3, bool exhausted)
{
    for (auto &poly_3 : polygons_3)
        _kpolygons_set.emplace_back(std::move(poly_3));

    add_bounding_box(polygons_3);
    if (exhausted)
        bbox_clip();

    decompose();

    if (exhausted)
        for (auto &polys : _kpolygons_set)
            polys.frozen();
}

void KPolygons_SET::bbox_clip()
{
    // clip supporting plane by bounding box

    auto bbox_begin = prev(_kpolygons_set.end(), 6);
    for (auto polys_i = _kpolygons_set.begin(); polys_i != bbox_begin; polys_i++)
    {
        std::vector<Point_2> points_2;
        for (auto bbox = bbox_begin; bbox != _kpolygons_set.end(); bbox++)
        {
            if (auto res = plane_polygon_intersect_3(polys_i->plane(), bbox->polygons_3().front()))
            {
                auto &[p1, p2] = *res;
                assert(polys_i->plane().has_on(p1));
                assert(polys_i->plane().has_on(p2));
                points_2.push_back(polys_i->plane().to_2d(p1));
                points_2.push_back(polys_i->plane().to_2d(p2));
            }
        }
        auto polygon_2 = get_convex(points_2.begin(), points_2.end());

        // replace by the clipped polygon
        assert(polys_i->_kpolygons_2.size() == 1);
        auto inline_points = polys_i->_kpolygons_2.front().inline_points;
        polys_i->_kpolygons_2.clear();
        polys_i->insert_kpoly_2(polygon_2)->set_inline_points(inline_points);
    }
}

void KPolygons_SET::decompose()
{
    for (auto polys_i = _kpolygons_set.begin(); polys_i != _kpolygons_set.end(); polys_i++)
        for (auto polys_j = std::next(polys_i); polys_j != _kpolygons_set.end(); polys_j++)
            if (auto res = CGAL::intersection(polys_i->plane(), polys_j->plane()))
                if (auto line_3 = boost::get<Line_3>(&*res))
                {
                    auto line_i = polys_i->insert_kline(polys_i->project_2(*line_3));
                    auto line_j = polys_j->insert_kline(polys_j->project_2(*line_3));
                    line_i->twin = line_j;
                    line_j->twin = line_i;
                    if (polys_i->is_bbox || polys_j->is_bbox)
                    {
                        line_i->is_bbox = true;
                        line_j->is_bbox = true;
                    }
                }

    for (auto &kpolys_2 : _kpolygons_set)
    {
        // init collide_ray
        for (auto& kp : kpolys_2.active_KP)
            kp.set_collide_ray(std::make_shared<Collide_Ray>(Ray_2{ kp.point(), kp._speed }, kpolys_2.klines().begin(), kpolys_2.klines().end()));
        for (auto& kline_2 : kpolys_2.klines()) {
            Point_2 start;
            if(kline_2._line_2.is_vertical())
                start = Point_2{ kline_2._line_2.x_at_y(0), 0 };
            else
                start = Point_2{ 0, kline_2._line_2.y_at_x(0) };

            auto ray = Ray_2{ start ,kline_2._line_2 };
            kline_2.collide_ray = std::make_shared<Collide_Ray>(ray, kpolys_2.klines().begin(), kpolys_2.klines().end());
        }

        // split
        for (auto kline_2 = kpolys_2.klines().begin(); kline_2 != kpolys_2.klines().end(); kline_2++)
        {
            auto current_max_id = max_id;
            auto kpoly_2 = kpolys_2._kpolygons_2.begin();
            while (kpoly_2 != kpolys_2._kpolygons_2.end())
            {
                if (kpoly_2->id >= current_max_id)
                    break;
                if (kpolys_2.try_split(kpoly_2, kline_2))
                    kpoly_2 = kpolys_2._kpolygons_2.erase(kpoly_2);
                else
                    kpoly_2++;
            }
        }
    }
}

Vector_2 Edge::sliding_speed(const Line_2 &line_2) const
{
    auto res = CGAL::intersection(line(), line_2);
    assert(res);
    if (auto cur_point = boost::get<Point_2>(&*res))
    {
        const auto &line_dt = moved_line();
        auto res_dt = CGAL::intersection(line_dt, line_2);
        assert(res_dt);

        if (auto point_dt = boost::get<Point_2>(&*res_dt))
        {
            auto speed = *point_dt - *cur_point;
            if (speed.squared_length() > 0)
            {
                assert(line_2.direction() == speed.direction() ||
                       -line_2.direction() == speed.direction());
            }
            else
            {
                assert(speed == CGAL::NULL_VECTOR);
            }
            return speed;
        }
    }

    return CGAL::NULL_VECTOR;
}

bool KPolygons_2::try_split(KPoly_Ref kpoly_2, KLine_Ref kline_2)
{
    const auto &line = kline_2->_line_2;
    auto edges = kpoly_2->get_edges();

    auto kp_edges = std::vector<std::pair<KP_Ref, Edge>>{};
    for (const auto &edge : edges)
    {
        if (auto res = CGAL::intersection(edge.seg(), line))
        {
            if (auto point_2 = boost::get<Point_2>(&*res))
            {
                auto speed = edge.sliding_speed(line);
                auto mode = speed == CGAL::NULL_VECTOR ? Mode::Frozen : Mode::Sliding;
                auto kp = new_KP(KPoint_2{*point_2, speed, mode});
                if (mode == Mode::Sliding)
                    kp->set_sliding_line(kline_2);

                kp_edges.push_back(std::make_pair(
                    kp,
                    edge));
            }
            else {
                return false;
                //     assert(false);
            }
        }
    }
    if (kp_edges.empty())
        return false;
    assert(kp_edges.size() == 2);

    const auto &[new_kp1, e1] = kp_edges[0];
    const auto &[new_kp2, e2] = kp_edges[1];

    auto origin_size = kpoly_2->size();
    KPoly_Ref new_poly1 = insert_kpoly_2(), new_poly2 = insert_kpoly_2();

    auto poly1_vert1 = new_poly1->steal_kp(BACK, new_kp1);
    poly1_vert1->set_edge(e1.vert1->edge, No_Check{});
    for (auto vert = e1.vert2; vert != e2.vert2; vert++)
    {
        auto new_vert = new_poly1->steal_kp(BACK, vert->kp);
        if(vert->edge)
            new_vert->set_edge(vert->edge, No_Check{});
        if (vert->has_twin())
            set_twin(new_vert, vert->twin());
    }
    auto poly1_vert2 = new_poly1->steal_kp(BACK, new_kp2);

    auto poly2_vert2 = new_poly2->steal_kp(BACK, new_kp2);
    poly2_vert2->set_edge(e2.vert1->edge, No_Check{});
    for (auto vert = e2.vert2; vert != e1.vert2; vert++)
    {
        auto new_vert = new_poly2->steal_kp(BACK, vert->kp);
        if (vert->edge)
            new_vert->set_edge(vert->edge, No_Check{});
        if (vert->has_twin())
            set_twin(new_vert, vert->twin());
    }
    auto poly2_vert1 = new_poly2->steal_kp(BACK, new_kp1);

    set_twin(poly2_vert2, poly1_vert2);
    set_twin(poly2_vert1, poly1_vert1);
    kline_2->add_seg_twin(poly1_vert1, poly1_vert2);

    //assert(kpoly_2->size() == 0);
    assert((new_poly1->size() + new_poly2->size()) == (origin_size + 4));

    //split inline points
    for (auto &[point_3, normal] : kpoly_2->inline_points)
    {
        auto point_2 = plane().to_2d(point_3);
        if (new_poly1->polygon_2().has_on_bounded_side(point_2))
        {
            assert(!new_poly2->polygon_2().has_on_bounded_side(point_2));
            new_poly1->inline_points.push_back(std::make_pair(point_3, normal));
        }
        else if (new_poly2->polygon_2().has_on_bounded_side(point_2))
        {
            new_poly2->inline_points.push_back(std::make_pair(point_3, normal));
        }
    }

    return true;
}

Point_2 KLine_2::transform2twin(const Point_2 &point) const
{
    assert(_line_2.has_on(point));
    auto twin_kpolys = twin->kpolygons;
    auto ret = twin_kpolys->project_2(kpolygons->to_3d(point));
    assert(twin->_line_2.has_on(ret));
    return ret;
}

std::pair<Point_2, Vector_2> KLine_2::transform2twin(const KPoint_2 &kpoint) const
{
    auto point_twin = transform2twin(static_cast<Point_2>(kpoint));
    if (kpoint._speed == CGAL::NULL_VECTOR)
        return {point_twin, CGAL::NULL_VECTOR};
    auto vector_twin = transform2twin(kpoint + kpoint._speed) - point_twin;
    assert(twin->_line_2.direction() == vector_twin.direction() ||
           twin->_line_2.direction() == -vector_twin.direction());
    return {point_twin, vector_twin};
}

void KPolygons_SET::add_bounding_box(const Polygons_3 &polygons_3)
{
    auto box = CGAL::Bbox_3{};
    for (const auto &poly_3 : polygons_3)
        box += CGAL::bbox_3(poly_3.points_3().begin(), poly_3.points_3().end());

    FT scale = 0.1 + std::max(std::max({box.max(0), box.max(1), box.max(2)}),
                              std::abs(std::min({box.min(0), box.min(1), box.min(2)})));

    //FT scale = 1;
    auto square = Points_2{};
    // square.push_back(Point_2{scale + 0.1, scale + 0.1});
    // square.push_back(Point_2{-scale - 0.1, scale + 0.1});
    // square.push_back(Point_2{-scale - 0.1, -scale - 0.1});
    // square.push_back(Point_2{scale + 0.1, -scale - 0.1});

    square.push_back(Point_2{scale, scale});
    square.push_back(Point_2{-scale, scale});
    square.push_back(Point_2{-scale, -scale});
    square.push_back(Point_2{scale, -scale});

    {
        auto plane = Plane_3{1, 0, 0, scale};
        _kpolygons_set.emplace_back(Polygon_3{plane, square});
        _kpolygons_set.back().frozen();
        _kpolygons_set.back().is_bbox = true;
    }
    {
        auto plane = Plane_3{0, 1, 0, scale};
        _kpolygons_set.emplace_back(Polygon_3{plane, square});
        _kpolygons_set.back().frozen();
        _kpolygons_set.back().is_bbox = true;
    }

    {
        auto plane = Plane_3{0, 0, 1, scale};
        _kpolygons_set.emplace_back(Polygon_3{plane, square});
        _kpolygons_set.back().frozen();
        _kpolygons_set.back().is_bbox = true;
    }

    {
        auto plane = Plane_3{ -1, 0, 0, scale };
        _kpolygons_set.emplace_back(Polygon_3{ plane, square });
        _kpolygons_set.back().frozen();
        _kpolygons_set.back().is_bbox = true;
    }
    {
        auto plane = Plane_3{ 0, -1, 0, scale };
        _kpolygons_set.emplace_back(Polygon_3{ plane, square });
        _kpolygons_set.back().frozen();
        _kpolygons_set.back().is_bbox = true;
    }
    {
        auto plane = Plane_3{ 0, 0, -1, scale };
        _kpolygons_set.emplace_back(Polygon_3{ plane, square });
        _kpolygons_set.back().frozen();
        _kpolygons_set.back().is_bbox = true;
    }
}

inline void Vertex::set_edge(const std::shared_ptr<Edge>& _edge) {
    assert(_edge);
    edge = _edge;
}
inline void Vertex::set_edge(const std::shared_ptr<Edge>& _edge, No_Check) {
    edge = _edge;
}