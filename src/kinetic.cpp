#include "kinetic.h"
#include <limits>
#include <algorithm>

void Vertex::transfer_to(Vert_Circ to)
{                               // update kp's reference
    if (kp->vertex->_id == _id) // list::iterator of different list instances cannot be compared
        kp->vertex = to;
    else if (kp->twin_vert->_id == _id)
        kp->twin_vert = to;
    else
        assert(false);
}

//void Vertex::set_twin(Vert_Circ twin)
//{
//    assert(kp->vertex->_id == _id);
//    kp->twin_vert = twin;
//}

bool Vertex::has_twin()
{

    if (kp->vertex == nullptr || kp->twin_vert == nullptr)
        return false;
    return true;
}
Vert_Circ &Vertex::twin()
{
    if (kp->vertex->_id == _id)
        return kp->twin_vert;
    else if (kp->twin_vert->_id == _id)
        return kp->vertex;
    else
        assert(false);
}
bool Vertex::stop_extend(KLine_Ref kline) {
    if( kline->is_bbox) return true;
    auto ret = kline->has_on(kp->point());
    // if(ret) kline->is_bbox = true;
    return ret;
}

void KPoint_2::assert_twin_vert()
{
    assert(twin_vert->kp->id() == id());
    assert(twin_vert->face->id != vertex->face->id);
}

KPolygon_2::KPolygon_2(KPolygons_2 *_parent, Polygon_2 poly_2)
    : parent(_parent), _polygon_2(std::move(poly_2))
{
    dirty = false;

    Vector_2 center_V = CGAL::NULL_VECTOR;
    for (const auto &point_2 : _polygon_2.container())
        center_V += point_2 - CGAL::ORIGIN;
    center_V = center_V / _polygon_2.size();
    Point_2 center_P = CGAL::ORIGIN + center_V;
    assert(_polygon_2.has_on_bounded_side(center_P));

    for (const auto &point_2 : _polygon_2.container())
         append_KP(KPoint_2{point_2, point_2 - center_P, Mode::Normal});
        //append_KP(KPoint_2{point_2, (point_2 - center_P)/_polygon_2.area(), Mode::Normal});
}

Vert_Circ KPolygon_2::insert_KP(std::list<Vertex>::iterator pos, KPoint_2 &&kpoint)
{
    dirty = true;
    auto kp = parent->new_KP(std::move(kpoint));

    auto vert = Vert_Circ{&vertices, vertices.insert(pos, Vertex{kp})};
    vert->face = this;
    vert->_id = next_id();

    kp->vertex = vert;
    return vert;
}

std::optional<Event> tmp_store;

Event Kinetic_queue::last_event()
{
    if (tmp_store)
    {
        auto next_event = tmp_store.value();
        tmp_store.reset();
        return next_event;
    }
    else
    {
        auto next_event = top();
        return next_event;
    }
}

std::vector<KP_Ref> Kinetic_queue::type_c(Vert_Circ vert, KLine_Ref kline, const Event &event)
{
    const auto &line_2 = kline->_line_2;
    auto face = vert->face;
    auto kp = vert->kp;
    auto prev_vert = std::prev(vert);
    auto next_vert = std::next(vert);


    if (next_vert->kp->point() == kp->point())
    { //type c
        assert(next_vert->kp->_status == Mode::Sliding);
        auto next_event = last_event();

        if (!vert->has_twin() && next_vert->has_twin())
        {
            // one type c + one type b
            std::cout << last_t << ": one type c + one type b" << std::endl;
            erase_vert(vert);
            assert(next_event.kp->id() == next_vert->kp->id());
            return type_b(next_vert->twin(), next_event.kline);
        }
        else if (!vert->has_twin() && !next_vert->has_twin())
        { // only type c
            std::cout << last_t << ": type c" << std::endl;
            // vert->kp->frozen();
            erase_vert(vert);
            next_vert->kp->frozen();
            return {next_vert->kp};
        }

        else if (vert->has_twin() && !next_vert->has_twin())
        { // one type c + one type b
            std::cout << last_t << ": one type c + one type b" << std::endl;
            // next_vert->kp->frozen();
            erase_vert(next_vert);
            return type_b(vert->twin(), kline);
        }
        else if (vert->has_twin() && next_vert->has_twin())
        { // one type c + two type b
            std::cout << last_t << ": one type c + two type b" << std::endl;
            auto next_twin = next_vert->twin();
            auto next_twin_face = next_vert->twin()->face;
            auto twin = vert->twin();
            auto twin_face = vert->twin()->face;
            auto frozen_kpoint = KPoint_2{kp->point(), CGAL::NULL_VECTOR, Mode::Frozen};
            auto frozen_vert = face->insert_KP(vert, frozen_kpoint);

            auto stolen_next_vert = twin_face->steal_vert(twin, next_vert);
            auto stolen_frozen = twin_face->steal_vert(twin, frozen_vert);

            next_twin_face->steal_as_twin(next_twin, stolen_frozen);
            auto stolen_vert = next_twin_face->steal_vert(next_twin, vert);

            auto triangle = face->parent->insert_kpoly_2();
            auto extend_vert = triangle->append_KP(frozen_kpoint);
            triangle->steal_as_twin_bk(stolen_next_vert);
            triangle->steal_as_twin_bk(stolen_vert);
            twin_face->erase(twin);
            next_twin_face->erase(next_twin);
            face->erase(vert);
            face->erase(next_vert);
            return { stolen_vert->kp, stolen_next_vert ->kp};
        }
        else
            assert(false);
    }
    else if (prev_vert->kp->point() == kp->point())
    { //type c
        assert(prev_vert->kp->_status == Mode::Sliding);
        // let prev_kp's event handle it
        tmp_store = event;
        return {};
    }
}

std::vector<KP_Ref> Kinetic_queue::type_b(Vert_Circ vert, KLine_Ref kline)
{
    const auto &line_2 = kline->_line_2;
    auto face = vert->face;
    auto kp = vert->kp;
    {
        auto prev_speed = KPolygon_2::Edge{std::prev(vert), vert}.sliding_speed(line_2);
        auto next_speed = KPolygon_2::Edge{vert, std::next(vert)}.sliding_speed(line_2);
        auto extend_kpoint = KPoint_2{kp->point(), kp->_speed, Mode::Sliding};

        //steal seg_twin_speed pointer
        // extend_kpoint.seg_twin_speed = vert->kp->seg_twin_speed;
        // vert->kp->seg_twin_speed = nullptr;

        auto new_vert1 = vert, new_vert2 = vert;
        if (next_speed == CGAL::NULL_VECTOR)
        {
            std::cout << last_t << ": type b Sliding_Prev" << std::endl;
            new_vert1 = face->insert_KP(vert, KPoint_2{kp->point(), prev_speed, Mode::Sliding});
            new_vert2->kp->frozen();
        }
        else
        {
            assert(prev_speed == CGAL::NULL_VECTOR);
            std::cout << last_t << ": type b Sliding_Next" << std::endl;
            new_vert1->kp->frozen();
            new_vert2 = face->insert_KP(std::next(new_vert1), KPoint_2{kp->point(), next_speed, Mode::Sliding});
        }

        if (vert->stop_extend(kline))
        { // stop extend
            return {new_vert1->kp, new_vert2->kp};
        }

        //extend
        kline->add_seg_twin(new_vert1, new_vert2);
        auto triangle = face->parent->insert_kpoly_2();

        auto extend_vert = triangle->append_KP(extend_kpoint);
        triangle->steal_as_twin_bk(new_vert2);
        triangle->steal_as_twin_bk(new_vert1);
        return {new_vert1->kp, new_vert2->kp, extend_vert->kp};
    }
}

std::vector<KP_Ref> Kinetic_queue::update_certificate(const Event &event)
{

    auto kline = event.kline;
    const auto &line_2 = kline->_line_2;
    auto kp = event.kp;
    auto vert = kp->vertex;
    auto prev_vert = std::prev(vert);
    auto next_vert = std::next(vert);
    auto face = vert->face;

    assert(line_2.has_on(kp->point()));

    if (kp->_status == Mode::Normal)
    {
        if (prev_vert->kp->point() == kp->point())
        {
            std::cout << last_t << ": vert collision" << std::endl;
            assert(!kp->has_twin_vert());
            auto sliding_vert = prev_vert;
            assert(sliding_vert->kp->_status == Mode::Sliding);
            bool has_twin = sliding_vert->has_twin();
            auto new_speed = KPolygon_2::Edge{vert, std::next(vert)}.sliding_speed(line_2);
            sliding_vert->kp->sliding_speed(new_speed);

            if (!has_twin)
            {
                erase_vert(vert);
                return {sliding_vert->kp};
            }
            auto twin_vert = sliding_vert->twin();

            twin_vert->face->steal_vert(twin_vert, vert);
            face->erase(vert);
            return {sliding_vert->kp};
        }
        else if (kp->point() == next_vert->kp->point())
        {
            std::cout << last_t << ": vert collision" << std::endl;
            assert(!kp->has_twin_vert());
            auto sliding_vert = next_vert;
            assert(sliding_vert->kp->_status == Mode::Sliding);
            bool has_twin = sliding_vert->has_twin();
            auto new_speed = KPolygon_2::Edge{std::prev(vert), vert}.sliding_speed(line_2);
            sliding_vert->kp->sliding_speed(new_speed);

            if (!has_twin)
            {
                erase_vert(vert);
                return {sliding_vert->kp};
            }
            auto twin_vert = sliding_vert->twin();

            twin_vert->face->steal_vert(std::next(twin_vert), vert);
            face->erase(vert);
            return {sliding_vert->kp};
        }
    }

    switch (kp->_status)
    {
    case Mode::Normal: //type a
        if (line_2.has_on(prev_vert->kp->point()) && line_2.has_on(kp->point()))
        {
            assert(false);
            std::cout << last_t << ": type a edge" << std::endl;
            return {};
        }
        else if (line_2.has_on(next_vert->kp->point()) && line_2.has_on(kp->point()))
        {
            assert(false);
            std::cout << last_t << ": type a edge" << std::endl;
            return {};
        }
        else
        {
            std::cout << last_t << ": type a" << std::endl;
            assert(prev_vert->kp->_status != Mode::Frozen);
            assert(next_vert->kp->_status != Mode::Frozen);
            auto prev_speed = KPolygon_2::Edge{std::prev(vert), vert}.sliding_speed(line_2);
            auto next_speed = KPolygon_2::Edge{vert, std::next(vert)}.sliding_speed(line_2);
            assert(-prev_speed.direction() == next_speed.direction());

            auto new_vert1 = face->insert_KP(vert, KPoint_2{kp->point(), prev_speed, Mode::Sliding});
            auto new_vert2 = face->insert_KP(vert, KPoint_2{kp->point(), next_speed, Mode::Sliding});

            if (vert->stop_extend(kline))
            { // stop extend
                erase_vert(vert);
            }
            else
            { //extend
                kline->add_seg_twin(new_vert1, new_vert2);
                auto triangle = face->parent->insert_kpoly_2();
                triangle->steal_vert_bk(vert);
                face->erase(vert);
                triangle->steal_as_twin_bk(new_vert2);
                triangle->steal_as_twin_bk(new_vert1);
            }

            return {new_vert1->kp, new_vert2->kp};
        }
        break;
    case Mode::Sliding:
        if (prev_vert->kp->point() == kp->point() ||
            next_vert->kp->point() == kp->point())
        { //type c
            return type_c(vert, kline, event);
        }
        else
        {
            if (!vert->has_twin()) //type b
                return type_b(vert, kline);

            //let type c handle it
            return type_c(vert->twin(), kline, event);

            break;
        }
    default:
        assert(false);
    }
    return {};
}

Kinetic_queue::Kinetic_queue(KPolygons_SET &kpolygons_set) : kpolygons_set(kpolygons_set)
{
    std::cout << last_t << ": num of polygons set " << kpolygons_set.size() << std::endl;
    for (auto &kpolys_2 : kpolygons_set._kpolygons_set)
        for (auto kp = kpolys_2.all_KP.begin(); kp != kpolys_2.all_KP.end(); kp++)
            kp_collide(kp);

    std::cout << last_t << ": queue size " << queue.size() << std::endl;
}

void Kinetic_queue::kp_collide(KP_Ref kp)
{
    if (kp->_status == Mode::Frozen)
        return;
    assert(kp->_speed != CGAL::NULL_VECTOR);
    auto kpolys_2 = kp->vertex->face->parent;
    auto r = Ray_2{*kp, kp->_speed};
    for (auto kline_2 = kpolys_2->_klines.begin(); kline_2 != kpolys_2->_klines.end(); kline_2++)
        if (auto res = CGAL::intersection(r, kline_2->_line_2))
        {
            if (boost::get<Ray_2>(&*res)) //sliding
                continue;
            if (auto point_2_p = boost::get<Point_2>(&*res))
            {
                if (*point_2_p == *kp) // that means the point is about to leave the line
                    continue;
                auto diff = *point_2_p - *kp;
                auto t = last_t;
                t += Vec_div(diff, kp->_speed);
                auto event = Event{t, kp, kline_2};
                id_events[kp->id()].push_back(event);
                insert(event);
            }
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

void Kinetic_queue::erase_vert(Vert_Circ vert)
{
    auto kp = vert->kp;
    auto face = vert->face;
    remove_events(kp);
    // erase AFTER we update queue
    face->erase(vert);
    face->parent->erase_kp(kp);
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
        auto event = top();
        pop();
        auto dt = event.t - last_t;
        assert(dt >= 0);
        kpolygons_set.move_dt(dt);

        need_update = update_certificate(event); //assume kpolygons_set have already growed
        last_t = event.t;                             //update last_t before detect next collide()
        for (auto kp : need_update)
        {
            remove_events(kp);
            kp_collide(kp);
        }
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
    else
        return to_next_event();
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
                    if (polys_i->is_bbox || polys_j->is_bbox) {
                        line_i->is_bbox = true;
                        line_j->is_bbox = true;
                    }
                }

    for (auto &kpolys_2 : _kpolygons_set)
    {
        // split
        for (auto &kline_2 : kpolys_2.klines())
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

Vector_2 KPolygon_2::Edge::sliding_speed(const Line_2 &line_2) const
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

bool KPolygons_2::try_split(KPoly_Ref kpoly_2, KLine_2 &kline_2)
{
    const auto &line = kline_2._line_2;
    auto edges = kpoly_2->get_edges();

    auto p_e = std::vector<std::pair<KPoint_2, KPolygon_2::Edge>>{};
    for (const auto &edge : edges)
    {
        if (auto res = CGAL::intersection(edge.seg(), line))
        {
            if (auto point_2 = boost::get<Point_2>(&*res))
            {
                auto speed = edge.sliding_speed(line);
                auto mode = speed == CGAL::NULL_VECTOR ? Mode::Frozen : Mode::Sliding;
                p_e.push_back(std::make_pair(
                    KPoint_2{*point_2, speed, mode},
                    edge));
            }
            //todo: corner case
            // else
            //     assert(false);
        }
    }
    if (p_e.empty())
        return false;
    assert(p_e.size() == 2);

    const auto &[new_kp1, e1] = p_e[0];
    const auto &[new_kp2, e2] = p_e[1];

    auto origin_size = kpoly_2->size();
    KPoly_Ref new_poly1 = insert_kpoly_2(), new_poly2 = insert_kpoly_2();

    auto poly1_vert1 = new_poly1->append_KP(new_kp1);
    for (auto vert = e1.vert2; vert != e2.vert2; vert++)
        new_poly1->steal_vert_bk(vert);
    auto poly1_vert2 = new_poly1->append_KP(new_kp2);

    new_poly2->steal_as_twin_bk(poly1_vert2);
    for (auto vert = e2.vert2; vert != e1.vert2; vert++)
        new_poly2->steal_vert_bk(vert);
    new_poly2->steal_as_twin_bk(poly1_vert1);

    kline_2.add_seg_twin(poly1_vert1, poly1_vert2);

    //assert(kpoly_2->size() == 0);
    assert((new_poly1->size() + new_poly2->size()) == (origin_size + 4));

    //split inline points
    for (auto &point_3 : kpoly_2->inline_points)
    {
        auto point_2 = plane().to_2d(point_3);
        if (new_poly1->polygon_2().has_on_bounded_side(point_2))
        {
            assert(!new_poly2->polygon_2().has_on_bounded_side(point_2));
            new_poly1->inline_points.push_back(point_3);
        }
        else
        {
            assert(new_poly2->polygon_2().has_on_bounded_side(point_2));
            new_poly2->inline_points.push_back(point_3);
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
        auto plane = Plane_3{1, 0, 0, -scale};
        _kpolygons_set.emplace_back(Polygon_3{plane, square});
        _kpolygons_set.back().frozen();
        _kpolygons_set.back().is_bbox = true;
    }
    {
        auto plane = Plane_3{0, 1, 0, -scale};
        _kpolygons_set.emplace_back(Polygon_3{plane, square});
        _kpolygons_set.back().frozen();
        _kpolygons_set.back().is_bbox = true;
    }

    {
        auto plane = Plane_3{0, 0, 1, -scale};
        _kpolygons_set.emplace_back(Polygon_3{plane, square});
        _kpolygons_set.back().frozen();
        _kpolygons_set.back().is_bbox = true;
    }
}
