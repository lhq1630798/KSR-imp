#include "kinetic.h"
#include <limits>

std::vector<KP_Circ> Kinetic_queue::update_certificate(const Event &event)
{

    auto kline = event.kline;
    const auto &line_2 = kline->_line_2;
    auto kp = event.kp;
    auto poly = kp->face;
    auto prev_kp = prev(kp);
    auto next_kp = next(kp);

    assert(line_2.has_on(kp->point()));

    if (kp->_status == Mode::Normal)
    {
        if (prev_kp->point() == kp->point())
        {
            std::cout << "vert collision" << std::endl;
            auto sliding_kp = prev_kp;
            assert(sliding_kp->_status == Mode::Sliding);
            auto sliding_twin = sliding_kp->twin;
            bool has_twin = (sliding_twin != nullptr);
            auto kpoint = KPoint_2{kp->point(), kp->_speed, Mode::Normal};
            auto new_speed = KPolygon_2::Edge{kp, next_kp}.sliding_speed(line_2);
            sliding_kp->sliding_speed(new_speed);
            erase_kp(kp);
            if (!has_twin)
                return {sliding_kp};
            auto crossed_kp = sliding_twin->face->insert_KP(sliding_twin, kpoint);
            sliding_twin->sliding_speed(new_speed);
            return {sliding_kp, sliding_twin, crossed_kp};
        }
        else if (kp->point() == next_kp->point())
        {
            std::cout << "vert collision" << std::endl;
            auto sliding_kp = next_kp;
            assert(sliding_kp->_status == Mode::Sliding);
            auto sliding_twin = sliding_kp->twin;
            bool has_twin = (sliding_twin != nullptr);
            auto kpoint = KPoint_2{kp->point(), kp->_speed, Mode::Normal};
            auto new_speed = KPolygon_2::Edge{prev_kp, kp}.sliding_speed(line_2);
            sliding_kp->sliding_speed(new_speed);
            erase_kp(kp);
            if (!has_twin)
                return {sliding_kp};
            auto crossed_kp = sliding_twin->face->insert_KP(next(sliding_twin), kpoint);
            sliding_twin->sliding_speed(new_speed);
            return {sliding_kp, sliding_twin, crossed_kp};
        }
    }

    // if (!kline->has_on(kp->point()))
    //     return {};

    switch (kp->_status)
    {
    case Mode::Normal: //type a
        if (line_2.has_on(prev_kp->point()) && line_2.has_on(kp->point()))
        {
            assert(false);
            std::cout << "type a edge" << std::endl;
            auto next_speed = KPolygon_2::Edge{kp, next_kp}.sliding_speed(line_2);
            kp->sliding_speed(next_speed);
            return {kp};
        }
        else if (line_2.has_on(next_kp->point()) && line_2.has_on(kp->point()))
        {
            assert(false);
            std::cout << "type a edge" << std::endl;
            auto prev_speed = KPolygon_2::Edge{prev_kp, kp}.sliding_speed(line_2);
            kp->sliding_speed(prev_speed);
            return {kp};
        }
        else
        {
            std::cout << "type a" << std::endl;
            assert(prev_kp->_status != Mode::Frozen);
            assert(next_kp->_status != Mode::Frozen);
            auto prev_speed = KPolygon_2::Edge{prev_kp, kp}.sliding_speed(line_2);
            auto next_speed = KPolygon_2::Edge{kp, next_kp}.sliding_speed(line_2);
            assert(-prev_speed.direction() == next_speed.direction());

            auto sliding_prev = poly->insert_KP(kp, KPoint_2{kp->point(), prev_speed, Mode::Sliding});
            auto sliding_next = poly->insert_KP(kp, KPoint_2{kp->point(), next_speed, Mode::Sliding});

            kline->add_seg_twin(sliding_prev, sliding_next);

            erase_kp(kp);
            return {prev_kp, next_kp};
        }
        break;
    case Mode::Sliding:
        if (next_kp->point() == kp->point())
        { //type c
            assert(next_kp->_status == Mode::Sliding);
            std::cout << "type c" << std::endl;
            kp->frozen();
            erase_kp(next_kp);
            return {kp};
        }
        else if (prev_kp->point() == kp->point())
        { //type c
            assert(prev_kp->_status == Mode::Sliding);
            std::cout << "type c" << std::endl;
            kp->frozen();
            erase_kp(prev_kp);
            return {kp};
        }
        else
        { //type b
            auto next_speed = KPolygon_2::Edge{kp, next_kp}.sliding_speed(line_2);
            if (next_speed != CGAL::NULL_VECTOR)
            {
                std::cout << "type b Sliding_Next" << std::endl;
                kp->sliding_speed(next_speed);
                poly->insert_KP(kp, KPoint_2{kp->point(), CGAL::NULL_VECTOR, Mode::Frozen});
            }
            else
            {
                auto prev_speed = KPolygon_2::Edge{prev_kp, kp}.sliding_speed(line_2);
                assert(prev_speed != CGAL::NULL_VECTOR);
                std::cout << "type b Sliding_Prev" << std::endl;
                kp->sliding_speed(prev_speed);
                poly->insert_KP(next_kp, KPoint_2{kp->point(), CGAL::NULL_VECTOR, Mode::Frozen});
            }
            return {kp};
        }
        break;
    default:
        assert(false);
    }
    return {};
}

Kinetic_queue::Kinetic_queue(KPolygons_SET &kpolygons_set) : kpolygons_set(kpolygons_set)
{
    std::cout << "num of polygons set " << kpolygons_set.size() << std::endl;
    for (auto &kpolys_2 : kpolygons_set._kpolygons_set)
        for (auto &kpoly_2 : kpolys_2._kpolygons_2)
        {
            auto kp = kpoly_2.kp_circulator(), end = kp;
            CGAL_For_all(kp, end)
                kp_collide(kp);
        }

    std::cout << "queue size " << queue.size() << std::endl;
}

void Kinetic_queue::kp_collide(KP_Circ kp)
{
    if (kp->_status == Mode::Frozen)
        return;
    assert(kp->_speed != CGAL::NULL_VECTOR);
    auto kpolys_2 = kp->face->parent;
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

void Kinetic_queue::erase_kp(KP_Circ kp)
{
    remove_events(kp);
    // erase AFTER we update queue
    kp->face->erase(kp);
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

        auto need_update = update_certificate(event); //assume kpolygons_set have already growed
        last_t = event.t;                             //update last_t before detect next collide()
        for (auto kp : need_update)
        {
            remove_events(kp);
            kp_collide(kp);
        }
    }
    return last_t;
}

size_t next_id;

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

KPolygons_SET::KPolygons_SET(const Polygons_3 &polygons_3) : _kpolygons_set(polygons_3.begin(), polygons_3.end())
{
    add_bounding_box(polygons_3);
    for (auto polys_i = _kpolygons_set.begin(); polys_i != _kpolygons_set.end(); polys_i++)
        for (auto polys_j = std::next(polys_i); polys_j != _kpolygons_set.end(); polys_j++)
            if (auto res = CGAL::intersection(polys_i->plane(), polys_j->plane()))
                if (auto line_3 = boost::get<Line_3>(&*res))
                {
                    auto line_i = polys_i->insert_kline(polys_i->project_2(*line_3));
                    auto line_j = polys_j->insert_kline(polys_j->project_2(*line_3));
                    line_i->twin = line_j;
                    line_j->twin = line_i;
                }

    for (auto &kpolys_2 : _kpolygons_set)
    {
        // split
        for (auto &kline_2 : kpolys_2.klines())
        {
            auto max_id = next_id;
            auto kpoly_2 = kpolys_2._kpolygons_2.begin();
            while (kpoly_2 != kpolys_2._kpolygons_2.end())
            {
                if (kpoly_2->id >= max_id)
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
            else
                assert(false);
        }
    }
    if (p_e.empty())
        return false;
    assert(p_e.size() == 2);

    const auto &[new_kp1, e1] = p_e[0];
    const auto &[new_kp2, e2] = p_e[1];

    KPoly_Ref new_poly1 = insert_kpoly_2(KPolygon_2{}), new_poly2 = insert_kpoly_2(KPolygon_2{});
    
    // poly1
    auto sliding_next1 = new_poly1->insert_KP(new_kp1);
    for (auto kp = e1.kp2; kp != e2.kp2; kp++)
        new_poly1->insert_KP(std::move(*kp));
    auto sliding_prev1 = new_poly1->insert_KP(new_kp2);
    kline_2.add_seg_twin(sliding_prev1, sliding_next1);
    // poly2
    auto sliding_next2 = new_poly2->insert_KP(new_kp2);
    for (auto kp = e2.kp2; kp != e1.kp2; kp++)
        new_poly2->insert_KP(std::move(*kp));
    auto sliding_prev2 = new_poly2->insert_KP(new_kp1);
    kline_2.add_seg_twin(sliding_prev2, sliding_next2);
    // pair the sliding kp
    sliding_prev1->twin = sliding_next2;
    sliding_next2->twin = sliding_prev1;
    sliding_next1->twin = sliding_prev2;
    sliding_prev2->twin = sliding_next1;

    assert((new_poly1->size() + new_poly2->size()) == (kpoly_2->size() + 4));
    return true;
}

// KP_Circ &KP_Circ::operator++()
// {
//     kp = kpoly_2->next(kp);
//     return *this;
// }

// KP_Circ &KP_Circ::operator--()
// {
//     kp = kpoly_2->prev(kp);
//     return *this;
// }

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

    auto scale = 0.1 + std::max(std::max({box.max(0), box.max(1), box.max(2)}),
                                std::abs(std::min({box.min(0), box.min(1), box.min(2)})));
    auto square = Points_2{};
    square.push_back(Point_2{scale + 0.1, scale + 0.1});
    square.push_back(Point_2{-scale - 0.1, scale + 0.1});
    square.push_back(Point_2{-scale - 0.1, -scale - 0.1});
    square.push_back(Point_2{scale + 0.1, -scale - 0.1});

    {
        auto plane = Plane_3{1, 0, 0, scale};
        _kpolygons_set.emplace_back(Polygon_3{plane, square});
        _kpolygons_set.back()._kpolygons_2.back().frozen();
    }
    {
        auto plane = Plane_3{1, 0, 0, -scale};
        _kpolygons_set.emplace_back(Polygon_3{plane, square});
        _kpolygons_set.back()._kpolygons_2.back().frozen();
    }
    {
        auto plane = Plane_3{0, 1, 0, scale};
        _kpolygons_set.emplace_back(Polygon_3{plane, square});
        _kpolygons_set.back()._kpolygons_2.back().frozen();
    }
    {
        auto plane = Plane_3{0, 1, 0, -scale};
        _kpolygons_set.emplace_back(Polygon_3{plane, square});
        _kpolygons_set.back()._kpolygons_2.back().frozen();
    }
    {
        auto plane = Plane_3{0, 0, 1, scale};
        _kpolygons_set.emplace_back(Polygon_3{plane, square});
        _kpolygons_set.back()._kpolygons_2.back().frozen();
    }
    {
        auto plane = Plane_3{0, 0, 1, -scale};
        _kpolygons_set.emplace_back(Polygon_3{plane, square});
        _kpolygons_set.back()._kpolygons_2.back().frozen();
    }
}
