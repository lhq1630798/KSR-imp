#include "kinetic.h"
#include <limits>

std::vector<KP_Ref> Kinetic_queue::update_certificate(const Event &event)
{

    auto kline = event.kline;
    const auto &line_2 = kline->_line_2;
    auto kp = event.kp;
    auto poly = kp->face;
    auto prev_kp = poly->prev(kp);
    auto next_kp = poly->next(kp);

    assert(line_2.has_on(kp->point()));

    if (kp->_status == Mode::Normal)
    {
        if (prev_kp->point() == kp->point())
        {
            std::cout << "vert collision" << std::endl;
            assert(prev_kp->_status == Mode::Sliding);
            auto next_speed = KPolygon_2::Edge{kp, next_kp}.sliding_speed(line_2);
            kp->sliding(next_speed);
            erase_kp(prev_kp);
            return {kp};
        }
        else if (kp->point() == next_kp->point())
        {
            std::cout << "vert collision" << std::endl;
            assert(next_kp->_status == Mode::Sliding);
            auto prev_speed = KPolygon_2::Edge{prev_kp, kp}.sliding_speed(line_2);
            kp->sliding(prev_speed);
            erase_kp(next_kp);
            return {kp};
        }
    }

    // if (!kline->has_on(kp->point()))
    //     return {};

    switch (kp->_status)
    {
    case Mode::Normal: //type a
        if (Line_2{prev_kp->point(), kp->point()} == line_2)
        {
            std::cout << "type a edge" << std::endl;
            auto next_speed = KPolygon_2::Edge{kp, next_kp}.sliding_speed(line_2);
            kp->sliding(next_speed);
            return {kp};
        }
        else if (Line_2{kp->point(), next_kp->point()} == line_2)
        {
            std::cout << "type a edge" << std::endl;
            auto prev_speed = KPolygon_2::Edge{prev_kp, kp}.sliding_speed(line_2);
            kp->sliding(prev_speed);
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

            kp->sliding(next_speed);
            auto new_kp = poly->insert_KP(kp, KPoint_2{kp->point(), prev_speed, Mode::Sliding});
            return {kp, new_kp};
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
                kp->sliding(next_speed);
                poly->insert_KP(kp, KPoint_2{kp->point(), CGAL::NULL_VECTOR, Mode::Frozen});
            }
            else
            {
                auto prev_speed = KPolygon_2::Edge{prev_kp, kp}.sliding_speed(line_2);
                assert(prev_speed != CGAL::NULL_VECTOR);
                std::cout << "type b Sliding_Prev" << std::endl;
                kp->sliding(prev_speed);
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
            for (auto kp = kpoly_2.kpoints_2().begin(); kp != kpoly_2.kpoints_2().end(); kp++)
                kp_collide(kp);

    std::cout << "queue size " << queue.size() << std::endl;
}

void Kinetic_queue::kp_collide(KP_Ref kp)
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
void Kinetic_queue::erase_kp(KP_Ref kp)
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

KPolygons_SET::KPolygons_SET(const Polygons_3 &polygons_3) : _kpolygons_set(polygons_3.begin(), polygons_3.end())
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

bool KPolygons_2::try_split(KPoly_Ref kpoly_2, const KLine_2 &kline_2)
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

    auto &new_kp1 = p_e[0].first;
    auto &new_kp2 = p_e[1].first;
    auto &e1 = p_e[0].second;
    auto &e2 = p_e[1].second;

    auto [p1t, sp1t] = kline_2.transform2twin(new_kp1);
    auto [p2t, sp2t] = kline_2.transform2twin(new_kp2);

    auto segRef = kline_2.twin->insert_seg(KSegment{p1t, p2t, sp1t, sp2t});

    KPoly_Ref poly1 = insert_kpoly_2(KPolygon_2{}), poly2 = insert_kpoly_2(KPolygon_2{});
    KPoly_Ref poly_p = poly1;
    for (const auto &edge : edges)
    {
        poly_p->insert_KP(*edge.kp1);
        if (edge.kp1 == e1.kp1)
        {
            assert(poly_p == poly1);
            auto kpRef = poly_p->insert_KP(new_kp1);
            kpRef->twin_speed = &segRef->speed1;
            poly_p = poly2; //switch
            kpRef = poly_p->insert_KP(new_kp1);
            kpRef->twin_speed = &segRef->speed1;
        }
        else if (edge.kp1 == e2.kp1)
        {
            assert(poly_p == poly2);
            auto kpRef = poly_p->insert_KP(new_kp2);
            kpRef->twin_speed = &segRef->speed2;
            poly_p = poly1; //switch
            kpRef = poly_p->insert_KP(new_kp2);
            kpRef->twin_speed = &segRef->speed2;
        }
    }
    assert((poly1->size() + poly2->size()) == (kpoly_2->size() + 4));
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

void add_bounding_box(KPolygons_SET &KPolygons_set)
{
}
