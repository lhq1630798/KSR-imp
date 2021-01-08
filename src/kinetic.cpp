#include "kinetic.h"
#include <limits>

void K_Polygon_3::update_certificate(size_t ind, const Line_2 line_2, Kinetic_queue &queue)
{
    assert(ind >= 0);
    assert(ind < points_2().size());
    const auto &point_2 = points_2()[ind];
    auto prev_ind = ind == 0 ? points_2().size() - 1 : ind - 1;
    auto next_ind = ind == points_2().size() - 1 ? 0 : ind + 1;
    assert(line_2.has_on(point_2));

    switch (_status[ind])
    {
    case Normal: //type a
        if (points_2()[prev_ind] == points_2()[ind])
        {
            std::cout << "type a half" << std::endl;
            assert(_status[prev_ind] == Sliding_Next);
            auto next_speed = sliding_speed(ind, next_ind, line_2);
            _speed[ind] = next_speed;
            _status[ind] = Sliding_Next;
            _ids[ind] = next_id();
            queue.erase_vert(*this, prev_ind);
        }
        else if (points_2()[ind] == points_2()[next_ind])
        {
            std::cout << "type a half" << std::endl;
            assert(_status[next_ind] == Sliding_Pre);
            auto prev_speed = sliding_speed(prev_ind, ind, line_2);
            _speed[ind] = prev_speed;
            _status[ind] = Sliding_Pre;
            _ids[ind] = next_id();
            queue.erase_vert(*this, next_ind);
        }
        else if (Line_2{points_2()[prev_ind], points_2()[ind]} == line_2)
        {
            std::cout << "type a edge" << std::endl;
            auto next_speed = sliding_speed(ind, next_ind, line_2);
            _speed[ind] = next_speed;
            _status[ind] = Sliding_Next;
            _ids[ind] = next_id();
        }
        else if (Line_2{points_2()[ind], points_2()[next_ind]} == line_2)
        {
            std::cout << "type a edge" << std::endl;
            auto prev_speed = sliding_speed(prev_ind, ind, line_2);
            _speed[ind] = prev_speed;
            _status[ind] = Sliding_Pre;
            _ids[ind] = next_id();
        }
        else
        {
            std::cout << "type a" << std::endl;
            assert(_status[prev_ind] != Frozen);
            assert(_status[next_ind] != Frozen);
            auto prev_speed = sliding_speed(prev_ind, ind, line_2);
            auto next_speed = sliding_speed(ind, next_ind, line_2);
            assert(-prev_speed.direction() == next_speed.direction());

            _speed[ind] = next_speed;
            _status[ind] = Sliding_Next;
            _ids[ind] = next_id();

            _polygon_2.insert(_polygon_2.begin() + ind, point_2);
            _speed.insert(_speed.begin() + ind, prev_speed);
            _status.insert(_status.begin() + ind, Sliding_Pre);
            _ids.insert(_ids.begin() + ind, next_id());
        }
        break;
    case Sliding_Next:
        if (points_2()[next_ind] == points_2()[ind])
        { //type c
            assert(_status[next_ind] == Sliding_Pre);
            std::cout << "type c" << std::endl;
            _status[ind] = Frozen;
            queue.erase_vert(*this, next_ind);
        }
        else
        { //type b
            std::cout << "type b Sliding_Next" << std::endl;
            auto next_speed = sliding_speed(ind, next_ind, line_2);
            _speed[ind] = next_speed;
            _status[ind] = Sliding_Next;
            _ids[ind] = next_id();
            insert_frozen(ind, point_2);
        }
        break;
    case Sliding_Pre:
        if (points_2()[prev_ind] == points_2()[ind])
        { //type c
            assert(_status[prev_ind] == Sliding_Next);
            std::cout << "type c" << std::endl;
            _status[ind] = Frozen;
            queue.erase_vert(*this, prev_ind);
        }
        else
        { //type b
            std::cout << "type b Sliding_Pre" << std::endl;
            auto prev_speed = sliding_speed(prev_ind, ind, line_2);
            _speed[ind] = prev_speed;
            _status[ind] = Sliding_Pre;
            // _ids[ind] = next_id();
            insert_frozen(next_ind, point_2);
        }

        break;
    default:
        assert(false);
    }
    update_points_3();
    return;
}

Kinetic_queue::Kinetic_queue(std::vector<K_Polygon_3> &k_polys_3) : k_polygons_3(k_polys_3)
{
    std::cout << "num of polygons " << k_polygons_3.size() << std::endl;
    for (auto &_this : k_polygons_3)
        for (auto &_other : k_polygons_3)
            collide(_this, _other);

    std::cout << "queue size " << queue.size() << std::endl;
}

void Kinetic_queue::collide(K_Polygon_3 &_this, K_Polygon_3 &_other)
{
    if (_this.plane() == _other.plane())
        return;
    auto result = CGAL::intersection(_this.plane(), _other.plane());
    if (!result)
        return;
    auto line_3 = *boost::get<Line_3>(&*result);

    // project to _this polygon
    auto line_2 = _this.project_2(line_3);

    //type a, b, c
    for (size_t i = 0; i < _this.points_2().size(); i++)
    {
        if (_this._status[i] == _this.Frozen)
            continue;
        auto &point_2 = _this.points_2()[i];
        auto &speed = _this._speed[i];
        auto &id = _this._ids[i];

        auto r = Ray_2{point_2, speed};
        assert(r.opposite().has_on(point_2));
        if (auto res = CGAL::intersection(r, line_2))
        {
            if (boost::get<Ray_2>(&*res)) //sliding
                continue;
            if (auto point_2_p = boost::get<Point_2>(&*res))
            {
                assert(*point_2_p != point_2);
                auto diff = *point_2_p - point_2;
                auto t = last_t;
                if (speed.x() != 0)
                {
                    t += diff.x() / speed.x();
                }
                else
                {
                    assert(speed.y() != 0);
                    t += diff.y() / speed.y();
                }
                // if(diff.squared_length() == 0){
                //     std::cout << "Simultaneous collisions" <<std::endl;
                //     continue;
                // }
                auto point_3 = _this.plane().to_3d(*point_2_p);
                auto event = Event{&_this, &_other, id, point_3, line_2, t};
                id_events[id].push_back(event);
                insert(event);
            }
        }
    }
    if (line_polygon_intersect_2(line_2, _this.polygon_2()))
    //type d
    {
    }
}

void Kinetic_queue::erase_vert(K_Polygon_3 &k_poly_3, size_t ind)
{
    for (const auto &rm_event : id_events[k_poly_3._ids[ind]])
        remove(rm_event);
    // erase AFTER we update queue
    k_poly_3.erase(ind);
}

void Kinetic_queue::kinetic_check(K_Polygon_3 &k_poly_3)
{
    auto &poly_2 = k_poly_3.polygon_2();
    while (!poly_2.is_simple())
    {
        for (size_t ind = 0; ind < poly_2.size(); ind++)
        {
            auto next = ind + 1;
            if (next == poly_2.size())
                next = 0;
            // type c or type a edge
            if (poly_2[ind] == poly_2[next])
            {
                assert(k_poly_3._status[ind] == k_poly_3.Sliding_Next ||
                       k_poly_3._status[next] == k_poly_3.Sliding_Pre);
                if (k_poly_3._status[ind] == k_poly_3.Normal)
                {
                    std::cout << "type a edge" << std::endl;
                    // k_poly_3._status[ind] = k_poly_3.Sliding_Pre;
                    erase_vert(k_poly_3, next);
                }
                else if (k_poly_3._status[next] == k_poly_3.Normal)
                {
                    std::cout << "type a edge" << std::endl;
                    // k_poly_3._status[next] = k_poly_3.Sliding_Pre;
                    erase_vert(k_poly_3, ind);
                }
                else
                {
                    std::cout << "type c" << std::endl;
                    assert(k_poly_3._status[ind] == k_poly_3.Sliding_Next &&
                           k_poly_3._status[next] == k_poly_3.Sliding_Pre);
                    k_poly_3._status[ind] = k_poly_3.Frozen;
                    for (const auto &rm_event : id_events[k_poly_3._ids[ind]])
                        remove(rm_event);
                    erase_vert(k_poly_3, next);
                }

                continue;
            }
        }
    }
}

FT Kinetic_queue::next_event()
{

    while (!queue.empty())
    {
        auto event = top();
        pop();
        auto dt = event.t - last_t;
        assert(dt >= 0);
        auto &_this = *event.this_p, _other = *event.other_p;
        assert(_this.plane().has_on(event.point_3));
        assert(_other.plane().has_on(event.point_3));
        if (Polygon_3{_other.plane(), _other.move_dt(dt)}.has_on(event.point_3))
        {
            for (const auto &rm_event : id_events[event.id])
                remove(rm_event);
            if (dt != 0)
                for (auto &k_poly : k_polygons_3)
                    k_poly.update_nocheck(k_poly.move_dt(dt));

            // split point
            std::cout << "find index: id ";
            for (size_t ind = 0; ind < _this.points_2().size(); ind++)
            {
                if (_this._ids[ind] == event.id)
                {
                    std::cout << event.id << " at _ids[" << ind << "]" << std::endl;
                    _this.update_certificate(ind, event.line_2, *this);
                }
            }

            last_t = event.t; //before next collide
            for (auto &_other : k_polygons_3)
                collide(_this, _other);

            break;
        }
    }
    return last_t;
}

void add_bounding_box(std::vector<K_Polygon_3> &k_polys_3)
{
    auto all_point_3 = Points_3{};
    auto box = CGAL::Bbox_3{};
    for (const auto &polys_3 : k_polys_3)
        box += CGAL::bbox_3(polys_3.points_3().begin(), polys_3.points_3().end());

    auto scale = 1 + std::max(std::max({box.max(0), box.max(1), box.max(2)}),
                              std::abs(std::min({box.min(0), box.min(1), box.min(2)})));
    auto square = Points_2{};
    square.push_back(Point_2{scale, scale});
    square.push_back(Point_2{-scale, scale});
    square.push_back(Point_2{-scale, -scale});
    square.push_back(Point_2{scale, -scale});

    {
        auto plane = Plane_3{1, 0, 0, scale};
        auto k_poy_3 = K_Polygon_3{Polygon_3{plane, square}};
        k_poy_3.freeze();
        k_polys_3.push_back(std::move(k_poy_3));
    }
    {
        auto plane = Plane_3{1, 0, 0, -scale};
        auto k_poy_3 = K_Polygon_3{Polygon_3{plane, square}};
        k_poy_3.freeze();
        k_polys_3.push_back(std::move(k_poy_3));
    }
    {
        auto plane = Plane_3{0, 1, 0, scale};
        auto k_poy_3 = K_Polygon_3{Polygon_3{plane, square}};
        k_poy_3.freeze();
        k_polys_3.push_back(std::move(k_poy_3));
    }
    {
        auto plane = Plane_3{0, 1, 0, -scale};
        auto k_poy_3 = K_Polygon_3{Polygon_3{plane, square}};
        k_poy_3.freeze();
        k_polys_3.push_back(std::move(k_poy_3));
    }
    {
        auto plane = Plane_3{0, 0, 1, scale};
        auto k_poy_3 = K_Polygon_3{Polygon_3{plane, square}};
        k_poy_3.freeze();
        k_polys_3.push_back(std::move(k_poy_3));
    }
    {
        auto plane = Plane_3{0, 0, 1, -scale};
        auto k_poy_3 = K_Polygon_3{Polygon_3{plane, square}};
        k_poy_3.freeze();
        k_polys_3.push_back(std::move(k_poy_3));
    }
}

