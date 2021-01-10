#include "kinetic.h"
#include <limits>

bool Kinetic_queue::update_certificate(const Event &event)
{

    auto &_this = *event.this_p;

    std::cout << "find index: id ";
    auto ind = _this.get_index(event.id).value();
    std::cout << event.id << " at _ids[" << ind << "]" << std::endl;

    auto line_2 = event.line_2;
    auto &points_2 = _this.points_2();
    auto &_speed = _this._speed;
    auto &_status = _this._status;
    auto &_ids = _this._ids;
    assert(ind >= 0);
    assert(ind < points_2.size());
    const auto &point_2 = points_2[ind];
    auto prev_ind = ind == 0 ? points_2.size() - 1 : ind - 1;
    auto next_ind = ind == points_2.size() - 1 ? 0 : ind + 1;

    if (_status[ind] == K_Polygon_3::Normal)
    {
        if (points_2[prev_ind] == points_2[ind])
        {
            std::cout << "vert collision" << std::endl;
            assert(_status[prev_ind] == K_Polygon_3::Sliding_Next);
            auto next_speed = _this.sliding_speed(ind, next_ind, line_2);
            _speed[ind] = next_speed;
            _status[ind] = K_Polygon_3::Sliding_Next;
            _ids[ind] = K_Polygon_3::next_id();
            erase_vert(_this, prev_ind);
            return true;
        }
        else if (points_2[ind] == points_2[next_ind])
        {
            std::cout << "vert collision" << std::endl;
            assert(_status[next_ind] == K_Polygon_3::Sliding_Pre);
            auto prev_speed = _this.sliding_speed(prev_ind, ind, line_2);
            _speed[ind] = prev_speed;
            _status[ind] = K_Polygon_3::Sliding_Pre;
            _ids[ind] = K_Polygon_3::next_id();
            erase_vert(_this, next_ind);
            return true;
        }
    }

    // auto dt = event.t - last_t;
    // assert(dt > 0);
    // auto &_other = *event.other_p;
    // if (!Polygon_3{_other.plane(), _other.move_dt(dt)}.has_on(event.point_3))
    //     return false;
    if (!event.other_p->has_on(event.point_3))
        return false;

    switch (_status[ind])
    {
    case K_Polygon_3::Normal: //type a
        if (Line_2{points_2[prev_ind], points_2[ind]} == line_2)
        {
            std::cout << "type a edge" << std::endl;
            auto next_speed = _this.sliding_speed(ind, next_ind, line_2);
            _speed[ind] = next_speed;
            _status[ind] = K_Polygon_3::Sliding_Next;
            _ids[ind] = K_Polygon_3::next_id();
        }
        else if (Line_2{points_2[ind], points_2[next_ind]} == line_2)
        {
            std::cout << "type a edge" << std::endl;
            auto prev_speed = _this.sliding_speed(prev_ind, ind, line_2);
            _speed[ind] = prev_speed;
            _status[ind] = K_Polygon_3::Sliding_Pre;
            _ids[ind] = K_Polygon_3::next_id();
        }
        else
        {
            std::cout << "type a" << std::endl;
            assert(_status[prev_ind] != K_Polygon_3::Frozen);
            assert(_status[next_ind] != K_Polygon_3::Frozen);
            auto prev_speed = _this.sliding_speed(prev_ind, ind, line_2);
            auto next_speed = _this.sliding_speed(ind, next_ind, line_2);
            assert(-prev_speed.direction() == next_speed.direction());

            _speed[ind] = next_speed;
            _status[ind] = K_Polygon_3::Sliding_Next;
            _ids[ind] = K_Polygon_3::next_id();

            _this._polygon_2.insert(_this._polygon_2.begin() + ind, point_2);
            _speed.insert(_speed.begin() + ind, prev_speed);
            _status.insert(_status.begin() + ind, K_Polygon_3::Sliding_Pre);
            _ids.insert(_ids.begin() + ind, K_Polygon_3::next_id());
        }
        break;
    case K_Polygon_3::Sliding_Next:
        if (points_2[next_ind] == points_2[ind])
        { //type c
            assert(_status[next_ind] == K_Polygon_3::Sliding_Pre);
            std::cout << "type c" << std::endl;
            _status[ind] = K_Polygon_3::Frozen;
            erase_vert(_this, next_ind);
        }
        else
        { //type b
            std::cout << "type b Sliding_Next" << std::endl;
            auto next_speed = _this.sliding_speed(ind, next_ind, line_2);
            assert(_speed[ind] != next_speed);
            _speed[ind] = next_speed;
            _status[ind] = K_Polygon_3::Sliding_Next;
            _ids[ind] = K_Polygon_3::next_id();
            _this.insert_frozen(ind, point_2);
        }
        break;
    case K_Polygon_3::Sliding_Pre:
        if (points_2[prev_ind] == points_2[ind])
        { //type c
            assert(_status[prev_ind] == K_Polygon_3::Sliding_Next);
            std::cout << "type c" << std::endl;
            _status[ind] = K_Polygon_3::Frozen;
            erase_vert(_this, prev_ind);
        }
        else
        { //type b
            std::cout << "type b Sliding_Pre" << std::endl;
            auto prev_speed = _this.sliding_speed(prev_ind, ind, line_2);
            assert(_speed[ind] != prev_speed);
            _speed[ind] = prev_speed;
            _status[ind] = K_Polygon_3::Sliding_Pre;
            _ids[ind] = K_Polygon_3::next_id();
            _this.insert_frozen(next_ind, point_2);
        }

        break;
    default:
        assert(false);
    }
    return true;
}

Kinetic_queue::Kinetic_queue(std::vector<K_Polygon_3> &k_polys_3) : k_polygons_3(k_polys_3)
{
    std::cout << "num of polygons " << k_polygons_3.size() << std::endl;
    for (auto &_this : k_polygons_3)
        for (auto &_other : k_polygons_3)
            collide(_this, _other);

    std::cout << "queue size " << queue.size() << std::endl;
}

void Kinetic_queue::collide(K_Polygon_3 &_this, K_Polygon_3 &_other, size_t old_id_max)
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
        if (_this._ids[i] > old_id_max) // id start from zero
            vert_collide(_this, _other, i, line_2);
    }
    if (line_polygon_intersect_2(line_2, _this.polygon_2()))
    //type d
    {
    }
}

void Kinetic_queue::vert_collide(K_Polygon_3 &_this, K_Polygon_3 &_other, size_t i, Line_2 &line_2)
{
    if (_this._status[i] == _this.Frozen)
        return;
    auto &point_2 = _this.points_2()[i];
    auto &speed = _this._speed[i];
    auto &id = _this._ids[i];

    auto r = Ray_2{point_2, speed};
    if (auto res = CGAL::intersection(r, line_2))
    {
        if (boost::get<Ray_2>(&*res)) //sliding
            return;
        if (auto point_2_p = boost::get<Point_2>(&*res))
        {
            if (*point_2_p == point_2) // that means the point is about to leave the plane
                return;
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
            auto point_3 = _this.plane().to_3d(*point_2_p);
            auto event = Event{&_this, &_other, id, point_3, line_2, t};
            id_events[id].push_back(event);
            insert(event);
        }
    }
}

void Kinetic_queue::erase_vert(K_Polygon_3 &k_poly_3, size_t ind)
{
    for (const auto &rm_event : id_events[k_poly_3._ids[ind]])
        remove(rm_event);
    // erase AFTER we update queue
    k_poly_3.erase(ind);
    k_poly_3.update_points_3();
}

FT Kinetic_queue::next_time()
{
    if (!queue.empty())
        return top().t;
    return INF;
}

FT Kinetic_queue::to_next_event()
{

    while (!queue.empty())
    {
        auto event = top();
        pop();
        auto dt = event.t - last_t;
        assert(dt >= 0);
        for (auto &k_poly : k_polygons_3)
            k_poly.update_nocheck(k_poly.move_dt(dt));
        assert(event.this_p->plane().has_on(event.point_3));
        assert(event.other_p->plane().has_on(event.point_3));

        auto old_id_max = K_Polygon_3::max_id(); //get old_id_max before update_certificate
        if (update_certificate(event)) //assume k_polygons_3 have already growed
        {
            for (const auto &rm_event : id_events[event.id])
                remove(rm_event);
            last_t = event.t; //update last_t before detect next collide()
            for (auto &_other : k_polygons_3)
                collide(*event.this_p, _other, old_id_max); // update queue only if id>old_id_max
            break;
        }
        else
        {
            last_t = event.t;
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

    auto scale = 0.1 + std::max(std::max({box.max(0), box.max(1), box.max(2)}),
                                std::abs(std::min({box.min(0), box.min(1), box.min(2)})));
    auto square = Points_2{};
    square.push_back(Point_2{scale + 0.1, scale + 0.1});
    square.push_back(Point_2{-scale - 0.1, scale + 0.1});
    square.push_back(Point_2{-scale - 0.1, -scale - 0.1});
    square.push_back(Point_2{scale + 0.1, -scale - 0.1});

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
