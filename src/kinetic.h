#pragma once
#include <queue>
#include "cgal_object.h"

static const auto INF = FT{99999};

class Kinetic_queue;

class K_Polygon_3 : public Polygon_3
{
public:
    std::vector<Vector_2> _speed;
    std::vector<size_t> _ids;
    Point_2 _center;
    explicit K_Polygon_3(Polygon_3 poly_3) : Polygon_3(poly_3)
    {
        auto center = Vector_2{};
        for (const auto &point_2 : poly_3.points_2())
            center += point_2 - CGAL::ORIGIN;
        _center = CGAL::ORIGIN + center * (1 / poly_3.points_2().size());
        for (const auto &point_2 : poly_3.points_2())
        {
            _speed.push_back(point_2 - _center);
            _status.push_back(Normal);
            _ids.push_back(next_id());
        }
    }
    void freeze()
    {
        for (size_t i = 0; i < points_2().size(); i++)
        {
            _speed[i] = Vector_2{0, 0};
            _status[i] = Frozen;
        }
    }
    size_t next_id() { return _next_id++; }
    K_Polygon_3 &update(Polygon_2 poly_2)
    {
        assert(poly_2.is_simple());
        return update_nocheck(std::move(poly_2));
    }

    Polygon_2 move_dt(FT dt)
    {
        assert(dt >= 0);
        if (dt == 0)
        {
            std::cout << "zero dt" << std::endl;
            return polygon_2();
        }
        auto new_points_2 = Points_2{};
        for (size_t i = 0; i < points_2().size(); i++)
        {
            auto move_dt = _status[i] == Frozen ? FT{0} : dt;
            new_points_2.push_back(points_2()[i] + _speed[i] * move_dt);
        }
        auto polygon_2 = Polygon_2{new_points_2.begin(), new_points_2.end()};
        assert(polygon_2.is_convex());

        return polygon_2;
    }

private:
    Vector_2 sliding_speed(size_t ind1, size_t ind2, const Line_2 line_2)
    {
        if (auto res = CGAL::intersection(Line_2{points_2()[ind1], points_2()[ind2]}, line_2))
        {
            if (auto cur_point = boost::get<Point_2>(&*res))
            {
                const auto &poly_dt = move_dt(1);
                if (auto res_dt = CGAL::intersection(Line_2{poly_dt.container()[ind1], poly_dt.container()[ind2]}, line_2))
                {
                    if (auto point_dt = boost::get<Point_2>(&*res_dt))
                    {
                        auto speed = *point_dt - *cur_point;
                        assert(speed.squared_length() > 0);
                        assert(line_2.direction() == speed.direction() ||
                               -line_2.direction() == speed.direction());
                        return speed;
                    }
                }
            }
        }
        assert(false);
        return Vector_2{};
    }
    void update_certificate(size_t ind, const Line_2 line_2, Kinetic_queue &);
    K_Polygon_3 &update_nocheck(Polygon_2 poly_2)
    {
        assert(poly_2.size() == _polygon_2.size());
        _polygon_2 = std::move(poly_2);
        update_points_3();
        return *this;
    }
    void erase(size_t ind)
    {
        _status.erase(_status.begin() + ind);
        _speed.erase(_speed.begin() + ind);
        _ids.erase(_ids.begin() + ind);
        _polygon_2.erase(_polygon_2.begin() + ind);
        update_points_3();
    }
    void insert_frozen(size_t ind, Point_2 point_2)
    {
        _status.insert(_status.begin() + ind, Frozen);
        _speed.insert(_speed.begin() + ind, Vector_2{0, 0});
        _ids.insert(_ids.begin() + ind, next_id());
        _polygon_2.insert(_polygon_2.begin() + ind, point_2);
        update_points_3();
    }
    enum mode
    {
        Frozen,
        Sliding_Pre,  //to previous index
        Sliding_Next, //to next index
        Normal
    };
    std::vector<mode> _status;
    inline static size_t _next_id = 0;
    friend Kinetic_queue;
};

class Event
{
public:
    FT t;
    K_Polygon_3 *this_p;
    K_Polygon_3 *other_p;
    size_t id;
    Point_3 point_3;
    Line_2 line_2;
    Event(K_Polygon_3 *_this, K_Polygon_3 *_other, size_t id, Point_3 p_3, Line_2 line_2, FT t)
        : this_p(_this), other_p(_other), id(id), point_3(p_3), line_2(line_2), t(t)
    {
        assert(t >= 0);
    }
};

/* Comparison operator for Event so std::set will properly order them */
inline bool operator<(const Event &r1, const Event &r2)
{
    if (r1.t != r2.t)
    {
        return r1.t < r2.t;
    }
    // very likely (r1.point_3 == r2.point_3)
    assert(r1.point_3 == r2.point_3);
    return r1.point_3.id() < r2.point_3.id();
}

class Kinetic_queue
{

public:
    Kinetic_queue(std::vector<K_Polygon_3> &k_polys_3);
    FT next_event();
    size_t size() { return queue.size(); }

private:
    void insert(const Event &event) { queue.insert(event); }
    void remove(const Event &event)
    {
        if (queue.find(event) != queue.end())
        {
            queue.erase(event);
        }
    }
    const Event &top(void) const { return *(queue.begin()); }
    void pop(void) { queue.erase(queue.begin()); }

    void check_update(K_Polygon_3 &, Polygon_2 &&);
    void erase_vert(K_Polygon_3 &, size_t ind);
    std::vector<K_Polygon_3> &k_polygons_3;
    std::set<Event> queue;
    // std::unordered_map<CGAL::Handle::Id_type, std::vector<Event>> id_events;
    std::unordered_map<size_t, std::vector<Event>> id_events;
    void collide(K_Polygon_3 &_this, K_Polygon_3 &_other);
    void Kinetic_queue::kinetic_check(K_Polygon_3 &k_poly_3);
    FT last_t = 0;
    friend K_Polygon_3;
};

void add_bounding_box(std::vector<K_Polygon_3> &k_polys_3);
void inf_perturb(Polygons_3 &polygons_3);