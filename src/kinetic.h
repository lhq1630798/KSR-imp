#pragma once
#include <queue>
#include "cgal_object.h"

static const auto INF = FT{99999};

class K_Polygon_3 : public Polygon_3
{
    enum mode
    {
        frozen,
        sliding,
        normal
    };
    std::vector<mode> _status;
    inline static size_t _next_id = 0;
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
            _status.push_back(normal);
            _ids.push_back(next_id());
        }
    }
    size_t next_id() {return _next_id++;}
    void update(Polygon_2 poly_2)
    {
        assert(poly_2.size() == _polygon_2.size());
        _polygon_2 = std::move(poly_2);
        update_points_3();
    }
    Polygon_2 move_dt(FT t)
    {
        auto new_points_2 = Points_2{};
        for (size_t i = 0; i < points_2().size(); i++)
            new_points_2.push_back(points_2()[i] + _speed[i] * t);
        return Polygon_2{new_points_2.begin(), new_points_2.end()};
    }
    Vector_2 sliding_speed(size_t ind1,size_t ind2, const Line_2 line_2){
        if(auto res = CGAL::intersection(Line_2{points_2()[ind1], points_2()[ind2]}, line_2)){
            if (auto cur_point = boost::get<Point_2>(&*res)){
                const auto& poly_dt = move_dt(1); 
                if(auto res_dt = CGAL::intersection(Line_2{poly_dt.container()[ind1], poly_dt.container()[ind2]}, line_2)){
                    if (auto point_dt = boost::get<Point_2>(&*res)){
                        return *point_dt - *cur_point;
                    }
                }
            }

        }
        assert(false);
        return Vector_2{};
    }
    void insert_sliding(size_t ind, const Line_2 line_2){
        assert(ind>=0);
        assert(ind<points_2().size());
        assert(_status[ind] == normal);
        //type a
        const auto &point_2 = points_2()[ind];
        auto prev_ind = ind == 0 ? points_2().size()-1 : ind-1;
        auto next_ind = ind == points_2().size()-1 ? 0 : ind+1;
        auto prev_speed = sliding_speed(prev_ind, ind, line_2);
        auto next_speed = sliding_speed(ind, next_ind, line_2);
        assert(-prev_speed.direction() == next_speed.direction());

        _speed[ind] = next_speed;
        _status[ind] = sliding;
        _ids[ind] = next_id();

        _polygon_2.container().insert(points_2().begin()+ind, point_2);
        _speed.insert(_speed.begin()+ind, prev_speed);
        _status.insert(_status.begin()+ind, sliding);
        _ids.insert(_ids.begin()+ind, next_id());

        update_points_3();
    }
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
        : this_p(_this), other_p(_other), id(id), point_3(p_3), line_2(line_2), t(t) {}
};

/* Comparison operator for Event so std::set will properly order them */
inline bool operator<(const Event &r1, const Event &r2) {
    if (r1.t != r2.t) {
        return r1.t < r2.t;
    }
    // todo: what if (r1.point_3 == r2.point_3) ?
    return (r1.point_3-CGAL::ORIGIN).squared_length() < (r2.point_3-CGAL::ORIGIN).squared_length();
}


class Kinetic_queue
{ 

public:

    Kinetic_queue(std::vector<K_Polygon_3> &k_polys_3);
    FT next_event();
    size_t size() { return queue.size(); }

private:
    void insert(const Event &event) { queue.insert(event); }
    void remove(const Event &event) {
        if (queue.find(event) != queue.end()) {
            queue.erase(event);
        }
    }
    const Event &top(void) const { return *(queue.begin()); }
    void pop(void) { queue.erase(queue.begin()); }

    std::vector<K_Polygon_3> &k_polygons_3;
    std::set<Event> queue;
    // std::unordered_map<CGAL::Handle::Id_type, std::vector<Event>> id_events;
    std::unordered_map<size_t, std::vector<Event>> id_events;
    void collide(K_Polygon_3 &_this, K_Polygon_3 &_other);
    FT last_t = 0;
};

