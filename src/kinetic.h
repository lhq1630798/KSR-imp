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

public:
    std::vector<Vector_2> _speed;
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
        }
    }
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
};

class Event
{
public:
    FT t;
    K_Polygon_3 *this_p;
    K_Polygon_3 *other_p;
    Point_3 point_3;
    Event(K_Polygon_3 *_this, K_Polygon_3 *_other, Point_3 p_3, FT t)
        : this_p(_this), other_p(_other), point_3(p_3), t(t) {}
};

inline static auto cmp = [](const Event &_this, const Event &_other) { return _this.t > _other.t; };
class Kinetic_queue
{ //smallest prior

public:
    std::priority_queue<Event, std::vector<Event>, decltype(cmp)> queue;
    Kinetic_queue(std::vector<K_Polygon_3> &k_polys_3);
    std::optional<FT> next_event();

private:
    void collide(K_Polygon_3 &_this, K_Polygon_3 &_other);
};
