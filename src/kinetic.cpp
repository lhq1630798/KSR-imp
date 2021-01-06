#include "kinetic.h"

Kinetic_queue::Kinetic_queue(std::vector<K_Polygon_3> &k_polys_3) : queue(cmp)
{
    std::cout << "num of polygons " << k_polys_3.size() << std::endl;
    for (auto &_this : k_polys_3)
        for (auto &_other : k_polys_3)
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

    if (!line_polygon_intersect_2(line_2, _this.polygon_2()))
    {
        //type a, b, c
        for (size_t i = 0; i < _this.points_2().size(); i++)
        {
            auto r = Ray_2{_this._center, _this._speed[i]};
            assert(r.has_on(_this.points_2()[i]));
            if (auto res =CGAL::intersection(r, line_2))
                if (auto point_2_p = boost::get<Point_2>(&*res))
                {
                    auto diff = *point_2_p - _this.points_2()[i];
                    assert(diff.x() != 0);
                    auto t = diff.x() / _this._speed[i].x();

                    queue.push(Event{&_this, &_other, _this.plane().to_3d(*point_2_p), t});
                }
        }
    }
    else
    {
        //type d
    }
}

std::optional<FT> Kinetic_queue::next_event()
{
    while (!queue.empty())
    {
        auto &event = queue.top();
        queue.pop();

        auto t = event.t;
        auto _this = *event.this_p, _other = *event.other_p;
        assert(_this.plane().has_on(event.point_3));
        assert(_other.plane().has_on(event.point_3));
        if (Polygon_3{_other.plane(), _other.move_dt(t)}.has_on(event.point_3))
            return t;
    }
    return {};
}

