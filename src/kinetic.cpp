#include "kinetic.h"

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

    if (!line_polygon_intersect_2(line_2, _this.polygon_2()))
    {
        //type a, b, c
        for (size_t i = 0; i < _this.points_2().size(); i++)
        {
            auto &point_2 = _this.points_2()[i];
            auto &speed = _this._speed[i];
            auto &id = _this._ids[i];

            auto r = Ray_2{point_2, speed};
            assert(r.opposite().has_on(point_2));
            if (auto res =CGAL::intersection(r, line_2))
            {
                if (boost::get<Ray_2>(&*res)) //sliding
                    continue;
                if (auto point_2_p = boost::get<Point_2>(&*res))
                {
                    auto diff = *point_2_p - point_2;
                    assert(diff.x() != 0);
                    auto t = last_t + diff.x() / speed.x();
                    auto point_3 = _this.plane().to_3d(*point_2_p);
                    auto event = Event{&_this, &_other, id, point_3, line_2, t};
                    id_events[id].push_back(event);
                    insert(event);
                }
            }

        }
    }
    else
    {
        //type d
    }
}

FT Kinetic_queue::next_event()
{
    while (!queue.empty())
    {
        auto event = top();
        pop();
        auto &t = event.t;
        auto &_this = *event.this_p, _other = *event.other_p;
        assert(_this.plane().has_on(event.point_3));
        assert(_other.plane().has_on(event.point_3));
        if (Polygon_3{_other.plane(), _other.move_dt(t)}.has_on(event.point_3)){
            for (const auto &rm_event : id_events[event.id])
                remove(rm_event);
            for (auto &k_poly : k_polygons_3)
			    k_poly.update(k_poly.move_dt(t - last_t));
		    last_t = t;

            // split point
            std::cout << "find index: id ";
            for(size_t ind = 0; ind < _this.points_2().size(); ind++){
                if (_this._ids[ind] == event.id){
                    std::cout << event.id << " at _ids[" << ind << "]" << std::endl;
                    _this.insert_sliding(ind,event.line_2);
                }
            }

            return t;
        }
    }
    return {};
}

