#include "kinetic.h"

std::optional<FT> collide(const K_Polygon_3 &_this,const K_Polygon_3 &_other)
{
    if (_this.plane() == _other.plane())
        return {};
    auto result = CGAL::intersection(_this.plane(), _other.plane());
    if (!result)
        return {};
    auto line_3 = *boost::get<Line_3>(&*result);

    // project to _this polygon
    auto line_2 = _this.project_2(line_3);
    FT t_min = INF;

    if (!line_polygon_intersect_2(line_2, _this.polygon_2())){
        //type a, b, c
        for (size_t i = 0; i < _this.points_2().size(); i++){
            auto r = Ray_2{_this._center, _this._speed[i]};
            assert(r.collinear_has_on(_this.points_2()[i]));
            auto res = CGAL::intersection(r, line_2);
            if (res)
                if (auto point_2 = boost::get<Point_2>(&*res)){
                    auto diff = *point_2 - _this.points_2()[i];
                    assert(diff.x() != 0);
                    auto t = diff.x() / _this._speed[i].x();
                    if (t<t_min) t_min = t;
                }
        }
    }
    else{
        //type d

    }
    if (t_min == INF) return {};
    return t_min;
}

