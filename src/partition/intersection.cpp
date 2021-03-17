#include "cgal/cgal_object.h"

std::optional<std::pair<Point_3, Point_3>> plane_seg_intersect_3(const Plane_3 &plane, const Segments_3 &segments_3){
    auto inters = Points_3{};
    for (size_t i = 0; i < segments_3.size(); i++ )
    {
        const auto &seg = segments_3[i];
        if (auto result = CGAL::intersection(seg, plane))
        {
            if (auto inters_point = boost::get<Point_3>(&*result))
            {
                inters.push_back(*inters_point);
            }
            else if (auto inters_seg = boost::get<Segment_3>(&*result)){
                // std::cout << "Segment_3 on plane\n";
                return {};
            }
        }
    }
    if (inters.empty())
        return {};
    assert(inters.size() == 2);
    return {{inters[0], inters[1]}};
}

Segments_3 edges_3(const Points_3 &points_3) 
{
    auto segments_3 = Segments_3{};
    auto num = points_3.size();
    assert(num >= 3);
    for (size_t i = 1; i < num; i++)
        segments_3.push_back(Segment_3{points_3[i - 1], points_3[i]}); //edges_3[0] corresponds to points_3[0,1]
    segments_3.push_back(Segment_3{points_3[num - 1], points_3[0]});
    return segments_3;
}

std::optional<std::pair<Point_3, Point_3>> plane_polygon_intersect_3(const Plane_3 &plane, const Polygon_3 &polygon)
{
    return plane_seg_intersect_3(plane, edges_3(polygon.points_3()));
}