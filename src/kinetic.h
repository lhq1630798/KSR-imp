#pragma once
#include "gl_object.h"
#include "cgal_object.h"
#include <list>
#include <queue>

static const auto INF = FT{99999};

extern size_t next_id;

class KPoint_2;
class KLine_2;
class KSegment;
class KPolygon_2;
class KPolygons_2;
class KPolygons_SET;

using KP_Ref = std::list<KPoint_2>::iterator;
using KLine_Ref = std::list<KLine_2>::iterator;
using KSeg_Ref = std::list<KSegment>::iterator;
using KPoly_Ref = std::list<KPolygon_2>::iterator;
using KPolys_Ref = std::list<KPolygons_2>::iterator;

// class KP_Circ
// { // circular iterator
// public:
//     KP_Circ() = default;

//     KP_Circ(KPolygon_2 *kpoly_2, KP_Ref kp) : kpoly_2(kpoly_2), kp(kp) {}

//     operator KP_Ref() { return kp; } //cast

//     KPoint_2 &operator*()
//     {
//         return *kp;
//     }
//     KPoint_2 *operator->()
//     {
//         return &this->operator*();
//     }
//     KP_Circ &operator++(); //前置

//     KP_Circ &operator--(); //前置

//     KP_Circ operator++(int) //后置
//     {
//         KP_Circ ret = *this;
//         ++*this;
//         return ret;
//     }
//     KP_Circ operator--(int) //后置
//     {
//         KP_Circ ret = *this;
//         --*this;
//         return ret;
//     }

// private:
//     KP_Ref kp;
//     KPolygon_2 *kpoly_2;
// };

enum class Mode
{
    Frozen,
    Sliding,
    Normal
};

class KPoint_2 : public Point_2
{
public:
    KPoint_2(const Point_2 &point_2, const Vector_2 &speed, Mode status)
        : Point_2(point_2), _speed(speed), _status(status)
    {
        assert((_speed == CGAL::NULL_VECTOR) != (_status != Mode::Frozen));
    }

    Point_2 move_dt(::FT dt)
    {
        return *this + dt * _speed;
    }

    KPoint_2 &operator=(const Point_2 &point)
    {
        Point_2::operator=(point);
        return *this;
    }
    KPoint_2 &operator=(Point_2 &&point)
    {
        Point_2::operator=(std::move(point));
        return *this;
    }

    void sliding(const Vector_2 &speed)
    {
        assert(speed != CGAL::NULL_VECTOR);
        _speed = speed;
        _status = Mode::Sliding;
    }

    void frozen()
    {
        _status = Mode::Frozen;
        _speed = CGAL::NULL_VECTOR;
    }
    const Point_2 &point() const { return *this; }

    size_t id() const { return _id; }

    KP_Ref twin; // for sliding twin point on the same plane
    KPolygon_2 *face = nullptr;
    Vector_2 *twin_speed = nullptr; // for speed on the twin plane
    Vector_2 _speed = CGAL::NULL_VECTOR;
    Mode _status = Mode::Frozen;

private:
    size_t _id = (size_t)-1;
    friend class KPolygon_2;
    friend class KPolygons_SET;
};

/*
    Some algorithms need to know how to hash references (std::unordered_map)
    Here we simply hash the unique ID of the element.
*/
// namespace std
// {
//     template <>
//     struct hash<KP_Ref>
//     {
//         uint64_t operator()(KP_Ref key) const
//         {
//             static const std::hash<size_t> h;
//             return h(key->id());
//         }
//     };
// } // namespace std

class KPolygon_2
{
    friend class KPolygons_2;

public:
    KPolygon_2() = default;

    KPolygon_2(KPolygon_2 &&kpolygon_2) = default;
    // KPolygon_2(KPolygon_2 &&kpolygon_2)
    //     : _polygon_2(std::move(kpolygon_2.polygon_2())), dirty(false)
    // {
    //     // Using list::splice, No iterators or references become invalidated
    //     // https://stackoverflow.com/questions/26378894/stdlist-are-the-iterators-invalidated-on-move
    //     _kpoints_2.splice(_kpoints_2.begin(), kpolygon_2._kpoints_2);
    // }

    // explicit KPolygon_2(std::list<KPoint_2> &&kpoints_2)
    // {
    //     _kpoints_2.splice(_kpoints_2.begin(), std::move(kpoints_2));
    // }

    void init(Polygon_2 poly_2) //should only called by heap object
    {
        _polygon_2 = std::move(poly_2);
        dirty = false;

        Vector_2 center_V = CGAL::NULL_VECTOR;
        for (const auto &point_2 : _polygon_2.container())
            center_V += point_2 - CGAL::ORIGIN;
        center_V = center_V / _polygon_2.size();
        Point_2 center_P = CGAL::ORIGIN + center_V;
        assert(_polygon_2.has_on_bounded_side(center_P));
        for (const auto &point_2 : _polygon_2.container())
            insert_KP(KPoint_2{point_2, point_2 - center_P, Mode::Normal});
    }

    KP_Ref insert_KP(KP_Ref pos, const KPoint_2 &kpoint)
    {
        auto ref = _kpoints_2.insert(pos, kpoint);
        ref->face = this;
        ref->_id = next_id++;
        return ref;
    }
    KP_Ref insert_KP(const KPoint_2 &kpoint)
    { //insert at end
        return insert_KP(_kpoints_2.end(), kpoint);
    }

    const Polygon_2 &polygon_2() const
    {
        check();
        return _polygon_2;
    }
    size_t size() const { return _kpoints_2.size(); };

    // const std::list<KPoint_2> &kpoints_2() const { return _kpoints_2; }
    std::list<KPoint_2> &kpoints_2() { return _kpoints_2; }

    void move_dt(FT dt)
    {
        assert(dt >= 0);
        if (dt == 0)
        {
            std::cout << "zero dt" << std::endl;
            return;
        }

        for (auto &kpoint_2 : _kpoints_2)
            kpoint_2 = kpoint_2.move_dt(dt);
        dirty = true;

        assert(polygon_2().is_convex());
    }

    void erase(KP_Ref kp)
    {
        _kpoints_2.erase(kp);
        dirty = true;
    }

    KP_Ref next(KP_Ref kp)
    {
        kp++;
        if (kp == _kpoints_2.end())
            kp = _kpoints_2.begin();
        return kp;
    }

    KP_Ref prev(KP_Ref kp)
    {
        if (kp == _kpoints_2.begin())
            kp = _kpoints_2.end();
        kp--;
        return kp;
    }

    class Edge
    {
    public:
        Edge(KP_Ref kp1, KP_Ref kp2) : kp1(kp1), kp2(kp2) {}
        KP_Ref kp1, kp2;
        Segment_2 seg() const { return Segment_2{*kp1, *kp2}; }
        Line_2 line() const { return Line_2{*kp1, *kp2}; }
        Line_2 moved_line() const { return Line_2{kp1->move_dt(1), kp2->move_dt(1)}; }
        Vector_2 sliding_speed(const Line_2 &line_2) const;
    };

    std::vector<Edge> get_edges()
    {
        std::vector<Edge> edges;
        for (auto kp = _kpoints_2.begin(); kp != _kpoints_2.end(); kp++)
            edges.emplace_back(kp, next(kp));
        return edges;
    }

    Vec3 _color = rand_color();
    size_t id = (size_t)-1;
    KPolygons_2 *parent = nullptr;

private:
    void check() const
    {
        if (dirty)
        {
            update_polygon_2();
            dirty = false;
        }
    }

    void update_polygon_2() const
    {
        _polygon_2 = Polygon_2{_kpoints_2.begin(), _kpoints_2.end()};
    }

    static Vec3 rand_color()
    {
        static auto color_rand = CGAL::Random{0};
        return Vec3{(float)color_rand.get_double(0, 0.8),
                    (float)color_rand.get_double(0.2, 1),
                    (float)color_rand.get_double(0.2, 1)};
    }

    mutable bool dirty = true;
    mutable Polygon_2 _polygon_2;
    std::list<KPoint_2> _kpoints_2;
};

class KSegment
{
public:
    KSegment(Point_2 p1, Point_2 p2, Vector_2 sp1, Vector_2 sp2)
        : point1(p1), point2(p2), speed1(sp1), speed2(sp2) {}
    void move_dt(FT dt)
    {
        point1 = point1 + dt * speed1;
        point2 = point2 + dt * speed2;
    }
    bool has_on(Point_2 p) const
    {
        return Segment_2{point1, point2}.has_on(p);
    }
    Point_2 point1, point2;
    Vector_2 speed1, speed2;
};

class KLine_2
{
public:
    KLine_2(Line_2 line_2, KPolygons_2 *kpolygons)
        : _line_2(line_2), kpolygons(kpolygons){};
    KSeg_Ref insert_seg(const KSegment &kseg)
    {
        assert(_line_2.has_on(kseg.point1) && _line_2.has_on(kseg.point2));
        return _ksegments.insert(_ksegments.end(), kseg);
    }
    bool has_on(Point_2 p) const
    {
        for (auto &seg : _ksegments)
            if (seg.has_on(p))
                return true;
        return false;
    }
    void move_dt(FT dt)
    {
        for (auto &seg : _ksegments)
            seg.move_dt(dt);
    }
    Point_2 transform2twin(const Point_2 &point) const;
    std::pair<Point_2, Vector_2> transform2twin(const KPoint_2 &kpoint) const;
    std::list<KSegment> _ksegments;
    Line_2 _line_2;
    KPolygons_2 *kpolygons = nullptr;
    KLine_Ref twin; // KLine_2 on the other supporting plane
};

class KPolygons_2
{
    // set of 2D polygonal partitions in a common supporting plane
    friend class KPolygons_SET;

public:
    // private:
    KPolygons_2(const Polygon_3 &poly_3) : _plane(poly_3.plane())
    {
        insert_kpoly_2(KPolygon_2{})->init(poly_3.polygon_2());
    }

    Plane_3 _plane;
    std::list<KPolygon_2> _kpolygons_2;
    std::list<KLine_2> _klines;

    KPolygons_2 &operator=(const KPolygons_2 &) = delete;

    KLine_Ref insert_kline(const Line_2 &line_2)
    {
        return _klines.insert(_klines.end(), KLine_2{line_2, this});
    }

    KPoly_Ref insert_kpoly_2(KPolygon_2 kpoly_2)
    {
        auto ref = _kpolygons_2.insert(_kpolygons_2.end(), std::move(kpoly_2));
        ref->parent = this;
        ref->id = next_id++;
        return ref;
    }

    const std::list<KLine_2> &klines() const { return _klines; }

    const Plane_3 &plane() const { return _plane; }

    void move_dt(FT dt)
    {
        for (auto &kpoly : _kpolygons_2)
            kpoly.move_dt(dt);
        for (auto &kline : _klines)
            kline.move_dt(dt);
    }

    bool try_split(KPoly_Ref, const KLine_2 &);

    Point_2 project_2(const Point_3 &point_3) const
    {
        return plane().to_2d(point_3);
    }

    Line_2 project_2(const Line_3 &line_3) const
    {
        auto p1 = project_2(line_3.point(0)), p2 = project_2(line_3.point(1));
        return Line_2{p1, p2};
    }

    Segment_2 project_2(const Segment_3 &segment_3) const
    {
        auto p1 = project_2(segment_3.start()), p2 = project_2(segment_3.end());
        return Segment_2{p1, p2};
    }

    Point_3 to_3d(const Point_2 &point_2)
    {
        return plane().to_3d(point_2);
    }

    Segment_3 to_3d(const Segment_2 &segment_2)
    {
        return Segment_3{to_3d(segment_2.start()), to_3d(segment_2.end())};
    }

    Polygons_3 polygons_3() const
    {
        Polygons_3 result;
        for (const auto &kpoly_2 : _kpolygons_2)
            result.emplace_back(_plane, kpoly_2.polygon_2(), kpoly_2._color);
        return result;
    }
};

class KPolygons_SET
{
    // set of KPolygons_2 in different supporting planes
public:
    std::list<KPolygons_2> _kpolygons_set;
    KPolygons_SET(const Polygons_3 &polygons_3);

    size_t size() const { return _kpolygons_set.size(); }

    void move_dt(FT dt)
    {
        for (auto &kpolys : _kpolygons_set)
            kpolys.move_dt(dt);
    }

    Polygon_Mesh Get_mesh()
    {
        Polygons_3 polys_3;
        for (const auto &kpolys : _kpolygons_set)
        {
            auto tmp = kpolys.polygons_3();
            polys_3.insert(polys_3.end(), tmp.begin(), tmp.end());
        }
        return Polygon_Mesh{std::vector<Polygon_GL>(polys_3.begin(), polys_3.end())};
    }
};

class Event
{
public:
    FT t;
    KP_Ref kp;
    KLine_Ref kline;
    Event(FT t, KP_Ref kp, KLine_Ref kline)
        : t(t), kp(kp), kline(kline)
    {
        assert(t > 0);
        assert(kp->face);
        assert(kp->face->parent);
        assert(kp->id() < next_id);
    }
};

class Kinetic_queue;
class Event;

/* Comparison operator for Event so std::set will properly order them */
inline bool operator<(const Event &r1, const Event &r2)
{
    if (r1.t != r2.t)
    {
        return r1.t < r2.t;
    }
    assert(r1.kp->face->parent == r2.kp->face->parent);
    return &*r1.kp < &*r2.kp;
}

class Kinetic_queue
{
public:
    Kinetic_queue(KPolygons_SET &kpolygons_set);
    FT to_next_event();
    FT next_time();
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
    void remove_events(KP_Ref kp)
    {
        for (const auto &rm_event : id_events[kp->id()])
            remove(rm_event);
    }
    const Event &top(void) const { return *(queue.begin()); }
    void pop(void) { queue.erase(queue.begin()); }

    void Kinetic_queue::kp_collide(KP_Ref kp);
    void Kinetic_queue::erase_kp(KP_Ref kp);

    std::vector<KP_Ref> update_certificate(const Event &);

    KPolygons_SET &kpolygons_set;
    std::set<Event> queue;
    std::unordered_map<size_t, std::vector<Event>> id_events;
    FT last_t = 0;
};

void add_bounding_box(KPolygons_SET &KPolygons_set);
// void inf_perturb(Polygons_3 &polygons_3);
FT Vec_div(Vector_2 v1, Vector_2 v2);