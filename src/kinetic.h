#pragma once
#include "gl_object.h"
#include "cgal_object.h"
#include <list>
#include <queue>

static const auto INF = FT{99999};

size_t next_id();

// void inf_perturb(Polygons_3 &polygons_3);
FT Vec_div(Vector_2 v1, Vector_2 v2);

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

class Vertex;
using Vert_Circ = CGAL::Circulator_from_container<std::list<Vertex>>;

enum class Mode
{
    Frozen,
    Sliding,
    Normal
};

class Vertex
{
public:
    Vertex(KP_Ref _kp) : kp(_kp){};
    void transfer_to(Vert_Circ to);
    void set_twin(Vert_Circ twin);
    Vert_Circ &twin();
    bool has_twin();
    size_t _id;
    KP_Ref kp;
    KPolygon_2 *face;
};

class KPoint_2 : public Point_2
{
private:
    size_t _id = (size_t)-1;

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

    void sliding_speed(const Vector_2 &speed)
    {
        assert(_status == Mode::Sliding);
        if (seg_twin_speed)
            *seg_twin_speed *= Vec_div(speed, _speed);
        _speed = speed;
    }
    bool has_twin_vert()
    {
        if (twin_vert == nullptr)
            return false;
        assert_twin_vert();
        return true;
    }
    void frozen()
    {
        if (_status == Mode::Sliding)
            sliding_speed(CGAL::NULL_VECTOR);
        else
            _speed = CGAL::NULL_VECTOR;
        _status = Mode::Frozen;
    }

    const Point_2 &point() const { return *this; }

    size_t id() const { return _id; }

    Vector_2 *seg_twin_speed = nullptr; // for speed on the twin plane
    Vector_2 _speed = CGAL::NULL_VECTOR;
    Mode _status = Mode::Frozen;
    Vert_Circ twin_vert; // for sliding twin point on the same plane
    Vert_Circ vertex;

private:
    void assert_twin_vert();
    friend class KPolygon_2;
    friend class KPolygons_2;
};

class KPolygon_2
{
    friend class KPolygons_2;

private:
    // friend class std::list<KPolygon_2>::allocator_type;
    // KPolygon_2 should only be constructed on heap... for example in std::list<KPolygon_2>
public:
    size_t id = (size_t)-1;

    KPolygon_2() = default;

    KPolygon_2(KPolygons_2 *parent, Polygon_2 poly_2);

    KPolygon_2(const KPolygon_2 &) = delete;
    KPolygon_2 &operator=(const KPolygon_2 &) = delete;

    void set_inline_points(std::vector<Point_3> points)
    {
        inline_points = std::move(points);
    }
    // ===================================================================
    // ============ methods that should mark dirty========================

    Vert_Circ insert_KP(Vert_Circ pos, KPoint_2 kpoint)
    {
        return insert_KP(pos.current_iterator(), std::move(kpoint));
    }
    Vert_Circ insert_KP(KPoint_2 kpoint)
    {
        return insert_KP(vertices.end(), std::move(kpoint));
    }

    Vert_Circ steal_vert(Vert_Circ pos, Vert_Circ from_vert)
    {
        return steal_vert(pos.current_iterator(), std::move(from_vert));
    }
    Vert_Circ steal_vert(Vert_Circ from_vert)
    {
        return steal_vert(vertices.end(), std::move(from_vert));
    }

    Vert_Circ steal_as_twin(Vert_Circ pos, Vert_Circ from_vert)
    {
        return steal_as_twin(pos.current_iterator(), from_vert);
    }

    Vert_Circ steal_as_twin(Vert_Circ from_vert)
    {
        return steal_as_twin(vertices.end(), from_vert);
    }
    void erase(Vert_Circ vert)
    {
        dirty = true;
        //vert->transfer_to(Vert_Circ{});
        vertices.erase(vert.current_iterator());
    }

    // ============ methods that should mark dirty========================
    // ===================================================================

    class Edge
    {
    public:
        Edge(Vert_Circ _vert1, Vert_Circ _vert2) : vert1(_vert1), vert2(_vert2) {}
        Vert_Circ vert1, vert2;
        Segment_2 seg() const { return Segment_2{*vert1->kp, *vert2->kp}; }
        Line_2 line() const { return Line_2{*vert1->kp, *vert2->kp}; }
        Vector_2 sliding_speed(const Line_2 &line_2) const;
        friend bool operator==(const Edge &a, const Edge &b) { return a.id == b.id; }

    private:
        Line_2 moved_line() const { return Line_2{vert1->kp->move_dt(1), vert2->kp->move_dt(1)}; }
        size_t id = next_edge_id++;
        inline static size_t next_edge_id = 0;
    };

    std::vector<Edge> get_edges()
    {
        std::vector<Edge> edges;
        auto vert = vert_circulator(), end = vert;
        CGAL_For_all(vert, end)
            edges.emplace_back(vert, std::next(vert));
        return edges;
    }

    const Polygon_2 &polygon_2() const
    {
        check();
        return _polygon_2;
    }
    size_t size() const { return vertices.size(); };

    Vert_Circ vert_circulator() { return Vert_Circ{&vertices}; }

    Vec3 _color = rand_color();
    KPolygons_2 *parent = nullptr;

    std::vector<Point_3> inline_points;

    FT area()
    {
        return polygon_2().area();
    }

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
        _polygon_2 = Polygon_2{};
        for (auto &vert : vertices)
            _polygon_2.push_back(vert.kp->point());
        assert(_polygon_2.is_convex());
    }

    Vert_Circ insert_KP(std::list<Vertex>::iterator pos, KPoint_2 &&kpoint);
    Vert_Circ steal_vert(std::list<Vertex>::iterator pos, Vert_Circ &&from_vert)
    {
        dirty = true;

        auto vert = Vert_Circ{&vertices, vertices.insert(pos, Vertex{from_vert->kp})};
        vert->face = this;
        vert->_id = next_id();

        from_vert->transfer_to(vert);

        return vert;
    }
    Vert_Circ steal_as_twin(std::list<Vertex>::iterator pos, Vert_Circ from_vert)
    {
        dirty = true;

        auto vert = Vert_Circ{&vertices, vertices.insert(pos, Vertex{from_vert->kp})};
        vert->face = this;
        vert->_id = next_id();

        //from_vert->set_twin(vert);
        from_vert->twin() = vert;
        return vert;
    }
    mutable bool dirty = true;
    mutable Polygon_2 _polygon_2;
    std::list<Vertex> vertices;
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
    void add_seg_twin(Vert_Circ sliding_prev, Vert_Circ sliding_next)
    {
        auto [p1t, sp1t] = transform2twin(*sliding_prev->kp);
        auto [p2t, sp2t] = transform2twin(*sliding_next->kp);
        auto seg_twin = twin->insert_seg(KSegment{p1t, p2t, sp1t, sp2t});
        sliding_prev->kp->seg_twin_speed = &seg_twin->speed1;
        sliding_next->kp->seg_twin_speed = &seg_twin->speed2;
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
        insert_kpoly_2(poly_3.polygon_2())->set_inline_points(poly_3.inline_points);
    }

    Plane_3 _plane;
    std::list<KPolygon_2> _kpolygons_2;
    std::list<KPoint_2> all_KP;
    std::list<KLine_2> _klines;

    KPolygons_2 &operator=(const KPolygons_2 &) = delete;

    KLine_Ref insert_kline(const Line_2 &line_2)
    {
        return _klines.insert(_klines.end(), KLine_2{line_2, this});
    }

    KP_Ref new_KP(const KPoint_2 &kpoint)
    {
        auto ref = all_KP.insert(all_KP.end(), kpoint);
        ref->_id = next_id();
        return ref;
    }

    void erase_kp(KP_Ref kp)
    {
        all_KP.erase(kp);
    }

    KPoly_Ref insert_kpoly_2(const Polygon_2 &poly_2)
    {
        auto ref = _kpolygons_2.emplace(_kpolygons_2.end(), this, poly_2);
        init_kpoly(ref);
        return ref;
    }
    KPoly_Ref insert_kpoly_2()
    {
        auto ref = _kpolygons_2.emplace(_kpolygons_2.end());
        init_kpoly(ref);
        return ref;
    }

    std::list<KLine_2> &klines() { return _klines; }

    const Plane_3 &plane() const { return _plane; }

    void frozen()
    {
        for (auto &kp : all_KP)
            kp.frozen();
    }
    void move_dt(FT dt)
    {
        assert(dt >= 0);
        for (auto &kp : all_KP)
            kp = kp.move_dt(dt);
        for (auto &kpoly : _kpolygons_2)
            kpoly.dirty = true;

        for (auto &kline : _klines)
            kline.move_dt(dt);
    }

    bool try_split(KPoly_Ref, KLine_2 &);

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
        //inline
        return result;
    }

private:
    void init_kpoly(KPoly_Ref ref)
    {
        ref->parent = this;
        ref->id = next_id();
    }
};

class KPolygons_SET
{
    // set of KPolygons_2 in different supporting planes
public:
    std::list<KPolygons_2> _kpolygons_set;
    KPolygons_SET(Polygons_3 polygons_3, bool exhausted = false);

    size_t size() const { return _kpolygons_set.size() - 6; }

    void move_dt(FT dt)
    {
        assert(dt >= 0);
        if (dt == 0)
        {
            std::cout << "zero dt" << std::endl;
            return;
        }
        for (auto &kpolys : _kpolygons_set)
            kpolys.move_dt(dt);
    }

    Polygon_Mesh Get_mesh()
    {
        Polygons_3 polys_3;
        for (auto kpolys = _kpolygons_set.begin(); kpolys != std::prev(_kpolygons_set.end(), 3); kpolys++)
        {
            auto tmp = kpolys->polygons_3();
            polys_3.insert(polys_3.end(), tmp.begin(), tmp.end());
        }
        return Polygon_Mesh{std::vector<Polygon_GL>(polys_3.begin(), polys_3.end())};
    }

    Lines_GL Get_Segments()
    {
        std::vector<Vec3> end_points{};
        for (auto kpolys = _kpolygons_set.begin(); kpolys != std::prev(_kpolygons_set.end(), 6); kpolys++)
            for (auto &kline : kpolys->_klines)
                for (auto &kseg : kline._ksegments)
                {
                    auto point1 = kpolys->plane().to_3d(kseg.point1);
                    auto point2 = kpolys->plane().to_3d(kseg.point2);
                    end_points.emplace_back((float)CGAL::to_double(point1.x()),
                                            (float)CGAL::to_double(point1.y()),
                                            (float)CGAL::to_double(point1.z()));
                    end_points.emplace_back((float)CGAL::to_double(point2.x()),
                                            (float)CGAL::to_double(point2.y()),
                                            (float)CGAL::to_double(point2.z()));
                }
        return Lines_GL{end_points};
    }

    Point_cloud_GL Get_Point_cloud()
    {
        std::vector<Vec3> points{};
        for (auto kpolys = _kpolygons_set.begin(); kpolys != std::prev(_kpolygons_set.end(), 6); kpolys++)
            for (auto &kpoly2 : kpolys->_kpolygons_2)
                for (auto &inline_point : kpoly2.inline_points)
                    points.emplace_back((float)CGAL::to_double(inline_point.x()),
                                        (float)CGAL::to_double(inline_point.y()),
                                        (float)CGAL::to_double(inline_point.z()));
        return Point_cloud_GL{std::move(points)};
    }

private:
    void add_bounding_box(const Polygons_3 &polygons_3);
    void decompose();
    void bbox_clip();
};

class Event
{
public:
    FT t;
    KP_Ref kp;
    KLine_Ref kline;
    KPolygons_2 *plane;
    Event(FT t, KP_Ref kp, KLine_Ref kline)
        : t(t), kp(kp), kline(kline), plane(kp->vertex->face->parent)
    {
        assert(t > 0);
        assert(kp->vertex != nullptr);
        assert(kp->vertex->face);
        assert(kp->vertex->face->parent);
        // assert(kp->id() < next_id);
    }
};

/* Comparison operator for Event so std::set will properly order them */
inline bool operator<(const Event &r1, const Event &r2)
{
    if (r1.t != r2.t)
    {
        return r1.t < r2.t;
    }
    assert(r1.plane == r2.plane);
    return &*r1.kp < &*r2.kp;
}

class Kinetic_queue
{
public:
    Kinetic_queue(KPolygons_SET &kpolygons_set);
    FT to_next_event();
    FT next_time();
    FT move_to_time(FT t);
    size_t size() { return queue.size(); }
    Update_Point get_update_point()
    {
        std::vector<Vec3> points;
        for (auto kp : need_update){
            auto point = kp->vertex->face->parent->plane().to_3d(*kp);
            points.emplace_back((float)CGAL::to_double(point.x()),
                                (float)CGAL::to_double(point.y()),
                                (float)CGAL::to_double(point.z()));
        }

        return Update_Point{std::move(points)};
    }

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

    void kp_collide(KP_Ref kp);
    void erase_vert(Vert_Circ vert);

    std::vector<KP_Ref> update_certificate(const Event &);
    std::vector<KP_Ref> type_b(Vert_Circ vert, KLine_Ref kline);
    std::vector<KP_Ref> type_c(Vert_Circ vert, KLine_Ref kline, const Event &event);
    Event last_event();

    KPolygons_SET &kpolygons_set;
    std::set<Event> queue;
    std::unordered_map<size_t, std::vector<Event>> id_events;
    std::vector<KP_Ref> need_update;
    // I think last_t may cause stack overflow https://github.com/CGAL/cgal/issues/1118
    FT last_t = 0;
};
