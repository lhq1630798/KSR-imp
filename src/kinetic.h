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
class Edge;
class KPolygon_2;
class KPolygons_2;
class KPolygons_SET;
class Collide_Ray;

using KP_Ref = std::list<KPoint_2>::iterator;
using KLine_Ref = std::list<KLine_2>::iterator;
using KSeg_Ref = std::list<KSegment>::iterator;
using KPoly_Ref = std::list<KPolygon_2>::iterator;
using KPolys_Ref = std::list<KPolygons_2>::iterator;

const struct Back {} BACK; //tag
const struct No_Check {}; //tag
class Vertex;
using Vert_Circ = CGAL::Circulator_from_container<std::list<Vertex>>;

enum class Mode
{
    Frozen,
    Sliding,
    Normal
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

    void set_collide_ray(std::shared_ptr<Collide_Ray> _collide_ray);
    void set_sliding_line(KLine_Ref kline);

    Vector_2 *seg_twin_speed = nullptr; // for speed on the twin plane
    Vector_2 _speed = CGAL::NULL_VECTOR;
    Mode _status = Mode::Frozen;
    Vert_Circ vertex;
    std::shared_ptr<Collide_Ray> collide_ray;
    bool ray_reverse;
    KLine_Ref sliding_line{};
private:
    friend class KPolygon_2;
    friend class KPolygons_2;
};

class Vertex
{
public:
    Vertex(KP_Ref _kp) : kp(_kp){};

    Vert_Circ &twin()
    {
        return _twin;
    }
    bool has_twin()
    {
        return !(twin() == nullptr);
    }
    void set_edge(const std::shared_ptr<Edge>& _edge);
    void set_edge(const std::shared_ptr<Edge>& _edge, No_Check);
    size_t _id = -1;
    KP_Ref kp;
    Vert_Circ _twin{};
    KPolygon_2 *face = nullptr;
    std::shared_ptr<Edge> edge;
};

inline void set_twin(Vert_Circ a, Vert_Circ b)
{
    a->twin() = b;
    b->twin() = a;
}

class Edge
{
public:
    Edge(Vert_Circ _vert1, Vert_Circ _vert2) : vert1(_vert1), vert2(_vert2)
    {
        _seg = Segment_2{ *vert1->kp, *vert2->kp };
        _line = _seg.supporting_line();
        _moved_line = Line_2{ vert1->kp->move_dt(1), vert2->kp->move_dt(1) };
    }
    Vert_Circ vert1, vert2;
    Segment_2 seg() const { return _seg; }
    Line_2 line() const { return _seg.supporting_line(); }
    Vector_2 sliding_speed(const Line_2& line_2) const;
    friend bool operator==(const Edge& a, const Edge& b) { return a.id == b.id; }

private:
    Line_2 moved_line() const { return _moved_line; }
    Segment_2 _seg;
    Line_2 _line;
    Line_2 _moved_line;
    size_t id = next_edge_id++;
    inline static size_t next_edge_id = 0;
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

    void set_inline_points(PWN_E points)
    {
        inline_points = std::move(points);
    }
    // ===================================================================
    // ============ methods that should mark dirty========================

    Vert_Circ steal_kp(Vert_Circ pos, KP_Ref kp)
    {
        return steal_kp(pos.current_iterator(), kp);
    }
    Vert_Circ steal_kp(const Back, KP_Ref kp)
    {
        return steal_kp(vertices.end(), kp);
    }
    void erase(Vert_Circ vert)
    {
        dirty = true;
        //vert->transfer_to(Vert_Circ{});
        vertices.erase(vert.current_iterator());
    }

    // ============ methods that should mark dirty========================
    // ===================================================================



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

    PWN_E inline_points;
    std::list<Vertex> vertices;

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

    Vert_Circ steal_kp(std::list<Vertex>::iterator pos, KP_Ref kp)
    {
        dirty = true;
        auto vert = Vert_Circ{&vertices, vertices.insert(pos, Vertex{kp})};
        vert->face = this;
        vert->_id = next_id();

        kp->vertex = vert;
        return vert;
    }

    mutable bool dirty = true;
    mutable Polygon_2 _polygon_2;
};

class KSegment
{
public:
    KSegment(Point_2 p1, Point_2 p2, Vector_2 sp1, Vector_2 sp2, FT t)
        : start1(p1), end1(p1), start2(p2), end2(p2), speed1(sp1), speed2(sp2), start_time(t){}
    void move_dt(FT dt)
    {
        end1 = end1 + dt * speed1;
        end2 = end2 + dt * speed2;
    }
    void move_to_t(FT t)
    {
        auto dt = (t - start_time);
        start_time = t;
        start1 = start1 + dt * speed1;
        start2 = start2 + dt * speed2;
        end1 = start1;
        end2 = start2;

    }
    bool has_on(Point_2 p) const
    {
        return Segment_2{ end1, end2 }.has_on(p);
    }
    Point_2 end1, end2;
    Point_2 start1, start2;
    Vector_2 speed1, speed2;
    FT start_time;
};

class KLine_2
{
public:
    KLine_2(Line_2 line_2, KPolygons_2 *kpolygons)
        : _line_2(line_2), kpolygons(kpolygons){};
    KSeg_Ref insert_seg(const KSegment &kseg)
    {
        assert(_line_2.has_on(kseg.start1) && _line_2.has_on(kseg.start2));
        return _ksegments.insert(_ksegments.end(), kseg);
    }
    void add_seg_twin(Vert_Circ sliding_prev, Vert_Circ sliding_next)
    {
        auto [p1t, sp1t] = transform2twin(*sliding_prev->kp);
        auto [p2t, sp2t] = transform2twin(*sliding_next->kp);
        auto seg_twin = twin->insert_seg(KSegment{p1t, p2t, sp1t, sp2t, time });
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
    void move_to_t(FT t)
    {
        time = t;
        for (auto& seg : _ksegments)
            seg.move_to_t(t);
    }
    Point_2 transform2twin(const Point_2 &point) const;
    std::pair<Point_2, Vector_2> transform2twin(const KPoint_2 &kpoint) const;
    std::list<KSegment> _ksegments;
    Line_2 _line_2;
    KPolygons_2 *kpolygons = nullptr;
    KLine_Ref twin; // KLine_2 on the other supporting plane
    bool is_bbox = false;
    std::shared_ptr< Collide_Ray> collide_ray;
    size_t line_id = next_id();
    FT time = 0;
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
    std::list<KPoint_2> active_KP;
    std::list<KPoint_2> frozen_KP;
    std::list<KLine_2> _klines;
    bool is_bbox = false;

    KPolygons_2 &operator=(const KPolygons_2 &) = delete;

    KLine_Ref insert_kline(const Line_2 &line_2)
    {
        return _klines.insert(_klines.end(), KLine_2{line_2, this});
    }

    KP_Ref new_KP(const KPoint_2 &kpoint)
    {
        auto ref = active_KP.insert(active_KP.end(), kpoint);
        ref->_id = next_id();
        if (ref->_status == Mode::Frozen)
            splice_to_frozen(ref);
        return ref;
    }
    KP_Ref new_sliding_KP(const Edge &edge, KLine_Ref kline, const Point_2 &start)
    {
        auto sliding_speed = edge.sliding_speed(kline->_line_2);
        auto kp = new_KP(KPoint_2{ start, sliding_speed, Mode::Sliding });
        kp->set_sliding_line(kline);
        return kp;
    }

    // KP_Ref kp_end()
    // {
    //     return active_KP.end();
    // }
    void splice_to_frozen(KP_Ref kp)
    {
         frozen_KP.splice(frozen_KP.end(), active_KP, kp);
    }
    void erase_kp(KP_Ref kp)
    {
        active_KP.erase(kp);
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
        for (auto kp = active_KP.begin(); kp != active_KP.end(); )
        {
            kp->frozen();
            splice_to_frozen(kp++);
        }
    }
    void move_dt(FT dt)
    {
        assert(dt >= 0);
        for (auto &kp : active_KP)
            kp = kp.move_dt(dt);
        for (auto &kpoly : _kpolygons_2)
            kpoly.dirty = true;

        for (auto &kline : _klines)
            kline.move_dt(dt);
    }

    bool try_split(KPoly_Ref, KLine_Ref);

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
                    auto point1 = kpolys->plane().to_3d(kseg.end1);
                    auto point2 = kpolys->plane().to_3d(kseg.end2);
                    end_points.emplace_back((float)CGAL::to_double(point1.x()),
                                            (float)CGAL::to_double(point1.y()),
                                            (float)CGAL::to_double(point1.z()));
                    end_points.emplace_back((float)CGAL::to_double(point2.x()),
                                            (float)CGAL::to_double(point2.y()),
                                            (float)CGAL::to_double(point2.z()));
                }
        return Lines_GL{end_points};
    }

    Lines_GL Get_Edges()
    {
        std::vector<Vec3> end_points{};
        for (auto kpolys = _kpolygons_set.begin(); kpolys != std::prev(_kpolygons_set.end(), 6); kpolys++)
            for (auto& kpoly2 : kpolys->_kpolygons_2)
                for (auto vert : kpoly2.vertices)
                {
                    if (vert.edge) {
                        auto point1 = kpolys->plane().to_3d(vert.kp->point());
                        auto point2 = kpolys->plane().to_3d(vert.kp->point() + vert.edge->seg().to_vector());
                        end_points.emplace_back((float)CGAL::to_double(point1.x()),
                            (float)CGAL::to_double(point1.y()),
                            (float)CGAL::to_double(point1.z()));
                        end_points.emplace_back((float)CGAL::to_double(point2.x()),
                            (float)CGAL::to_double(point2.y()),
                            (float)CGAL::to_double(point2.z()));
                    }
                }
        return Lines_GL{ end_points };
    }

    Point_cloud_GL Get_Point_cloud()
    {
        std::vector<Vec3> points{};
        for (auto kpolys = _kpolygons_set.begin(); kpolys != std::prev(_kpolygons_set.end(), 6); kpolys++)
            for (auto &kpoly2 : kpolys->_kpolygons_2)
                for (auto &[inline_point, normal] : kpoly2.inline_points)
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
    size_t kp_id;
    Point_2 pos;
    size_t event_id = next_event_id++;
    Event(FT t, KP_Ref kp, KLine_Ref kline, Point_2 pos)
        : t(t), kp(kp), kline(kline), plane(kp->vertex->face->parent), kp_id(kp->id()), pos(pos)
    {
        assert(t > 0);
        assert(kp->vertex != nullptr);
        assert(kp->vertex->face);
        assert(kp->vertex->face->parent);
        // assert(kp->id() < next_id);
    }
private:
    inline static size_t next_event_id = 0;
};

/* Comparison operator for Event so std::set will properly order them */
inline bool operator<(const Event &r1, const Event &r2)
{
    if (r1.event_id == r2.event_id) return false;
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
    Kinetic_queue(KPolygons_SET &kpolygons_set, bool exhausted = false);
    FT to_next_event();
    FT next_time();
    FT move_to_time(FT t);
    void done();
    size_t size() { return queue.size(); }
    Update_Point get_update_point()
    {
        std::vector<Vec3> points;
        for (auto kp : need_update)
        {
            auto point = kp->vertex->face->parent->plane().to_3d(*kp);
            points.emplace_back((float)CGAL::to_double(point.x()),
                                (float)CGAL::to_double(point.y()),
                                (float)CGAL::to_double(point.z()));
        }

        return Update_Point{std::move(points)};
    }

private:
    void insert(const Event &event) { 
        id_events[event.kp_id].push_back(event);
        event_num[event.kp_id]++;
        queue.insert(event); 
    }
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
        id_events.erase(kp->id());
    }
    const Event &top(void) const { return *(queue.begin()); }
    void pop(void) { 
        event_num[top().kp_id]--;
        queue.erase(queue.begin()); 
    }

    void kp_collide(KP_Ref kp);
    void erase_kp(KPolygons_2* parent, KP_Ref kp);
    bool stop_extend(KLine_Ref kline, KP_Ref kp )
    {
        assert(kline->_line_2.has_on(kp->point()));
        if (kline->is_bbox)
            return true;
        if (exhausted) return false;
        auto ret = kline->has_on(kp->point());
        //if(ret) kline->is_bbox = true;
        return ret;
    }

    void update_certificate();
    std::vector<KP_Ref> type_a(const Event &);
    std::vector<KP_Ref> type_b(Vert_Circ vert, KLine_Ref kline);
    std::vector<KP_Ref> type_c(Event, Event);

    KPolygons_SET &kpolygons_set;
    std::set<Event> queue;
    std::unordered_map<size_t, std::vector<Event>> id_events;
    std::unordered_map<size_t, int> event_num;
    std::vector<KP_Ref> need_update;
    bool exhausted;
    // I think last_t may cause stack overflow https://github.com/CGAL/cgal/issues/1118
    FT last_t = 0;
};


class Collide_Ray {
public:
    struct Record {
        FT dot;
        KLine_Ref kline_2;
        Point_2 pos;
    };

    Collide_Ray(Ray_2 _ray, KLine_Ref begin, KLine_Ref end);
    std::vector<Record> next_hit(KP_Ref kp);
    bool is_reverse(const KPoint_2 &kpoint) {
        if (kpoint._speed * vec > 0) {
            assert(kpoint._speed.direction() == vec.direction());
            return false;
        }
        assert(kpoint._speed.direction() == -vec.direction());
        return true;
    }
private:
    Point_2 start;
    Ray_2 ray;
    Vector_2 vec;
    std::vector<Record> records;
};