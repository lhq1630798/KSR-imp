#include "shape_diameter.h"
#include <CGAL/mesh_segmentation.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/linear_least_squares_fitting_3.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing_on_polygon_mesh.h>
#include "util/config.h"

namespace PMP = CGAL::Polygon_mesh_processing;

namespace Region_Growing {
    void save_region_growing_mesh(IC::Surface_Mesh, Regions);
    std::vector<IC::Plane_3> fit_planes(Regions, const IC::PWN_vector&);
    std::vector<EC::Detected_shape> create_detect_shape(std::vector<IC::Plane_3>, Regions, const IC::PWN_vector&);

    class SDF_Region_type {
        bool m_is_valid = false;
    public:
        SDF_Region_type(IC::Surface_Mesh _mesh) : mesh(_mesh)
        {
            const auto& param = Config::Detection::get();

            auto vnormals = mesh.add_property_map<IC::vertex_descriptor, IC::Vector_3>("v:normals", CGAL::NULL_VECTOR).first;
            fnormals = mesh.add_property_map<IC::face_descriptor, IC::Vector_3>("f:normals", CGAL::NULL_VECTOR).first;
            PMP::compute_normals(mesh, vnormals, fnormals);

            sdf_map = mesh.add_property_map<IC::face_descriptor, double>("f:sdf").first;
            auto [min, max] = CGAL::sdf_values(mesh, sdf_map, 2.0 / 3.0 * CGAL_PI, 25, true);
            sdf_min = min;
            sdf_max = max;

            segment_map = mesh.add_property_map<IC::face_descriptor, std::size_t>("f:sid").first;
            int level = param.level;
            float smooth = param.smooth;
            int number_of_segments = CGAL::segmentation_from_sdf_values(mesh, sdf_map, segment_map, level, smooth, false);

            std::vector <std::vector<double>> segments_sdfs(number_of_segments);
            for (auto fd : mesh.faces()) {
                segments_sdfs[segment_map[fd]].push_back(get_sdf(fd));
            }
            for (const auto& seg_sdfs : segments_sdfs) {
                auto ave_sdf = std::accumulate(seg_sdfs.begin(), seg_sdfs.end(), 0.0) / seg_sdfs.size();
                average_sdfs.push_back(ave_sdf);
                fmt::print("segment sdf {}\n", ave_sdf);
            }
        }

        bool is_part_of_region(
            const std::size_t,
            const std::size_t query_index,
            const std::vector<std::size_t>&) const {
            const auto& param = Config::Detection::get();
            double dist = max_face_distance(query_index);
            //double query_sdf = get_sdf(query_index);
            double query_sdf = get_segment_sdf(query_index);
            double sdf_rate = param.sdf_rate;
            double global_max_dist = param.max_distance_to_plane;
            //// it means region growing should start from small sdf face
            //return dist < 0.1*seed_sdf;

            return dist < sdf_rate* seed_sdf&&
                dist < sdf_rate* query_sdf&&
                dist < global_max_dist;

        }

        bool is_valid_region(const std::vector<std::size_t>& region) const {
            float m_min_region_size = Config::Detection::get().min_region_size;
            return (region.size() >= m_min_region_size);
        }

        void update(const std::vector<std::size_t>& region) {
            if (region.size() == 1) { // create new reference plane and normal
                m_plane_of_best_fit =
                    IC::Plane_3(face_centroid(region[0]), face_normal(region[0]));
                //seed_sdf = get_sdf(region[0]);
                seed_sdf = get_segment_sdf(region[0]);
                return;
            }

            std::vector<IC::Point_3> points;
            for (auto index : region) {
                auto fd = IC::face_descriptor(static_cast<IC::Surface_Mesh::size_type>(index));
                for (const auto vertex : mesh.vertices_around_face(mesh.halfedge(fd))) {
                    points.push_back(mesh.point(vertex));
                }
            }
            IC::Point_3 fitted_centroid;
            CGAL::linear_least_squares_fitting_3(
                points.begin(), points.end(),
                m_plane_of_best_fit, fitted_centroid,
                CGAL::Dimension_tag<0>(),
                IC::K(),
                CGAL::Eigen_diagonalize_traits<IC::FT, 3>());
            // TODO: need correct plane orientation?
            // TODO: update seed sdf?
        }
        double get_sdf(IC::face_descriptor fd) const {
            double sdf = sdf_min + (sdf_max - sdf_min) * sdf_map[fd];
            return sdf;
        }
        double get_sdf(std::size_t index) const {
            auto fd = IC::face_descriptor(static_cast<IC::Surface_Mesh::size_type>(index));
            return get_sdf(fd);
        }
        double get_segment_sdf(std::size_t index) const {
            auto fd = IC::face_descriptor(static_cast<IC::Surface_Mesh::size_type>(index));
            auto segment_id = segment_map[fd];
            return average_sdfs[segment_id];
        }
    private:
        IC::Point_3 face_centroid(std::size_t index) const {
            auto fd = IC::face_descriptor(static_cast<IC::Surface_Mesh::size_type>(index));
            std::vector<IC::Point_3> points;
            for (auto vd : mesh.vertices_around_face(mesh.halfedge(fd))) {
                points.push_back(mesh.point(vd));
            }
            assert(points.size() == 3);
            auto center = CGAL::centroid(points[0], points[1], points[2]);
            return center;
        }
        IC::Vector_3 face_normal(std::size_t index) const {
            auto fd = IC::face_descriptor(static_cast<IC::Surface_Mesh::size_type>(index));
            return fnormals[fd];
        }
        double max_face_distance(std::size_t index) const {
            auto fd = IC::face_descriptor(static_cast<IC::Surface_Mesh::size_type>(index));
            double max_dist = 0;
            for (auto vd : mesh.vertices_around_face(mesh.halfedge(fd))) {
                double dist = CGAL::squared_distance(m_plane_of_best_fit, mesh.point(vd));
                max_dist = max_dist > dist ? max_dist : dist;
            }
            return std::sqrt(max_dist);
        }
        IC::Surface_Mesh mesh;
        IC::Surface_Mesh::Property_map< IC::face_descriptor, double> sdf_map;
        IC::Surface_Mesh::Property_map< IC::face_descriptor, IC::Vector_3> fnormals;
        IC::Surface_Mesh::Property_map< IC::face_descriptor, std::size_t> segment_map;
        std::vector <double> average_sdfs;
        double sdf_min;
        double sdf_max;
        double seed_sdf = 0;
        IC::Plane_3 m_plane_of_best_fit;
    };

    std::vector<double> fitting_score(const IC::Surface_Mesh& mesh) {
        std::vector<double> m_scores(mesh.num_faces());

        for (std::size_t index = 0; index < mesh.num_faces(); ++index) {
            std::vector<IC::Point_3> points;
            auto fd = IC::face_descriptor(static_cast<IC::Surface_Mesh::size_type>(index));
            //auto fd = *(mesh.faces().begin() + index);

            for (auto face : mesh.faces_around_face(mesh.halfedge(fd))) {
                if (face == mesh.null_face())
                    continue;
                const auto hedge = halfedge(face, mesh);
                const auto vertices = vertices_around_face(hedge, mesh);
                for (const auto vertex : vertices) {
                    const auto& tmp_point = mesh.point(vertex);
                    points.push_back(tmp_point);
                }
            }

            IC::Plane_3 fitted_plane;
            IC::Point_3 fitted_centroid;
            m_scores[index] = CGAL::linear_least_squares_fitting_3(
                points.begin(), points.end(),
                fitted_plane, fitted_centroid,
                CGAL::Dimension_tag<0>(),
                IC::K(),
                CGAL::Eigen_diagonalize_traits<IC::FT, 3>());
        }
        return m_scores;
    }


    Regions region_growing_sdf(const IC::Surface_Mesh& mesh) {
        using Neighbor_query = CGAL::Shape_detection::Polygon_mesh::One_ring_neighbor_query<IC::Surface_Mesh>;
        using Seed_Map = CGAL::Pointer_property_map<size_t>::type;
        //using Sorting = CGAL::Shape_detection::Polygon_mesh::Least_squares_plane_fit_sorting<IC::K, IC::Surface_Mesh, Neighbor_query>;
        //using Seed_Map = Sorting::Seed_map;
        using Region_growing = CGAL::Shape_detection::Region_growing<IC::Surface_Mesh::Face_range, Neighbor_query, SDF_Region_type, Seed_Map>;

        for (std::size_t index = 0; index < mesh.num_faces(); ++index) {
            auto fd = *(mesh.faces().begin() + index);
            assert(index == std::size_t(fd));
        }

        Neighbor_query neighbor_query{ mesh };
        SDF_Region_type region_type{ mesh };

        std::vector<std::size_t> order;
        for (std::size_t i = 0; i < mesh.num_faces(); ++i)
            order.push_back(i);

        //================================================================
        ////start from small segments
        //std::vector<double> sdf_values;
        //for (std::size_t i = 0; i < mesh.num_faces(); ++i) {
        //    sdf_values.push_back(region_type.get_segment_sdf(i));
        //}
        //std::sort(order.begin(), order.end(), 
        //    [&sdf_values](std::size_t i, std::size_t j)  {
        //    return sdf_values[i] < sdf_values[j];
        //});

        //// in evert segment, start from large fitting score
        //auto scores = fitting_score(mesh);
        //auto begin = order.begin();
        //auto end = begin;
        //while (begin != order.end()) {
        //    while (end != order.end() && region_type.get_segment_sdf(*begin) == region_type.get_segment_sdf(*end)) {
        //        end++;
        //    }
        //    std::sort(begin, end,
        //        [&scores](std::size_t i, std::size_t j) {
        //        return scores[i] > scores[j];
        //    });
        //    begin = end;
        //}
        //================================================================
        auto scores = fitting_score(mesh);
        std::sort(scores.begin(), scores.end(),
            [&scores](std::size_t i, std::size_t j) {
            return scores[i] > scores[j];
        });
        //================================================================
        //// skip face with very small sdf
        //int skip_count = 0;
        //for (auto &index : order) {
        //    if (sdf_values[index] > Config::Detection::get().smallest_sdf) {
        //        break;
        //    }
        //    index = -1;
        //    skip_count++;
        //}
        //fmt::print("skip {} faces under smallest sdf\n", skip_count);
        //================================================================
        Region_growing region_growing(faces(mesh), neighbor_query, region_type, CGAL::make_property_map(order));


        //using Vertex_to_point_map = typename boost::property_map<IC::Surface_Mesh, CGAL::vertex_point_t>::type;
        //const Vertex_to_point_map vertex_to_point_map(
        //    get(CGAL::vertex_point, mesh));
        //Sorting sorting(mesh, neighbor_query, vertex_to_point_map);
        //sorting.sort();
        //Region_growing region_growing(faces(mesh), neighbor_query, region_type, sorting.seed_map());

        Regions regions;
        region_growing.detect(std::back_inserter(regions));

        if (Config::Detection::get().save_result) {
            save_region_growing_mesh(mesh, regions);
        }

        return regions;
    }
}