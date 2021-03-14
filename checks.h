#pragma once

#include "polyhedron/glm_ext/glm_extensions.h"

#include "xml_config.h"
#include "enums.h"


namespace hidden {
    constexpr float precision_bias = FLT_EPSILON;
    
    template<typename base_t> constexpr glm::vec<3, base_t> 
    swizzle_vector_3d(
        const glm::vec<3, base_t> &v, 
        const swizzle_mode mode
    ) 
    {
        switch(mode) {
            case xzy:
                return v.xzy();
            case yxz:
                return v.yxz();
            case yzx:
                return v.yzx();
            case zxy:
                return v.zxy();
            case zyx:
                return v.zyx();
            default:
                return v;
                
        };
    } 
    
    template<typename base_t> constexpr glm::vec<2, base_t> 
    swizzle_vector_2d(
        const glm::vec<3, base_t> &v, 
        const swizzle_mode mode
    ) 
    {
        switch(mode) {
            case xzy:
                return v.xz();
            case yxz:
                return v.yx();
            case yzx:
                return v.yz();
            case zxy:
                return v.zx();
            case zyx:
                return v.zy();
            default:
                return v;
        };
    } 
    
    struct plane {
        glm::vec3 normal;
        float d;
    };
    
    template<class T>
    T prand(const uint32_t min, const uint32_t max) {
        static uint32_t x = 123456789;
        static uint32_t y = 362436069;
        static uint32_t z = 521288629;
        static uint32_t w = 88675123;
        uint32_t t = x ^ (x << 11);   
        x = y; y = z; z = w;   
        w = w ^ (w >> 19) ^ (t ^ (t >> 8));
        return (T)(w % (max - min + 1)) + min;
    }
};

namespace checks {
    namespace raycast {
        //! returns true if the distance is close to FLT_EPSILON
        //! out_distance is the actual distance to the edge
        template<typename base_t>
        bool distance_to_edge (
            const glm::vec<2, base_t> &in_pt, 
            const glm::vec<2, base_t> &in_v1, 
            const glm::vec<2, base_t> &in_v2,
            const float bias,
            float &out_distance) 
        {            
            const glm::vec2 edge = glm::closestPointOnLine(in_pt, in_v1, in_v2);
            out_distance = glm::length(edge - in_pt);
            if(out_distance <= bias) {
                return true;
            }
            return false;
        }
        
        template<typename base_t>
        bool pt_in_triangle (
            const glm::vec<2, base_t> &in_pt, 
            const glm::vec<3, base_t> &in_v1, 
            const glm::vec<3, base_t> &in_v2, 
            const glm::vec<3, base_t> &in_v3) 
        {
            glm::vec<3, base_t> dist_1 = glm::vec<3, base_t>(in_pt.x, in_pt.y, 0) - in_v1;
            glm::vec<3, base_t> dist_2 = glm::vec<3, base_t>(in_pt.x, in_pt.y, 0) - in_v2;
            glm::vec<3, base_t> dist_3 = glm::vec<3, base_t>(in_pt.x, in_pt.y, 0) - in_v3;
            
            glm::vec<3, float> e1 = in_v1 - in_v2;
            glm::vec<3, float> e2 = in_v2 - in_v3;
            glm::vec<3, float> e3 = in_v3 - in_v1;
            
            const base_t d1 = dist_2.x * e1.y - e1.x * dist_2.y;
            const base_t d2 = dist_3.x * e2.y - e2.x * dist_3.y;
            const base_t d3 = dist_1.x * e3.y - e3.x * dist_1.y;
            
            const bool has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
            const bool has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);
            return !(has_neg && has_pos);
        }        
        
        template<typename base_t>
        bool pt_in_triangle (
            const glm::vec<2, base_t> &in_pt, 
            const glm::vec<3, base_t> &in_v1, 
            const glm::vec<3, base_t> &in_v2, 
            const glm::vec<3, base_t> &in_v3,
            int &out_distance ) 
        {
            glm::vec<3, base_t> dist_1 = glm::vec<3, base_t>(in_pt.x, in_pt.y, 0) - in_v1;
            glm::vec<3, base_t> dist_2 = glm::vec<3, base_t>(in_pt.x, in_pt.y, 0) - in_v2;
            glm::vec<3, base_t> dist_3 = glm::vec<3, base_t>(in_pt.x, in_pt.y, 0) - in_v3;
            
            glm::vec<3, float> e1 = in_v1 - in_v2;
            glm::vec<3, float> e2 = in_v2 - in_v3;
            glm::vec<3, float> e3 = in_v3 - in_v1;
            
            const base_t d1 = dist_2.x * e1.y - e1.x * dist_2.y;
            const base_t d2 = dist_3.x * e2.y - e2.x * dist_3.y;
            const base_t d3 = dist_1.x * e3.y - e3.x * dist_1.y;
            
            const bool has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
            const bool has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);
            
            const bool is_in = !(has_neg && has_pos);
            if(is_in) {
                const glm::vec3 n = glm::cross(glm::normalize(e1), glm::normalize(e2));
                const float k = glm::compAdd(n*(glm::vec3)in_v1);
                out_distance = (k - n.x*in_pt.x-n.y*in_pt.y) / n.z;
            }
            return is_in;
        }
        
        //! this function tries to find a position without edge collision
        //! and then tries to perform a ray intersection test
        template<swizzle_mode mode, typename base_t, typename ind_buf_t>
        std::set<int> get_intersections_safe(
            const glm::vec<2, base_t> &pos, 
            const std::vector<glm::vec<3, base_t>> &vert_buffer, 
            const ind_buf_t &face_indices
        )
        {
            // issue: edge collision
            // in case of an edge collision, we move the position a bit
            constexpr int max_iter = 1000;
            glm::vec<2, base_t> p2d;
                        
            // try to find a good position
            for(int i = 0; i <= max_iter; i++) {
                bool collision = false;
                glm::vec<2, base_t> tmp = pos;
                for(const auto &f : face_indices) {
                    tmp[hidden::prand<int>(0,1)] += hidden::prand<float>(1, max_iter) / max_iter * i / (max_iter+1);
 
                    const glm::vec<2, base_t> swizzle_v1 = hidden::swizzle_vector_2d(vert_buffer[f[0]], mode);
                    const glm::vec<2, base_t> swizzle_v2 = hidden::swizzle_vector_2d(vert_buffer[f[1]], mode);
                    const glm::vec<2, base_t> swizzle_v3 = hidden::swizzle_vector_2d(vert_buffer[f[2]], mode);
                    
                    float proximity; 
                    if(distance_to_edge(tmp, swizzle_v1, swizzle_v2, hidden::precision_bias, proximity)) { collision = true; break; }
                    if(distance_to_edge(tmp, swizzle_v1, swizzle_v3, hidden::precision_bias, proximity)) { collision = true; break; }
                    if(distance_to_edge(tmp, swizzle_v2, swizzle_v3, hidden::precision_bias, proximity)) { collision = true; break; }
                    if(glm::length(tmp - swizzle_v1) < hidden::precision_bias) { collision = true; break; }
                    if(glm::length(tmp - swizzle_v2) < hidden::precision_bias) { collision = true; break; }
                    if(glm::length(tmp - swizzle_v3) < hidden::precision_bias) { collision = true; break; }
                }
                // success because there was no collision
                if(!collision) {
                    p2d = tmp;
                    break;
                }
                // fail, there was a collision and we don't want to test forever
                if(i == max_iter) {
                    return {};
                }
            }

            //! do a 2d ray intersection test
            glm::vec<3, base_t> dir;
            std::set<int> inters_dist;
            for(const auto &f : face_indices) {                
                // models are scales to 0 <= x <= MAX, 
                // so starting from zero is a bad idea
                float ray_start = -1;
                
                // zwizzle the position and check
                glm::vec<3, base_t> pos_3d;
                if constexpr (mode == swizzle_mode::yzx) {
                    dir = glm::vec<3, base_t>(1,0,0);
                    pos_3d = glm::vec<3, base_t>(ray_start, p2d.x, p2d.y);
                }
                else if(mode == swizzle_mode::xzy) {
                    dir = glm::vec<3, base_t>(0,1,0);
                    pos_3d = glm::vec<3, base_t>(p2d.x, ray_start, p2d.y);
                }
                else if(mode == swizzle_mode::xyz){
                    dir = glm::vec<3, base_t>(0,0,1);
                    pos_3d = glm::vec<3, base_t>(p2d.x, p2d.y, ray_start);
                }

                glm::vec2 bary_pos(0); float distance = 0;
                const bool is_inters = glm::intersectRayTriangle(
                    pos_3d,
                    dir,
                    vert_buffer[f[0]],
                    vert_buffer[f[1]],
                    vert_buffer[f[2]],
                    bary_pos,
                    distance
                );                
                // glm 9.9: 
                // the intersectRayTriangle behaves like a line intersection,
                // always returning two opposing intersecting faces
                const bool positive = distance > 0 ? true : false;
                if(is_inters && positive) {
                    inters_dist.insert(glm::round(distance + ray_start));
                }
            }
            return inters_dist;
        }
        
        //! this function is just fast
        template<swizzle_mode mode, typename base_t, typename ind_buf_t>
        std::set<int> get_intersections_fast(
            const glm::vec<2, base_t> &pos, 
            const std::vector<glm::vec<3, base_t>> &vert_buffer, 
            const ind_buf_t &face_indices
        ) 
        {
            std::set<int> inters_dist;
            for(const auto &f : face_indices) {
                const glm::vec<3, base_t> &v1 = hidden::swizzle_vector_3d(vert_buffer[f[0]], mode);
                const glm::vec<3, base_t> &v2 = hidden::swizzle_vector_3d(vert_buffer[f[1]], mode);
                const glm::vec<3, base_t> &v3 = hidden::swizzle_vector_3d(vert_buffer[f[2]], mode);

                int d = 0;
                if(pt_in_triangle(pos, v1, v2, v3, d)) {
                    inters_dist.insert(d);
                    continue;
                }
            }
            return inters_dist;
        }
    };

    namespace intersection_3d {
        inline bool axis_test_x02(const glm::vec3 &e, const glm::vec3 &voxel_center, const std::array<glm::vec3, 3> &face, const glm::vec3 &half_box_size) {
            auto v0 = face[0] - voxel_center;
            auto v2 = face[2] - voxel_center;

            float fa = std::abs(e.z);
            float fb = std::abs(e.y);

            float p1 = e.z * v0.y - e.y * v0.z;
            float p3 = e.z * v2.y - e.y * v2.z;

            float min = p1 > p3 ? p3 : p1;
            float max = p1 > p3 ? p1 : p3;

            float rad = fa * half_box_size.y + fb * half_box_size.z;
            if (min > rad || max < -rad) {
                return true;
            }
            return false;
        }
        inline bool axis_test_x01(const glm::vec3 &e, const glm::vec3 &voxel_center, const std::array<glm::vec3, 3> &face, const glm::vec3 &half_box_size) {
            auto v0 = face[0] - voxel_center;
            auto v1 = face[1] - voxel_center;

            float fa = std::abs(e.z);
            float fb = std::abs(e.y);

            float p1 = e.z * v0.y - e.y * v0.z;
            float p2 = e.z * v1.y - e.y * v1.z;

            float min = p1 > p2 ? p2 : p1;
            float max = p1 > p2 ? p1 : p2;

            float rad = (fa + fb) * half_box_size.z;
            if (min > rad || max < -rad) {
                return true;
            }
            return false;
        }

        inline bool axis_test_y02(const glm::vec3 &e, const glm::vec3 &voxel_center, const std::array<glm::vec3, 3> &face, const glm::vec3 &half_box_size) {
            auto v0 = face[0] - voxel_center;
            auto v2 = face[2] - voxel_center;

            float fa = std::abs(e.z);
            float fb = std::abs(e.x);

            float p1 = -e.z * v0.x + e.x * v0.z;
            float p2 = -e.z * v2.x + e.x * v2.z;

            float min = p1 > p2 ? p2 : p1;
            float max = p1 > p2 ? p1 : p2;

            float rad = (fa + fb) * half_box_size.z;
            if (min > rad || max < -rad) {
                return true;
            }
            return false;
        }
        inline bool axis_test_y01(const glm::vec3 &e, const glm::vec3 &voxel_center, const std::array<glm::vec3, 3> &face, const glm::vec3 &half_box_size) {
            auto v0 = face[0] - voxel_center;
            auto v1 = face[1] - voxel_center;

            float fa = std::abs(e.z);
            float fb = std::abs(e.x);

            float p1 = -e.z * v0.x + e.x * v0.z;
            float p2 = -e.z * v1.x + e.x * v1.z;

            float min = p1 > p2 ? p2 : p1;
            float max = p1 > p2 ? p1 : p2;

            float rad = (fa + fb) * half_box_size.z;
            if (min > rad || max < -rad) {
                return true;
            }
            return false;
        }

        inline bool axis_test_z12(const glm::vec3 &e, const glm::vec3 &voxel_center, const std::array<glm::vec3, 3> &face, const glm::vec3 &half_box_size) {
            auto v1 = face[1] - voxel_center;
            auto v2 = face[2] - voxel_center;

            float fa = std::abs(e.y);
            float fb = std::abs(e.x);

            float p1 = e.y * v1.x - e.x * v1.y;
            float p2 = e.y * v2.x - e.x * v2.y;

            float min = p1 > p2 ? p2 : p1;
            float max = p1 > p2 ? p1 : p2;

            float rad = (fa + fb) * half_box_size.z;
            if (min > rad || max < -rad) {
                return true;
            }
            return false;
        }
        inline bool axis_test_z01(const glm::vec3 &e, const glm::vec3 &voxel_center, const std::array<glm::vec3, 3> &face, const glm::vec3 &half_box_size) {
            auto v1 = face[0] - voxel_center;
            auto v2 = face[1] - voxel_center;

            float fa = std::abs(e.y);
            float fb = std::abs(e.x);

            float p1 = e.y * v1.x - e.x * v1.y;
            float p2 = e.y * v2.x - e.x * v2.y;

            float min = p1 > p2 ? p2 : p1;
            float max = p1 > p2 ? p1 : p2;

            float rad = (fa + fb) * half_box_size.z;
            if (min > rad || max < -rad) {
                return true;
            }
            return false;
        }

        inline bool allGreaterEqualThan(const std::array<float, 3> &v, const float c) {
            for(auto &p : v) {
                if(p <= c) return false;
            }
            return true;
        }

        inline bool allSmallerEqualThan(const std::array<float, 3> &v, const float c) {
            for(auto &p : v) {
                if(p >= c) return false;
            }
            return true;
        }

        inline bool plane_box_overlap(const hidden::plane &plane, const glm::vec3 &voxel_center, const glm::vec3 &halfboxsize) {
            glm::vec3 vmax = voxel_center;
            glm::vec3 vmin = voxel_center;
            for (int dim = 0; dim < 3; dim++) {
                if (plane.normal[dim] > 0) {
                    vmin[dim] += -halfboxsize[dim];
                    vmax[dim] += halfboxsize[dim];
                }
                else {
                    vmin[dim] += halfboxsize[dim];
                    vmax[dim] += -halfboxsize[dim];
                }
            }

            if (glm::dot(plane.normal, vmin) + plane.d > 0) {
                return false;
            }
            if (glm::dot(plane.normal, vmax) + plane.d >= 0) {
                return true;
            }
            return false;
        }

        //! return true if outside
        //! return false if maybe inside
        inline bool face_in_hexahedron(const std::array<glm::vec3, 3> &face, const glm::vec3 voxel_center, const glm::vec3 &half_box_size) {
            const glm::vec3 e1 = face[2] - face[1];
            const glm::vec3 e2 = face[0] - face[2];

            const glm::vec3 norm = glm::cross(e1, e2);
            if(!plane_box_overlap({norm, -glm::dot(norm, face[0])}, voxel_center, half_box_size)) {
                return false;
            }

            const glm::vec3 e0 = face[1] - face[0];
            if(axis_test_x02(e0, voxel_center, face, half_box_size)) return false;
            if(axis_test_x02(e1, voxel_center, face, half_box_size)) return false;
            if(axis_test_x01(e2, voxel_center, face, half_box_size)) return false;

            if(axis_test_y02(e0, voxel_center, face, half_box_size)) return false;
            if(axis_test_y02(e1, voxel_center, face, half_box_size)) return false;
            if(axis_test_y01(e2, voxel_center, face, half_box_size)) return false;

            if(axis_test_z12(e0, voxel_center, face, half_box_size)) return false;
            if(axis_test_z01(e1, voxel_center, face, half_box_size)) return false;
            if(axis_test_z12(e2, voxel_center, face, half_box_size)) return false;

            const auto max = voxel_center + half_box_size;
            if(allGreaterEqualThan({face[0].x, face[1].x, face[2].x}, max.x)) return false;
            if(allGreaterEqualThan({face[0].y, face[1].y, face[2].y}, max.y)) return false;
            if(allGreaterEqualThan({face[0].z, face[1].z, face[2].z}, max.z)) return false;

            const auto min = voxel_center - half_box_size;
            if(allSmallerEqualThan({face[0].x, face[1].x, face[2].x}, min.x)) return false;
            if(allSmallerEqualThan({face[0].y, face[1].y, face[2].y}, min.y)) return false;
            if(allSmallerEqualThan({face[0].z, face[1].z, face[2].z}, min.z)) return false;

            return true;
        }

        // for voxel shell calculation we invert the test
        // we do not check wheter a voxel is in the mesh, but whether a face is in the bbox of the voxel
        template<typename base_t>
        bool is_shell(const glm::vec3 &pos, const mesh::polyhedron<base_t> &poly, const glm::vec3 &voxel_size) {
            const auto &vertex_arr =  poly._vertices._vertex_arr;
            const auto &index_buf = poly._indices._buffer;
            const size_t faces = poly._indices._buffer.size() / poly._indices._stride;
            const glm::vec3 half_box_size = voxel_size / 2.f;

            for(size_t face_id = 0; face_id < faces; face_id++) {
                // walk over faces
                const glm::ivec3 id = glm::ivec3(face_id) * poly._indices._stride + glm::ivec3(0,1,2);
                const size_t vid1 = index_buf.at(id.x);
                const size_t vid2 = index_buf.at(id.y);
                const size_t vid3 = index_buf.at(id.z);

                // face vertices
                const std::array<glm::vec3, 3> face = {
                    vertex_arr.at(vid1),
                    vertex_arr.at(vid2),
                    vertex_arr.at(vid3)
                };
                if(face_in_hexahedron(face, pos, half_box_size)) return true;
            }

            return false;
        }
    };
};
