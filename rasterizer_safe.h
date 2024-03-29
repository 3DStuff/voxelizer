#pragma once

#include "polyhedron/mesh/polyhedron.h"
#include "polyhedron/glm_ext/glm_extensions.h"

#include "index_buffer.h"
#include "oqtree/oqtree.h"
#include "raster_results.h"
#include "checks.h"
#include "buffer.h"
#include "timer.h"

#include <set>
#include <omp.h>


namespace rasterize {
    //! buffer for the intersections
    //! used by rasterize::all_rle 
    //! for memory optimization
    template <typename integral_t>
    class intersections {
        // [][][] operator madness
        using parent_t = intersections<integral_t>;
        struct proxy_i {
            struct proxy_ii {
                integral_t _x;
                integral_t _y;
                const parent_t *_p = nullptr;
                proxy_ii(const parent_t *p, const integral_t x, const integral_t y) : _x(x), _y(y), _p(p) {}
                uint8_t operator[] (const integral_t z) const {
                    return _p->get(_x, _y, z);
                }
            };
            integral_t _x;
            const parent_t *_p = nullptr;
            proxy_i(const parent_t *p, const integral_t x) : _x(x), _p(p) {}
            proxy_ii operator[] (const integral_t y) const {
                return proxy_ii(_p, _x, y);
            }
        };
        
        glm::ivec3 _dim;
        
    public:
        intersections() = default;
        intersections(const glm::ivec3 &dim) {
            _dim = dim;
            yz = buf_t(dim.y, dim.z, 0);
            xz = buf_t(dim.x, dim.z, 0);
            xy = buf_t(dim.x, dim.y, 0);
        }
        
        using buf_t = buffer3d<integral_t>;
        //! xy plane
        buf_t xy;
        //! yz plane
        buf_t yz;
        //! xz plane
        buf_t xz;

        void clear() {
            yz.clear();
            yz.clear();
            xz.clear();
        }
        
        uint8_t get(const integral_t x, const integral_t y, const integral_t z) const {
            // check whether a position is in between intersections
            auto is_in = [](const auto &arr, const integral_t pos) {
                if(arr.size() < 2) {
                    return false;
                }

                bool is_out = arr.size() % 2 == 0 ? true : false;                               
                for(size_t i = 0; i < arr.size()-1; i++) {   
                    if(pos > arr[i] && pos < arr[i+1]) {
                        return is_out;
                    }
                    is_out = !is_out;
                }
                return false;
            };
            auto not_in_shell = [](const auto &arr, const integral_t lx, const integral_t ly) {
                if(lx < (integral_t)arr.size()) {
                    return true;
                }
                if(ly < (integral_t)arr[lx].size()) {
                    return true;
                }
                return false;
            };

            // is shell?
            for(const auto &dist : yz[y][z]) {
                if(dist == x) return voxel_type::shell;
            }
            for(const auto &dist : xz[x][z]) {
                if(dist == y) return voxel_type::shell;
            }
            for(const auto &dist : xy[x][y]) {
                if(dist == z) return voxel_type::shell;
            }

            // is interior?
            if(not_in_shell(yz, y, z) && !is_in(yz[y][z], x)) return voxel_type::background;
            if(not_in_shell(xz, x, z) && !is_in(xz[x][z], y)) return voxel_type::background;
            if(not_in_shell(xy, x, y) && !is_in(xy[x][y], z)) return voxel_type::background;
            return voxel_type::interior;
        }
        
        proxy_i operator[] (const integral_t x) const {
            return proxy_i(this, x);
        }
    };

    //! generates a voxel mesh from an arbitrary polyhedron
    //! save some memory by usage of quad trees and avoiding allocation of a 3d buffer
    //! by storing just the distances of ray intersections
    template<typename base_t = float>
    class solid_safe {
        using id_t = typename mesh::polyhedron<base_t>::index_t;
        using inters_t = intersections<uint32_t>;
        
        mesh::polyhedron<base_t> _polyhedron;
        glm::vec<3, base_t> _polyhedron_dim;
        
        tree<quad::boundary, mesh::face<id_t>> _xy_tree;
        tree<quad::boundary, mesh::face<id_t>> _yz_tree;
        tree<quad::boundary, mesh::face<id_t>> _xz_tree;
        
        // scale factors for quad trees
        glm::vec<3, base_t> _tree_search_steps = glm::vec<3, base_t>(1);
        const float _tree_scale = 2.f;
        const size_t _branch_size = 128;
        
    public:
        using voxel_data_t = raster_results<inters_t, mesh::polyhedron<base_t>>;
        
        solid_safe(const mesh::polyhedron<base_t> &poly, glm::ivec3 scale) {
            benchmark::timer t("Generate quad trees");
            
            _polyhedron = mesh::polyhedron<base_t>::normalized(poly);
            _polyhedron = mesh::polyhedron<base_t>::scaled(_polyhedron, scale);
            
            // if a face is exactly on the end of the array, we cannot draw a voxel anymore: add one to dim
            _polyhedron_dim = glm::ceil(_polyhedron.dim()) + 1.f;
            const glm::vec<3, base_t> dim_half = _polyhedron_dim / 2.f;

            _xy_tree = tree<quad::boundary, mesh::face<id_t>>(quad::boundary(dim_half.xy()*_tree_scale, dim_half.xy()*_tree_scale), _branch_size);
            _yz_tree = tree<quad::boundary, mesh::face<id_t>>(quad::boundary(dim_half.yz()*_tree_scale, dim_half.yz()*_tree_scale), _branch_size);
            _xz_tree = tree<quad::boundary, mesh::face<id_t>>(quad::boundary(dim_half.xz()*_tree_scale, dim_half.xz()*_tree_scale), _branch_size);

            for (const auto &f : _polyhedron._indices._buffer) {
                const glm::vec<3, base_t> v1 = _polyhedron._vertices[f[0]] * _tree_scale;
                const glm::vec<3, base_t> v2 = _polyhedron._vertices[f[1]] * _tree_scale;
                const glm::vec<3, base_t> v3 = _polyhedron._vertices[f[2]] * _tree_scale;
                
                const glm::ivec3 min = glm::floor(glm::min(v1, glm::min(v2, v3)));
                const glm::ivec3 max = glm::ceil(glm::max(v1, glm::max(v2, v3)));
#pragma omp parallel sections 
{
#pragma omp section
                for(int x = min.x; x <= max.x; x++)
                for(int y = min.y; y <= max.y; y++) {
                    _xy_tree.insert({x,y}, f);
                }
#pragma omp section
                for(int x = min.x; x <= max.x; x++)
                for(int z = min.z; z <= max.z; z++) {
                    _xz_tree.insert({x,z}, f);
                }
#pragma omp section
                for(int y = min.y; y <= max.y; y++)
                for(int z = min.z; z <= max.z; z++) {
                    _yz_tree.insert({y,z}, f);
                }
}
            }

#ifdef DEBUG
            draw_quad_tree(_xy_tree);
            draw_quad_tree(_yz_tree);
            draw_quad_tree(_xz_tree);
#endif
        }
        
        voxel_data_t rasterize() const {
            // create timer object
            benchmark::timer tmp("rasterize()");       
            inters_t r(_polyhedron_dim);
#pragma omp parallel
{
#pragma omp for nowait
            for(int y = 0; y < (int)_polyhedron_dim.y; y++)
            for(int z = 0; z < (int)_polyhedron_dim.z; z++) {
                glm::vec2 p = glm::vec2(y, z) * _tree_scale;
                auto buffer = _yz_tree.find(p, _tree_search_steps);
                std::set<int> inters = checks::raycast::get_intersections_safe<yzx>(glm::vec2(y,z), _polyhedron._vertices, buffer);                
                std::copy(inters.begin(), inters.end(), std::back_inserter(r.yz[y][z]));
            }

#pragma omp for nowait
            for(int x = 0; x < (int)_polyhedron_dim.x; x++)
            for(int z = 0; z < (int)_polyhedron_dim.z; z++) {
                glm::vec2 p = glm::vec2(x, z) * _tree_scale;
                auto buffer = _xz_tree.find(p, _tree_search_steps);
                std::set<int> inters = checks::raycast::get_intersections_safe<xzy>(glm::vec2(x,z), _polyhedron._vertices, buffer);
                std::copy(inters.begin(), inters.end(), std::back_inserter(r.xz[x][z]));
            }

#pragma omp for nowait
            for(int x = 0; x < (int)_polyhedron_dim.x; x++)
            for(int y = 0; y < (int)_polyhedron_dim.y; y++) {
                glm::vec2 p = glm::vec2(x, y) * _tree_scale;
                auto buffer = _xy_tree.find(p, _tree_search_steps);
                std::set<int> inters = checks::raycast::get_intersections_safe<xyz>(glm::vec2(x,y), _polyhedron._vertices, buffer);
                std::copy(inters.begin(), inters.end(), std::back_inserter(r.xy[x][y]));
            }
}
            return voxel_data_t(_polyhedron_dim, r, _polyhedron);
        }
    };
};
