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
    template <typename data_t>
    class intersections {
        // [][][] operator madness
        using parent_t = intersections<data_t>;
        struct proxy_i {
            struct proxy_ii {
                int _x;
                int _y;
                const parent_t *_p = nullptr;
                proxy_ii(const parent_t *p, const int x, const int y) : _x(x), _y(y), _p(p) {}
                uint8_t operator[] (const int z) const {
                    return _p->get(_x, _y, z);
                }
            };
            int _x;
            const parent_t *_p = nullptr;
            proxy_i(const parent_t *p, const int x) : _x(x), _p(p) {}
            proxy_ii operator[] (const int y) const {
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
        
        using buf_t = buffer3d<data_t>;
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
        
        uint8_t get(const int x, const int y, const int z) const {
            // check whether a position is in between intersections
            auto is_in = [](const auto &arr, const int pos) {
                if(arr.size() == 0) return false;
                if(arr.size() == 1) return false;
                bool is_out = arr.size() % 2 == 0 ? true : false;                               
                for(size_t i = 0; i < arr.size()-1; i++) {   
                    if(pos > arr[i] && pos < arr[i+1]) {
                        return is_out;
                    }
                    is_out = !is_out;
                }
                return false;
            };
            auto not_in_shell = [](const auto &arr, const int x, const int y) {
                if(x < 0) return false;
                if(y < 0) return false;
                assert(arr.size() <= std::numeric_limits<int>::max());
                if(x >= (int)arr.size()-1) return false;
                assert(arr[x].size() <= std::numeric_limits<int>::max());
                if(y >= (int)arr[x].size()-1) return false;
                return true;
            };

            // is shell?
            for(int dist : yz[y][z]) {
                dist = constrain(0, _dim.x-1, dist);
                if(dist == x) return voxel_type::shell;
            }
            for(int dist : xz[x][z]) {
                dist = constrain(0, _dim.y-1, dist);
                if(dist == y) return voxel_type::shell;
            }
            for(int dist : xy[x][y]) {
                dist = constrain(0, _dim.z-1, dist);
                if(dist == z) return voxel_type::shell;
            }

            // is interior?
            int cnt = 0;
            if(not_in_shell(yz, y, z))
                if(is_in(yz[y][z], x)) cnt++;

            if(not_in_shell(xz, x, z))
                if(is_in(xz[x][z], y)) cnt++;

            if(not_in_shell(xy, x, y))
                if(is_in(xy[x][y], z)) cnt++;

            if(cnt >= 2)
                return voxel_type::interior;
            else 
                return voxel_type::background;
        }
        
        proxy_i operator[] (const int x) const {
            return proxy_i(this, x);
        }
    };

    //! generates a voxel mesh from an arbitrary polyhedron
    //! save some memory by usage of quad trees and avoiding allocation of a 3d buffer
    //! by storing just the distances of ray intersections
    template<typename base_t = float>
    class solid_safe {
        using id_t = typename mesh::polyhedron<base_t>::index_t;
        using inters_t = intersections<int>;
        
        mesh::polyhedron<base_t> _polyhedron;
        glm::vec<3, base_t> _polyhedron_dim;
        
        tree<quad::boundary, mesh::face<id_t>> _xy_tree;
        tree<quad::boundary, mesh::face<id_t>> _yz_tree;
        tree<quad::boundary, mesh::face<id_t>> _xz_tree;
        
        // scale factors for quad trees
        glm::vec<3, base_t> _tree_search_steps = glm::vec<3, base_t>(1);
        float _tree_scale = 1;
        
    public:
        using voxel_data_t = raster_results<inters_t, mesh::polyhedron<base_t>>;
        
        solid_safe(const mesh::polyhedron<base_t> &poly, glm::ivec3 scale) {
            benchmark::timer t("Generate quad trees");
            
            _polyhedron = mesh::polyhedron<base_t>::normalized(poly);
            _polyhedron = mesh::polyhedron<base_t>::scaled(_polyhedron, scale);
            
            // use _polyhedron_dim to allocate the intersection arrays
            // add one to the dimension:
            // if a face is exactly on the end of the array, we cannot draw a voxel anymore
            _polyhedron_dim = glm::ceil(_polyhedron.dim());
            const glm::vec<3, base_t> dim_half = _polyhedron_dim / 2.f;

            _xy_tree = tree<quad::boundary, mesh::face<id_t>>(quad::boundary(dim_half.xy()*_tree_scale, dim_half.xy()*_tree_scale), 16);
            _yz_tree = tree<quad::boundary, mesh::face<id_t>>(quad::boundary(dim_half.yz()*_tree_scale, dim_half.yz()*_tree_scale), 16);
            _xz_tree = tree<quad::boundary, mesh::face<id_t>>(quad::boundary(dim_half.xz()*_tree_scale, dim_half.xz()*_tree_scale), 16);

            // build quad trees
#pragma omp parallel sections
{
#pragma omp section
{
            for (const auto &f : _polyhedron._indices._buffer) {
                const glm::vec<3, base_t> v1 = _polyhedron._vertices[f[0]] * _tree_scale;
                const glm::vec<3, base_t> v2 = _polyhedron._vertices[f[1]] * _tree_scale;
                const glm::vec<3, base_t> v3 = _polyhedron._vertices[f[2]] * _tree_scale;
                
                glm::ivec3 min = glm::floor(glm::min(v1, glm::min(v2, v3)));
                glm::ivec3 max = glm::ceil(glm::max(v1, glm::max(v2, v3)));
                
                for(int x = min.x; x < max.x; x++)
                for(int y = min.y; y < max.y; y++) {
                    const bool is_in = checks::raycast::pt_in_triangle(glm::vec2(x,y), v1.xyz(), v2.xyz(), v3.xyz());
                    if(is_in) {
                        _xy_tree.insert(glm::vec2(x,y), f);
                    }
                }
            }
}
#pragma omp section
{
            for (const auto &f : _polyhedron._indices._buffer) {
                const glm::vec<3, base_t> v1 = _polyhedron._vertices[f[0]] * _tree_scale;
                const glm::vec<3, base_t> v2 = _polyhedron._vertices[f[1]] * _tree_scale;
                const glm::vec<3, base_t> v3 = _polyhedron._vertices[f[2]] * _tree_scale;
                
                glm::ivec3 min = glm::floor(glm::min(v1, glm::min(v2, v3)));
                glm::ivec3 max = glm::ceil(glm::max(v1, glm::max(v2, v3)));
                
                for(int y = min.y; y < max.y; y++)
                for(int z = min.z; z < max.z; z++) {
                    const bool is_in = checks::raycast::pt_in_triangle(glm::vec2(y,z), v1.yzx(), v2.yzx(), v3.yzx());
                    if(is_in) {
                        _yz_tree.insert(glm::vec2(y,z), f);
                    }
                }
            }
}
#pragma omp section 
{
            for (const auto &f : _polyhedron._indices._buffer) {
                const glm::vec<3, base_t> v1 = _polyhedron._vertices[f[0]] * _tree_scale;
                const glm::vec<3, base_t> v2 = _polyhedron._vertices[f[1]] * _tree_scale;
                const glm::vec<3, base_t> v3 = _polyhedron._vertices[f[2]] * _tree_scale;
                
                glm::ivec3 min = glm::floor(glm::min(v1, glm::min(v2, v3)));
                glm::ivec3 max = glm::ceil(glm::max(v1, glm::max(v2, v3)));
                
                for(int x = min.x; x < max.x; x++)
                for(int z = min.z; z < max.z; z++) {
                    const bool is_in = checks::raycast::pt_in_triangle(glm::vec2(x,z), v1.xzy(), v2.xzy(), v3.xzy());
                    if(is_in) {
                        _xz_tree.insert(glm::vec2(x,z), f);
                    }
                }
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
            
            // the quad tree is scaled up or down to 128 x Y x Z voxels
            // thus, we calc the scale factors to access the tree            
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
