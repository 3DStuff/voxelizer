#pragma once

#include "polyhedron/stl/stl_import.h"
#include "polyhedron/mesh/polyhedron.h"
#include "polyhedron/glm_ext/glm_extensions.h"

#include "oqtree/oqtree.h"

#include "checks.h"
#include "xml_config.h"
#include "buffer.h"
#include "timer.h"
#include "enums.h"

#include <set>
#include <vector>


namespace rasterize {
    template<typename voxel_buf_t, typename mesh_t>
    struct raster_results {
        glm::vec3 _arr_dim;
        voxel_buf_t _voxels;
        mesh_t _mesh;
        
        raster_results() = default;
        raster_results(const glm::vec3 &dim, const voxel_buf_t &dta, const mesh_t &mesh) {
            _arr_dim = dim;
            _voxels = dta;
            _mesh = mesh;
        }

        void clear() {
            _voxels.clear();
            _mesh.clear();
        }
    };

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
                proxy_ii(const parent_t *p, const int x, const int y) : _p(p), _x(x), _y(y) {}
                uint8_t operator[] (const int z) const {
                    return _p->get(_x, _y, z);
                }
            };
            int _x;
            const parent_t *_p = nullptr;
            proxy_i(const parent_t *p, const int x) : _p(p), _x(x) {}
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
                for(int i = 0; i < arr.size()-1; i++) {   
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
                if(x >= arr.size()-1) return false;
                if(y >= arr[x].size()-1) return false;
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

    template<typename vec_t>
    float area(const vec_t &v1, const vec_t &v2, const vec_t &v3) {
        float le1 = glm::length(v2-v1);
        float le2 = glm::length(v3-v1);
        float le3 = glm::length(v3-v2);
        float p = 0.5 * (le1 + le2 + le3);
        float area = std::sqrt(p * (p - le1) * (p - le2) * (p - le3));
        return area;
    }
    
    template<typename base_t>
    mesh::polyhedron<base_t> prepare_index_buffers(
        const mesh::polyhedron<base_t> &in_mesh, 
        const glm::ivec3 &in_scale,
        buffer3d<mesh::face<id_t>> &out_xy, 
        buffer3d<mesh::face<id_t>> &out_yz,
        buffer3d<mesh::face<id_t>> &out_xz
    ) 
    {           
        benchmark::timer tmp("prepare_index_buffers()");
            
        mesh::polyhedron<base_t> mesh;
        mesh = mesh::polyhedron<base_t>::normalized(in_mesh);
        mesh = mesh::polyhedron<base_t>::scaled(mesh, in_scale);
        const glm::ivec3 dim = glm::ceil(mesh.dim());

        auto &indices = mesh._indices;
        out_xy = buffer3d<mesh::face<id_t>>(dim.x+1, dim.y+1, 0);
        out_yz = buffer3d<mesh::face<id_t>>(dim.y+1, dim.z+1, 0);
        out_xz = buffer3d<mesh::face<id_t>>(dim.x+1, dim.z+1, 0);
        
        // run over target voxel coordinates 
        // and check whether a faces cuts a posiion in one of the planes
        size_t num_elems = 0;
        for(const mesh::face<id_t> &f : indices._buffer) {
            const auto &v1 = mesh._vertices[f[0]];
            const auto &v2 = mesh._vertices[f[1]];
            const auto &v3 = mesh._vertices[f[2]];

            constexpr float area_bias = 0.1;
            const float a_xz = area(v1.xz(), v2.xz(), v3.xz());
            const float a_xy = area(v1.xy(), v2.xy(), v3.xy());
            const float a_yz = area(v1.yz(), v2.yz(), v3.yz());
            
            // calc bbox around the face
            // we use the bbox to create a simple search buffer (for later rasterization)
            const glm::ivec3 min = glm::floor(glm::min(glm::min(v1, v2), v3));
            const glm::ivec3 max = glm::ceil(glm::max(glm::max(v1, v2), v3));

            // using the bbox: 
            // insert the face indices into the buffers
            // do this for each major plane..
            // xy
            if(a_xy > area_bias) {
                for(int x = min.x; x <= max.x; x++)
                for(int y = min.y; y <= max.y; y++) {
                    out_xy[x][y].push_back(f);
                    num_elems += 3;
                }
            }
            // yz
            if(a_yz > area_bias) {
                for(int y = min.y; y <= max.y; y++)
                for(int z = min.z; z <= max.z; z++) {
                    out_yz[y][z].push_back(f);
                    num_elems += 3;
                }
            }
            // xz
            if(a_xz > area_bias) {
                for(int x = min.x; x <= max.x; x++)
                for(int z = min.z; z <= max.z; z++) {                 
                    out_xz[x][z].push_back(f);
                    num_elems += 3;
                }
            }
        }
        float s_mb = (float)((num_elems * sizeof(id_t)) / std::pow(1024, 2));
        std::cout << "search buffers are " << (size_t)s_mb << " MBytes" << std::endl;
        return mesh;
    }
    
    //! generates a voxel mesh from an arbitrary polyhedron
    //! is very fast but memory consumption does not so scale well
    //! because we use a search array and allocate complete a 3d buffer
    template<typename base_t = float>
    class solid_fast {
        using id_t = typename mesh::polyhedron<base_t>::index_t;
        
        mesh::polyhedron<base_t> _polyhedron;
        glm::ivec3 _dim;
        
        buffer3d<mesh::face<id_t>> _xy_plane_buffer;
        buffer3d<mesh::face<id_t>> _yz_plane_buffer;
        buffer3d<mesh::face<id_t>> _xz_plane_buffer;
        
    public:
        using voxel_data_t = raster_results<buffer3d<uint8_t>, mesh::polyhedron<base_t>>;
        
        solid_fast(const mesh::polyhedron<base_t> &poly, glm::ivec3 dim) {
            _polyhedron = prepare_index_buffers(poly, dim, _xy_plane_buffer, _yz_plane_buffer, _xz_plane_buffer);
            _dim = glm::ceil(_polyhedron.dim());
        }
        
        voxel_data_t rasterize() const {
            raster_results res(_dim, buffer3d<uint8_t>(_dim.x, _dim.y, _dim.z, 0), _polyhedron);
        
            // create timer object
            benchmark::timer tmp("rasterize()");
            
#pragma omp parallel for
            for(int y = 0; y < _dim.y; y++)
            for(int z = 0; z < _dim.z; z++) {
                std::set<int> inters = checks::raycast::get_intersections_fast<yzx>(glm::vec2(y,z), _polyhedron._vertices, _yz_plane_buffer[y][z]);
                bool is_in = false;
                int from = -1;
                
                for(int inters : inters) {
                    inters = constrain(0, _dim.x-1, inters);
                    res._voxels[inters][y][z] = voxel_type::shell;
                    
                    if(from > -1)
                        for(int i = from; i < inters; i++) {
                            if(res._voxels[i][y][z] == voxel_type::shell) continue;
                            res._voxels[i][y][z] = is_in; // initially set voxel to 1
                        }
                    
                    from = inters+1;
                    is_in = !is_in;
                }
            }
#pragma omp parallel for
            for(int x = 0; x < _dim.x; x++)
            for(int z = 0; z < _dim.z; z++) {
                std::set<int> inters = checks::raycast::get_intersections_fast<xzy>(glm::vec2(x,z), _polyhedron._vertices, _xz_plane_buffer[x][z]);
                bool is_in = false;
                int from = -1;
                
                for(int inters : inters) {
                    inters = constrain(0, _dim.y-1, inters);
                    res._voxels[x][inters][z] = voxel_type::shell;
                    
                    if(from > -1)
                        for(int i = from; i < inters; i++) {
                            if(res._voxels[x][i][z] == voxel_type::shell) continue;
                            res._voxels[x][i][z] <<= is_in; // first bit shift; now the value should be either 0 or voxel_type::interior
                        }
                    
                    from = inters+1;
                    is_in = !is_in;
                }
            }
#pragma omp parallel for
            for(int x = 0; x < _dim.x; x++)
            for(int y = 0; y < _dim.y; y++) {
                std::set<int> inters = checks::raycast::get_intersections_fast<xyz>(glm::vec2(x,y), _polyhedron._vertices, _xy_plane_buffer[x][y]);
                bool is_in = false;
                int from = -1;
                
                for(int inters : inters) {
                    inters = constrain(0, _dim.z-1, inters);
                    res._voxels[x][y][inters] = voxel_type::shell;
                    
                    if(from > -1)
                        for(int i = from; i < inters; i++) {
                            if(res._voxels[x][y][i] == voxel_type::shell) continue;
                            res._voxels[x][y][i] <<= is_in; // second bit shift; now the value should be either 0 or voxel_type::interior
                        }
                    
                    from = inters+1;
                    is_in = !is_in;
                }
            }
            return res;
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
    
    //! generates a voxel mesh from an arbitrary polyhedron
    template<typename base_t = float>
    class hollow {
        mesh::polyhedron<base_t> _polyhedron;
        
    public:
        using voxel_data_t = raster_results<buffer3d<uint8_t>, mesh::polyhedron<base_t>>;
        
        hollow(const mesh::polyhedron<base_t> &poly, const glm::ivec3 &dim) {
            _polyhedron = poly;
            _polyhedron.normalize();
            _polyhedron.scale(dim);
        }
        
        voxel_data_t rasterize() const {
            benchmark::timer tmp("rasterize()");
            
            const glm::ivec3 dim = glm::ceil(_polyhedron.dim());
            raster_results res(dim, buffer3d<uint8_t>(dim.x, dim.y, dim.z, 0), _polyhedron);
            
            const auto &index_buf = _polyhedron._indices._buffer;
            const auto &vertex_buffer = _polyhedron._vertices;
            
            for(mesh::face<uint32_t> f : index_buf) {
                std::array<glm::vec<3, base_t>, 3> face = {
                    vertex_buffer[f[0]],
                    vertex_buffer[f[1]],
                    vertex_buffer[f[2]]
                };

                const glm::vec3 lmin = glm::floor(glm::min(face[2], glm::min(face[0], face[1]))) - glm::vec3(0.5f);
                const glm::vec3 lmax = glm::ceil(glm::max(face[2], glm::max(face[0], face[1]))) + glm::vec3(0.5f);
                const glm::ivec3 lsteps = lmax - lmin;

                face[0] -= lmin;
                face[1] -= lmin;
                face[2] -= lmin;

                for(int x = 0; x < lsteps.x; x++)
                for(int y = 0; y < lsteps.y; y++)
                for(int z = 0; z < lsteps.z; z++) {
                    int nx = x + lmin.x;
                    int ny = y + lmin.y;
                    int nz = z + lmin.z;
                    
                    if(res._voxels[nx][ny][nz]) continue;
                    if(checks::intersection_3d::face_in_hexahedron(face, {x,y,z}, glm::vec3(0.5))) {
                        res._voxels[nx][ny][nz] = voxel_type::shell;
                    }
                }
            }
            return res;
        }
    };
};
