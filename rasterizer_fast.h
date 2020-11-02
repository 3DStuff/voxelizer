#pragma once

#include "polyhedron/mesh/polyhedron.h"
#include "polyhedron/glm_ext/glm_extensions.h"

#include "index_buffer.h"
#include "raster_results.h"
#include "checks.h"
#include "buffer.h"
#include "timer.h"

#include <set>
#include <omp.h>


namespace rasterize {
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
            double min_face_area = poly.avg_face_area() / 10;
            _polyhedron = prepare_index_buffers(poly, dim, _xy_plane_buffer, _yz_plane_buffer, _xz_plane_buffer, min_face_area);
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
};
