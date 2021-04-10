#pragma once

#include "polyhedron/mesh/polyhedron.h"
#include "polyhedron/glm_ext/glm_extensions.h"

#include "index_buffer.h"
#include "raster_results.h"
#include "checks.h"
#include "buffer.h"
#include "timer.h"

#include <set>
#include <unordered_set>
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
        
        size_t seed_size = 128;
        glm::ivec3 _seed = {0, 0, 0};

        // casts rays along a world coordinate plane: xy, yz, xz
        // faster then checking every cell, but causing issues with infill
        template <swizzle_mode mode> void planar_raycast(auto &buf) {
            benchmark::timer tmp("planar_raycast()");

            glm::ivec2 dim;
            if constexpr(mode == yzx)
                dim = {_dim.y,_dim.z};
            if constexpr(mode == xzy)
                dim = {_dim.x,_dim.z};
            if constexpr(mode == xyz)
                dim = {_dim.x,_dim.y};

#pragma omp parallel for
            for(int i = 0; i < dim.x; i++)
            for(int j = 0; j < dim.y; j++) {
                std::set<int> inters;
                if constexpr(mode == yzx)
                    inters = checks::raycast::get_intersections_fast<mode>(glm::vec2(i,j), _polyhedron._vertices, _yz_plane_buffer[i][j]);
                if constexpr(mode == xzy)
                    inters = checks::raycast::get_intersections_fast<mode>(glm::vec2(i,j), _polyhedron._vertices, _xz_plane_buffer[i][j]);
                if constexpr(mode == xyz)
                    inters = checks::raycast::get_intersections_fast<mode>(glm::vec2(i,j), _polyhedron._vertices, _xy_plane_buffer[i][j]);

                bool is_in = false;
                int from = -1;
                for(int inters : inters) {
                    // 1) assign shell voxels
                    if constexpr(mode == yzx) {
                        inters = constrain(0, _dim.x-1, inters);
                        buf._voxels[inters][i][j] = voxel_type::shell;
                    }
                    if constexpr(mode == xzy) {
                        inters = constrain(0, _dim.y-1, inters);
                        buf._voxels[i][inters][j] = voxel_type::shell;
                    }
                    if constexpr(mode == xyz) {
                        inters = constrain(0, _dim.z-1, inters);
                        buf._voxels[i][j][inters] = voxel_type::shell;
                    }

                    // 2) assign interior voxels by bit shift 
                    // here we run from shell voxel to shell voxel
                    // after three bit shifts the voxel is equal to voxel_type::interior
                    if(from > -1) {
                        for(int k = from; k < inters; k++) {
                            if(!is_in) continue;
                            if constexpr(mode == yzx) {
                                uint8_t &vox = buf._voxels[k][i][j];
                                if(vox == voxel_type::shell) continue;
                                vox = (vox != 0) ? vox << 1 : 1;
                            }
                            else if constexpr(mode == xzy) {
                                uint8_t &vox = buf._voxels[i][k][j];
                                if(vox == voxel_type::shell) continue;
                                vox = (vox != 0) ? vox << 1 : 1;
                            }
                            else if constexpr(mode == xyz) {
                                uint8_t &vox = buf._voxels[i][j][k];
                                if(vox == voxel_type::shell) continue;
                                vox = (vox != 0) ? vox << 1 : 1;
                            }

                            // save some voxel positions as seeds and try to space them a bit
                            bool seed_valid = buf._voxels[_seed.x][_seed.y][_seed.z] == voxel_type::interior;
                            if(!seed_valid) {
                                glm::ivec3 pos;
                                if constexpr(mode == yzx) pos = {k,i,j};
                                if constexpr(mode == xzy) pos = {i,k,j};
                                if constexpr(mode == xyz) pos = {i,j,k};
                                _seed = pos;
                            }
                        }
                    }
                    from = inters+1;
                    is_in = !is_in;
                }
            }
        }

    public:
        using voxel_data_t = raster_results<buffer3d<uint8_t>, mesh::polyhedron<base_t>>;

        solid_fast(const mesh::polyhedron<base_t> &poly, glm::ivec3 dim) {
            double min_face_area = poly.avg_face_area() / 10;
            _polyhedron = prepare_index_buffers(poly, dim, _xy_plane_buffer, _yz_plane_buffer, _xz_plane_buffer, min_face_area);
            _dim = glm::ceil(_polyhedron.dim());
        }

        voxel_data_t rasterize() {
            benchmark::timer tmp("rasterize()");
            voxel_data_t buf(_dim, buffer3d<uint8_t>(_dim.x, _dim.y, _dim.z, 0), _polyhedron);
            planar_raycast<yzx>(buf);
            planar_raycast<xzy>(buf);
            planar_raycast<xyz>(buf);
            return buf;
        }
    };
};
