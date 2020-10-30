#pragma once

#include "polyhedron/mesh/polyhedron.h"
#include "polyhedron/glm_ext/glm_extensions.h"

#include "raster_results.h"
#include "checks.h"
#include "buffer.h"
#include "timer.h"

#include <omp.h>


namespace rasterize {
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
