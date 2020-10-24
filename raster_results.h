#pragma once

#include "polyhedron/glm_ext/glm_extensions.h"


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
};