#pragma once

#include "polyhedron/mesh/polyhedron.h"
#include "polyhedron/glm_ext/glm_extensions.h"
#include "buffer.h"

#include <iostream>


namespace rasterize {
    template<typename base_t, typename id_t>
    mesh::polyhedron<base_t> prepare_index_buffers(
        const mesh::polyhedron<base_t> &in_mesh, 
        const glm::ivec3 &in_scale,
        buffer3d<mesh::face<id_t>> &out_xy, 
        buffer3d<mesh::face<id_t>> &out_yz,
        buffer3d<mesh::face<id_t>> &out_xz,
        double min_face_area = 0.1
    ) 
    {           
        benchmark::timer tmp("prepare_index_buffers()");
            
        auto area = [](const auto &v1, const auto &v2, const auto &v3) {
            double le1 = glm::length(v2-v1);
            double le2 = glm::length(v3-v1);
            double le3 = glm::length(v3-v2);
            double p = 0.5 * (le1 + le2 + le3);
            double area = std::sqrt(p * (p - le1) * (p - le2) * (p - le3));
            return area;
        };

        mesh::polyhedron<base_t> mesh;
        mesh = mesh::polyhedron<base_t>::normalized(in_mesh);
        mesh = mesh::polyhedron<base_t>::scaled(mesh, in_scale);
        const glm::ivec3 dim = glm::ceil(mesh.dim());

        auto &indices = mesh._indices;
        out_xy = buffer3d<mesh::face<id_t>>(dim.x+1, dim.y+1, 0);
        out_yz = buffer3d<mesh::face<id_t>>(dim.y+1, dim.z+1, 0);
        out_xz = buffer3d<mesh::face<id_t>>(dim.x+1, dim.z+1, 0);

        // run over target voxel coordinates 
        // and check whether a faces cuts a position in one of the planes
        size_t num_elems = 0;
        for(const mesh::face<id_t> &f : indices._buffer) {
            const auto &v1 = mesh._vertices[f[0]];
            const auto &v2 = mesh._vertices[f[1]];
            const auto &v3 = mesh._vertices[f[2]];

            const double a_xz = area(v1.xz(), v2.xz(), v3.xz());
            const double a_xy = area(v1.xy(), v2.xy(), v3.xy());
            const double a_yz = area(v1.yz(), v2.yz(), v3.yz());
            
            // calc bbox around the face
            // we use the bbox to create a simple search buffer (for later rasterization)
            const glm::ivec3 min = glm::floor(glm::min(glm::min(v1, v2), v3));
            const glm::ivec3 max = glm::ceil(glm::max(glm::max(v1, v2), v3));

            // using the bbox: 
            // insert the face indices into the buffers
            // do this for each major plane..
            // xy
            if(a_xy > min_face_area) {
                for(int x = min.x; x <= max.x; x++)
                for(int y = min.y; y <= max.y; y++) {
                    out_xy[x][y].push_back(f);
                    num_elems += 3;
                }
            }
            // yz
            if(a_yz > min_face_area) {
                for(int y = min.y; y <= max.y; y++)
                for(int z = min.z; z <= max.z; z++) {
                    out_yz[y][z].push_back(f);
                    num_elems += 3;
                }
            }
            // xz
            if(a_xz > min_face_area) {
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
};