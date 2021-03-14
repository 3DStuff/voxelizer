#pragma once

#include "polyhedron/glm_ext/glm_extensions.h"
#include "polyhedron/io/stl/stl_import.h"
#include "polyhedron/io/obj/obj_import.h"
#include "polyhedron/mesh/polyhedron.h"
#include "polyhedron/mesh/bbox.h"
#include "xml_config.h"
#include "vox_file.h"
#include "rle/rle_io.h"
#include "merge.h"
#include "timer.h"

#include <cassert>
#include <fstream>
#include <vector>
#include <map>
#include <filesystem>


namespace voxelize {
    struct project_cfg {
        cfg::shape_settings xml;
        mesh::polyhedron<float> mesh;
        mesh::bbox<float> loc_bbox;
        mesh::bbox<float> glo_bbox;

        project_cfg(
            const cfg::shape_settings &settings, 
            const mesh::polyhedron<float> &poly, 
            const mesh::bbox<float> &lbbox = mesh::bbox<float>(), 
            const mesh::bbox<float> &gbbox = mesh::bbox<float>()
        ) : xml(settings), mesh(poly), loc_bbox(lbbox), glo_bbox(gbbox) {}

        //! clear mesh and rasterization data
        void clear() {
            mesh.clear();
        }
    };

    template <typename rasterizer_t>
    class voxelizer {
        using raster_out_t = typename rasterizer_t::voxel_data_t;

        cfg::xml_project _project_cfg;
        std::vector<project_cfg> _rasterizer_res;
        mesh::bbox<float> _glob_bbox;

    private:
        //! calculates the project bbox
        //! buffers the meshes after creation
        bool load();
        
    public:
        voxelizer(const cfg::xml_project &cfg);
        std::vector<uint8_t> to_bytes(const project_cfg &, const raster_out_t &) const;
        void clear();
        void run();

        // TODO not implemented yet
        template<typename rule_t> void to_fs_obj(const project_cfg &, const raster_out_t &) const;
        template<typename rule_t> void to_fs_stl(const project_cfg &, const raster_out_t &) const;
        void to_fs_bytes(const project_cfg &, const raster_out_t &) const;
        void to_fs_rle(const project_cfg &, const raster_out_t &) const;
        void to_fs_vox(const project_cfg &, const raster_out_t &) const;

        void to_fs(const project_cfg &, const raster_out_t &) const;
    };

    #include "voxelizer.ipp"
};

