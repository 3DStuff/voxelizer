#pragma once

#include "polyhedron/glm_ext/glm_extensions.h"
#include "polyhedron/mesh/polyhedron.h"
#include "polyhedron/stl/stl_import.h"

#include "oqtree/oqtree.h"

#include "checks.h"
#include "xml_config.h"
#include "rules.h"
#include "buffer.h"
#include "timer.h"
#include "enums.h"
#include "rasterizer.h"
#include "vox_file.h"

#include <string>
#include <set>
#include <map>
#include <vector>
#include <tuple>
#include <filesystem>
#include <fstream>


namespace voxelize {
    template <typename rasterizer_t>
    class voxelizer {
        cfg::xml_project _project_cfg;
        
        struct voxel_data_t {
            mesh::polyhedron<float> mesh;
            cfg::shape_settings cfg;
            stl::bbox<float> bbox;
            typename rasterizer_t::voxel_data_t data;

            //! clear mesh and rasterization data
            void clear() {
                mesh.clear();
                data.clear();
            }
        };
        std::vector<voxel_data_t> _rasterizer_res;
        
        stl::bbox<float> _prj_bbox;
        float _max_grid_size = 0;
        float _voxel_size = 0;
        
    private:
        //! calculates the project bbox
        //! buffers the meshes after creation
        void project_bbox();
        std::filesystem::path make_fname(const voxel_data_t &d) const;
        
    public:
        voxelizer(const cfg::xml_project &cfg);
        std::vector<uint8_t> to_bytes(const voxel_data_t &in_mdata, glm::ivec3 &out_proj_voxels) const;
        void clear();
        void run();

        // TODO not implemented yet
        template<typename rule_t> void to_fs_obj(const voxel_data_t &mdata) const;
        template<typename rule_t> void to_fs_stl(const voxel_data_t &mdata) const;

        void to_fs_bytes(const voxel_data_t &mdata) const;
        void to_fs_vox(const voxel_data_t &mdata) const;

        void to_fs(const voxel_data_t &mdata) const;
    };

    #include "voxelizer.ipp"
};

