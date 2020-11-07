#pragma once

#include <cassert>
#include <string>
#include <set>
#include <filesystem>
#include <vector>

#include "enums.h"
#include "polyhedron/glm_ext/glm_extensions.h"
#include "rle/rle.h"
#include "rle/rle_io.h"


namespace voxelize {
    template<typename base_t>
    class rle_merge {
    private:
        std::string _project_dir;
        std::string _rle_outf;
        std::vector<std::vector<size_t>> _meta;
        std::vector<compress::rle<base_t>> _rle;

    public:
        rle_merge(const std::string &dir, const std::string &target_file) {
            _project_dir = dir;
            _rle_outf = target_file;
        }

        void run() {
            std::vector<std::string> rle_files;
            for (const auto & entry : std::filesystem::directory_iterator(_project_dir)) {
                if(".rle" != entry.path().extension()) continue;

                std::cout << "add: " << entry.path() << std::endl; 

                compress::rle_io<base_t> rle_inp;
                rle_inp.from_file(entry.path().string());
                _rle.push_back(rle_inp.get());
                _meta.push_back(rle_inp.meta());
            }

            // sanity check
            assert(_rle.size() > 0 && "rle_merge::run(): No *.rle files found :(");
            assert(_meta.begin()->size() == 4 && "rle_merge::run(): *.rle files invalid :(");

            auto rle_first = _rle.begin();
            auto ritr = _rle.begin();
            while(ritr != _rle.end()) {
                assert(ritr->data()._uncompressed_size == rle_first->data()._uncompressed_size && "rle_merge::run(): rle files are incompatible :(");
                ritr++;
            }
            
            auto meta_first = _meta.begin();
            auto mitr = _meta.begin();
            while(mitr != _meta.end()) {
                const bool dim_equal = std::equal(meta_first->begin(), meta_first->end(), mitr->begin());
                assert(dim_equal && "rle_merge::run(): rle files are incompatible :(");
                mitr++;
            }

            auto flatten_3dindex = [](const glm::ivec3 &dim, const glm::ivec3 &pos, const array_order order) {
                int64_t id = -1;
                if (order == array_order::row_major) {
                    id = (int64_t)pos.z * dim.x*dim.y + pos.y * dim.x + pos.x;
                }
                else if (order == array_order::column_major) {
                    id = (int64_t)pos.x * dim.y*dim.z + pos.y * dim.z + pos.z;
                }
                assert(id >= 0 && "Index is invalid");
                return id;
            };

            const glm::ivec3 dim = glm::ivec3(meta_first->at(0), meta_first->at(1), meta_first->at(2));
            const array_order order = (array_order)(meta_first->at(3));
            compress::rle<uint8_t> rle_out;

            std::cout << "rle_merge::run(): working with [" << dim[0] << "," << dim[1] << "," << dim[2] << "] grid" << std::endl;
            for(int z = 0; z < dim[2]; z++) {
                std::cout << "merge: " << z << "/" << dim[2] << std::endl;
                for(int y = 0; y < dim[1]; y++)
                for(int x = 0; x < dim[0]; x++) {
                    const int64_t id = flatten_3dindex(dim, glm::ivec3(x,y,z), order);
                    std::set<uint8_t> materials;
                    for(const compress::rle<uint8_t> &r : _rle) {
                        const uint8_t mat = *r[id];
                        if(mat > 0) materials.insert(mat);
                    }
                    if(materials.empty()) {
                        rle_out << (uint8_t)0;
                        continue;
                    }
                    rle_out << *materials.begin();
                }
            }

            compress::rle_io rio(rle_out, *meta_first);
            rio.to_file(_rle_outf);
        }
    };
};