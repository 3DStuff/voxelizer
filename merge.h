#pragma once

#include <cassert>
#include <string>
#include <set>
#include <limits>
#include <filesystem>
#include <vector>
#include <fstream>

#include "xml_config.h"
#include "enums.h"
#include "polyhedron/glm_ext/glm_extensions.h"
#include "rle/rle.h"
#include "rle/rle_io.h"
#include "timer.h"


namespace {
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
};

namespace voxelize {
    template<typename base_t>
    class rle_merge {
    private:
        const cfg::xml_project &_project_cfg;
        const mesh::bbox<float> &_glob_bbox;
        std::string _project_dir;

        std::vector<std::vector<size_t>> _meta;
        std::vector<compress::rle<base_t>> _rle;
        std::vector<std::string> _raw;

    public:
        rle_merge(const cfg::xml_project &project_cfg, const mesh::bbox<float> &glob_bbox) : _project_cfg(project_cfg), _glob_bbox(glob_bbox)
        {
            _project_dir = _project_cfg.target_dir();

            std::vector<std::string> rle_files;
            for (const auto & entry : std::filesystem::directory_iterator(_project_dir)) {
                if(".rle" != entry.path().extension()) continue;
                std::cout << "add: " << entry.path() << std::endl; 
                compress::rle_io<base_t> rle_inp;
                rle_inp.from_file(entry.path().string());
                _rle.push_back(rle_inp.get());
                _meta.push_back(rle_inp.meta());
            }

            for (const auto & entry : std::filesystem::directory_iterator(_project_dir)) {
                if(".raw" != entry.path().extension()) continue;
                std::cout << "add: " << entry.path() << std::endl; 
                _raw.push_back(entry.path().string());
            }

            // sanity checks
            if(!_rle.empty()) assert(_meta.begin()->size() == 4 && "rle_merge::run(): *.rle files invalid :(");

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
        }

        void run() {
            for(const auto &target : _project_cfg.merge_targets()) {
                if(!target._file_ext_out.count("rle") && !target._file_ext_out.count("raw")) {
                    std::cerr << "Merging only possible for binary files" << std::endl;
                    return;
                }

                if(target._type == "fast") {
                    run_fast_raw(target);
                    //run_fast_rle(target);
                }
                if(target._type == "efficient") {
                    run_efficient_rle(target);
                }
            }
        }

    protected:
        void run_fast_raw(const cfg::merge_target &target) {
            if(_raw.size() < 2) return;
            benchmark::timer t("run_fast_raw::run() - merge took");

            // get meta
            const auto shape = *_project_cfg.shapes().begin();
            const glm::ivec3 dim = glm::ceil(_glob_bbox.dim() / shape._voxel_size);
            const std::vector<size_t> meta = { (size_t)dim.x, (size_t)dim.y, (size_t)dim.z, (size_t)shape._byte_order };

            auto check_buf = [&](const std::vector<uint8_t> &buf){
                std::cout << buf.size() <<"; " << dim.x <<";"<< dim.y<<";" << dim.z << std::endl;
                assert(buf.size() % dim.x == 0 && "");
                assert(buf.size() % dim.y == 0 && "");
                assert(buf.size() % dim.z == 0 && "");
                const size_t z = buf.size() / (dim.x*dim.y);
                const size_t y = buf.size() / (dim.x*dim.z);
                const size_t x = buf.size() / (dim.y*dim.z);
                assert(glm::all(glm::equal(glm::ivec3(x,y,z), dim)) && "");
            };

            std::vector<uint8_t> glo_buf;
            auto f = std::ifstream(*_raw.begin(), std::ofstream::binary);
            std::copy(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>(), std::back_inserter(glo_buf));
            f.close();
            check_buf(glo_buf);

            for(int i = 1; i < _raw.size(); i++) {
                std::cout << "progress: " << (int)((float)i/_raw.size()*100) << "/" << 100 << std::endl;

                std::vector<uint8_t> loc_buf;
                f = std::ifstream(_raw[i], std::ofstream::binary);
                std::copy(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>(), std::back_inserter(loc_buf));
                f.close();
                check_buf(loc_buf);

#pragma omp parallel for
                for(int j = 0; j < loc_buf.size(); j++) {
                    if(loc_buf[j] == 0) {
                        continue;
                    }
                    if(glo_buf[j] == 0) {
                        glo_buf[j] = loc_buf[j];
                        continue;
                    }
                    glo_buf[j] = glo_buf[j] > loc_buf[j] ? loc_buf[j] : glo_buf[j];
                }
            }

            if(target._file_ext_out.count("rle")) {
                std::string rle_outf = std::filesystem::path(_project_dir) / (target._file_out + "rle");
                compress::rle rle_out(glo_buf);
                compress::rle_io rio(rle_out, meta);
                rio.to_file(rle_outf);
            }
            if(target._file_ext_out.count("raw")) {
                std::string raw_outf = std::filesystem::path(_project_dir) / (target._file_out + "raw");
                std::ofstream f(raw_outf, std::ofstream::binary);
                f.write((char*)&glo_buf[0], glo_buf.size());
                f.close();
            }
        }

        void run_efficient_rle(const cfg::merge_target &target) const {
            if(_rle.size() < 2) return;
            benchmark::timer t("run_efficient_rle::run() - merge took");

            const auto meta_first = _meta.begin();
            const glm::vec<3, size_t> dim = glm::ivec3(meta_first->at(0), meta_first->at(1), meta_first->at(2));
            const array_order order = (array_order)(meta_first->at(3));
            compress::rle<uint8_t> rle_out;

            // absolutely memory efficient
            for(int z = 0; z < dim[2]; z++) {
                std::cout << "progress: " << (int)((float)z/dim[2]*100) << "/" << 100 << std::endl;
                benchmark::timer z_timer("iteration took");
                for(int y = 0; y < dim[1]; y++)
                for(int x = 0; x < dim[0]; x++) {
                    const int64_t id = flatten_3dindex(dim, glm::ivec3(x,y,z), order);

                    // search tissue with lowest value (lowest == highest priority)
                    uint8_t winner_mat = 0;
#pragma omp parallel for
                    for(int i = 0; i < _rle.size(); i++) {
                        const compress::rle<uint8_t> &r = _rle[i];
                        const auto itr = r[id];
                        assert(itr != r.end() && "rle_merger::run(): iterator out of index");

                        const uint8_t mat = *itr;
                        if(mat == 0) continue;
                        if(winner_mat < 1) {
#pragma omp critical
                            winner_mat = mat;
                            continue;
                        }
                        if(winner_mat > mat) {
#pragma omp critical
                            winner_mat = mat;
                            continue;
                        }
                    }
                    
                    rle_out << winner_mat;
                }
            }

            if(target._file_ext_out.count("rle")) {
                std::string rle_outf = std::filesystem::path(_project_dir) / (target._file_out + "rle");
                compress::rle_io rio(rle_out, *meta_first);
                rio.to_file(rle_outf);
            }
            if(target._file_ext_out.count("raw")) {
                std::string raw_outf = std::filesystem::path(_project_dir) / (target._file_out + "raw");
                std::ofstream f(raw_outf, std::ofstream::binary);
                std::vector<uint8_t> buf = rle_out.decode();
                f.write((char*)&buf[0], buf.size());
                f.close();
            }
        }

        void run_fast_rle(const cfg::merge_target &target) {
            if(_rle.size() < 2) return;
            benchmark::timer t("run_fast_rle::run() - merge took");

            const auto meta_first = _meta.begin();
            const glm::vec<3, size_t> dim = glm::ivec3(meta_first->at(0), meta_first->at(1), meta_first->at(2));
            const array_order order = (array_order)(meta_first->at(3));

            const uint64_t num_voxels = (uint64_t)dim.x * dim.y * dim.z;
            std::vector<uint8_t> buffer(num_voxels, 0);
            for(int i = 0; i < _rle.size(); i++) {
                const compress::rle<uint8_t> &r = _rle[i];
                std::cout << "progress: " << (int)((float)i/_rle.size()*100) << "/" << 100 << std::endl;
                const std::vector<uint8_t> cur_arr = r.decode();
#pragma omp parallel for
                for(int z = 0; z < dim[2]; z++)
                for(int y = 0; y < dim[1]; y++)
                for(int x = 0; x < dim[0]; x++) {
                    const int64_t id = flatten_3dindex(dim, glm::ivec3(x,y,z), order);
                    if(cur_arr[id] == 0) continue;
                    if(buffer[id] == 0) {
                        buffer[id] = cur_arr[id];
                        continue;
                    }
                    buffer[id] = buffer[id] > cur_arr[id] ? cur_arr[id] : buffer[id];
                }
            }

            if(target._file_ext_out.count("rle")) {
                std::string rle_outf = std::filesystem::path(_project_dir) / (target._file_out + "rle");
                compress::rle rle_out(buffer);
                compress::rle_io rio(rle_out, *meta_first);
                rio.to_file(rle_outf);
            }
            if(target._file_ext_out.count("raw")) {
                std::string raw_outf = std::filesystem::path(_project_dir) / (target._file_out + "raw");
                std::ofstream f(raw_outf, std::ofstream::binary);
                f.write((char*)&buffer[0], buffer.size());
                f.close();
            }
        }
    };
};