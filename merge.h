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
    inline void write_info(std::string file, const glm::ivec3 &dim_glo, int byte_order) {
        // generate a text file whichs stores some additional info about the file
        std::ofstream f_size_info(file, std::ofstream::out);
        f_size_info << "[xyz]: " << dim_glo.x << " " << dim_glo.y << " " << dim_glo.z << "\n";
        const std::string s_byte_order = byte_order == array_order::row_major ? "row major" : "column major";
        f_size_info << "[byte order]: " << s_byte_order << "\n";
        f_size_info.close();
    }
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
        rle_merge(const cfg::xml_project &project_cfg, const mesh::bbox<float> &glob_bbox) 
        : _project_cfg(project_cfg), _glob_bbox(glob_bbox)
        {
            _project_dir = _project_cfg.target_dir();
            for (const auto & entry : std::filesystem::directory_iterator(_project_dir)) {
                // skip merge targets
                bool skip = false;
                for(const auto &f : project_cfg.merge_targets()) {
                    const std::string e = entry.path().string();
                    if(e.find(f._file_out) != e.npos) {
                        skip = true;
                    }
                }
                if(skip) {
                    std::cout << "skip: " << entry.path().string() << std::endl;
                    continue;
                }

                if(".rle" == entry.path().extension()) {
                    std::cout << "add: " << entry.path() << std::endl; 
                    compress::rle_io<base_t> rle_inp;
                    rle_inp.from_file(entry.path().string());
                    _rle.push_back(rle_inp.get());
                    _meta.push_back(rle_inp.meta());
                }
                if(".raw" == entry.path().extension()) {
                    _raw.push_back(entry.path().string());      
                }
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
                    if(run_fast_raw(target)) continue;
                    if(run_fast_rle(target)) continue;
                }
                if(target._type == "efficient") {
                    if(run_efficient_rle(target)) continue;
                }
            }
        }

    protected:
        bool run_fast_raw(const cfg::merge_target &target) {
            if(_raw.size() < 2) return false;
            benchmark::timer t("run_fast_raw::run() - merge took");

            // get meta
            const auto shape = *_project_cfg.shapes().begin();
            const glm::ivec3 dim = glm::ceil(_glob_bbox.dim() / shape._voxel_size);
            const std::vector<size_t> meta = { (size_t)dim.x, (size_t)dim.y, (size_t)dim.z, (size_t)shape._byte_order };

            auto check_buf = [&](const std::vector<uint8_t> &buf){
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

            int perc = 0;
            size_t prog = 0;
            benchmark::timer ms("time");
#pragma omp parallel for
            for(size_t fid = 1; fid < _raw.size(); fid++) {
                const int cur_perc = (int)((float)prog/_raw.size()*100);
                if(perc != cur_perc) {
                    perc = cur_perc;
#pragma omp critical
{
                    std::cout << "progress: " << perc << "/" << 100 << " ";
                    ms.reset();
}
                }
#pragma omp atomic
                prog++;

                std::vector<uint8_t> loc_buf;
                std::ifstream f_in = std::ifstream(_raw[fid], std::ofstream::binary);
                std::copy(std::istreambuf_iterator<char>(f_in), std::istreambuf_iterator<char>(), std::back_inserter(loc_buf));
                f_in.close();
                check_buf(loc_buf);

                for(int id = 0; id < loc_buf.size(); id++) {
                    if(loc_buf[id] == 0) continue;
#pragma omp critical
                    glo_buf[id] = glo_buf[id] < loc_buf[id] ? loc_buf[id] : glo_buf[id];
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
                std::ofstream fout(raw_outf, std::ofstream::binary);
                fout.write((char*)&glo_buf[0], glo_buf.size());
                fout.close();
                // generate info file
                std::string raw_infof = std::filesystem::path(_project_dir) / (target._file_out + "info");
                write_info(raw_infof, {meta[0], meta[1], meta[2]}, meta[3]);
            }
            return true;
        }

        bool run_fast_rle(const cfg::merge_target &target) {
            if(_rle.size() < 2) return false;
            benchmark::timer t("run_fast_rle::run() - merge took");

            const auto meta_first = _meta.begin();
            const uint64_t num_voxels = (uint64_t)meta_first->at(0) * meta_first->at(1) * meta_first->at(2);
            std::vector<uint8_t> glo_buf(num_voxels, 0);

            int perc = 0;
            uint64_t prog = 0;
            benchmark::timer ms("time");
#pragma omp parallel for
            for(size_t fid = 0; fid < _rle.size(); fid++) {
                const int cur_perc = (int)((float)prog/_rle.size()*100);
                if(perc != cur_perc) {
                    perc = cur_perc;
#pragma omp critical
{
                    std::cout << "progress: " << perc << "/" << 100 << " ";
                    ms.reset();
}
                }
#pragma omp atomic
                prog++;

                const compress::rle<uint8_t> &r = _rle[fid];
                const std::vector<uint8_t> loc_buf = r.decode();
                for(uint64_t id = 0; id < num_voxels; id++) {
                    if(loc_buf[id] == 0) continue;
#pragma omp critical
                    glo_buf[id] = glo_buf[id] < loc_buf[id] ? loc_buf[id] : glo_buf[id];
                }
            }

            if(target._file_ext_out.count("rle")) {
                std::string rle_outf = std::filesystem::path(_project_dir) / (target._file_out + "rle");
                compress::rle rle_out(glo_buf);
                compress::rle_io rio(rle_out, *meta_first);
                rio.to_file(rle_outf);
            }
            if(target._file_ext_out.count("raw")) {
                std::string raw_outf = std::filesystem::path(_project_dir) / (target._file_out + "raw");
                std::ofstream fout(raw_outf, std::ofstream::binary);
                fout.write((char*)&glo_buf[0], glo_buf.size());
                fout.close();
                // generate info file
                std::string raw_infof = std::filesystem::path(_project_dir) / (target._file_out + "info");
                const glm::ivec3 dim = {meta_first->at(0), meta_first->at(1), meta_first->at(2)};
                write_info(raw_infof, dim, meta_first->at(3));
            }
            return true;
        }

        bool run_efficient_rle(const cfg::merge_target &target) const {
            if(_rle.size() < 2) return false;
            benchmark::timer t("run_efficient_rle::run() - merge took");

            const auto meta_first = _meta.begin();
            const uint64_t num_voxels = (uint64_t)meta_first->at(0) * meta_first->at(1) * meta_first->at(2);
            compress::rle<uint8_t> rle_out;

            int perc = 0;
            benchmark::timer ms("time");
            for(uint64_t id = 0; id < num_voxels; id++) {
                const int cur_perc = (int)((float)id/num_voxels*100);
                if(perc != cur_perc) {
                    perc = cur_perc;
                    std::cout << "progress: " << perc << "/" << 100 << " ";
                    ms.reset();
                }
                // search tissue with highest value (highest == highest priority)
                uint8_t winner_mat = 0;
#pragma omp parallel for reduction (max: winner_mat)
                for(int i = 0; i < _rle.size(); i++) {
                    const compress::rle<uint8_t> &r = _rle[i];
                    const uint8_t mat = *r[id];
                    winner_mat = winner_mat < mat ? mat : winner_mat;
                }
                rle_out << winner_mat;
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
                // generate info file
                std::string raw_infof = std::filesystem::path(_project_dir) / (target._file_out + "info");
                const glm::ivec3 dim = {meta_first->at(0), meta_first->at(1), meta_first->at(2)};
                write_info(raw_infof, dim, meta_first->at(3));
            }
            return true;
        }
    };
};