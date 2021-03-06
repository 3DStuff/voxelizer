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

        int _num_threads = 1;
        int _max_threads = 1;

    public:
        rle_merge(const cfg::xml_project &project_cfg, const mesh::bbox<float> &glob_bbox) 
        : _project_cfg(project_cfg), _glob_bbox(glob_bbox)
        {
            // does file belong to the project?
            auto is_in_shapes = [&](const std::filesystem::path &p) {
                for(const auto &f : project_cfg.shapes()) {
                    const std::string e = p.string();
                    if(e.find(f._file_out) != e.npos) {
                        return true;
                    }
                }
                return false;
            };

            _project_dir = _project_cfg.target_dir();
            for (const auto & entry : std::filesystem::directory_iterator(_project_dir)) {
                if(!is_in_shapes(entry)) {
                    std::cout << "skip (does not belong to project): " << entry.path().string() << std::endl;
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

            // rle sanity checks
            if(!_rle.empty()) assert(_meta.begin()->size() == 4 && "rle_merge::run(): *.rle files invalid :(");
            auto ritr = _rle.begin();
            while(ritr != _rle.end()) {
                assert(ritr->data()._uncompressed_size == _rle.begin()->data()._uncompressed_size && "rle_merge::run(): rle files are incompatible :(");
                ritr++;
            }
            auto meta_first = _meta.begin();
            auto mitr = _meta.begin();
            while(mitr != _meta.end()) {
                const bool dim_equal = std::equal(meta_first->begin(), meta_first->end(), mitr->begin());
                assert(dim_equal && "rle_merge::run(): rle files are incompatible :(");
                std::ignore = dim_equal;
                mitr++;
            }

            // estimate maximum number of threads
            // the more threads the more memory is necessary
            // we test allocate memory to check whether the programm will run
#ifdef CMAKE_OMP_FOUND
            auto test_alloc = [](int num_threads, const size_t num_voxels) {
                auto ta_impl = [](int num_threads, const size_t num_voxels, const auto& fcn) {
                    if(num_threads < 1) {
                        return 0;
                    }
                    constexpr int safety_margin = 1;
                    char *tmp = nullptr;
                    try {
                        const size_t n = (num_threads+safety_margin+1) * num_voxels;
                        tmp = new char[n];
                    }
                    catch(...) {
                        if(tmp) delete tmp;
                        std::cerr << "Not enough memory for " << num_threads << " threads :(" << std::endl;
                        return fcn(--num_threads, num_voxels, fcn);
                    }
                    if(tmp) delete tmp;
                    return num_threads;
                };
                return ta_impl(num_threads, num_voxels, ta_impl);
            };
#endif
            const auto shape = *_project_cfg.shapes().begin();
            const glm::ivec3 dim = glm::ceil(_glob_bbox.dim() / shape._voxel_size);
            const size_t num_voxels = (size_t)dim.x * dim.y * dim.z;

#ifdef CMAKE_OMP_FOUND
            _max_threads = omp_get_max_threads();
            _num_threads = test_alloc(omp_get_max_threads(), num_voxels);
#endif
            for(const auto &target : _project_cfg.merge_targets()) {
                if(target._type == "efficient") continue;

                if(_num_threads < _max_threads && _num_threads >= 1) {
                    const float mem_gb = (num_voxels * (_max_threads+1)) / (1024*1024*1024);
                    std::cout << "For target: " << target._file_out << std::endl;
                    std::cerr << "  * reduce thread count due to lack of memory" << std::endl;
                    std::cerr << "  * free memory needed for run with " << _max_threads << " threads: " << mem_gb << " GB" << std::endl;
                }
                else if(_num_threads < 1) {
                    const float mem_gb = (num_voxels * 2) / (1024*1024*1024);
                    std::cout << "For target: " << target._file_out << std::endl;
                    std::cerr << "  * fast merging not possible due to lack of memory." << std::endl;
                    std::cerr << "  * consider using \"efficient\" export (type=\"efficient\")";
                    std::cerr << "  * alternatively increase free memory to at least " << mem_gb << " GB" << std::endl;
                }
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

        // returns the tissue with higher priority (smallest value)
        static uint8_t smallest(const cfg::merge_target &target, uint8_t first, uint8_t second) {
            // one of the materials background
            // no further check necessary
            if(first == 0) return second;
            if(second == 0) return first;

            // check priority if defined
            uint8_t pf = first;
            uint8_t ps = second;
            if(!target._prio_map.empty()) {
                auto &prios = target._prio_map;
                pf = prios[first];
                ps = prios[second];
            }
            return pf < ps ? first : second;
        }

    protected:   
        bool run_fast_raw(const cfg::merge_target &target) {
            if(_raw.size() < 2) return false;
            if(_num_threads < 1) return false;
            benchmark::timer t("run_fast_raw::run() - merge took");

            // get meta
            const auto shape = *_project_cfg.shapes().begin();
            const glm::ivec3 dim = glm::ceil(_glob_bbox.dim() / shape._voxel_size);
            const std::vector<size_t> meta = { (size_t)dim.x, (size_t)dim.y, (size_t)dim.z, (size_t)shape._byte_order };

            auto check_buf = [&](const std::vector<uint8_t> &buf){
                assert(buf.size() % dim.x == 0 && "");
                assert(buf.size() % dim.y == 0 && "");
                assert(buf.size() % dim.z == 0 && "");
                glm::ivec3 chk(
                    buf.size() / (dim.y*dim.z),
                    buf.size() / (dim.x*dim.z),
                    buf.size() / (dim.x*dim.y)
                );
                assert(glm::all(glm::equal(chk, dim)) && "");
                std::ignore = chk;
            };

            std::vector<uint8_t> glo_buf;
            auto f = std::ifstream(*_raw.begin(), std::ofstream::binary);
            std::copy(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>(), std::back_inserter(glo_buf));
            f.close();
            check_buf(glo_buf);

            int perc = 0;
            size_t prog = 0;
            benchmark::timer ms("time");

#ifdef CMAKE_OMP_FOUND
            omp_set_num_threads(_num_threads);
#endif
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

                for(size_t id = 0; id < loc_buf.size(); id++) {
                    if(loc_buf[id] == 0) continue;

#pragma omp critical
                    glo_buf[id] = smallest(target, glo_buf[id], loc_buf[id]);
                }
            }

            if(target._file_ext_out.count("rle")) {
                std::string rle_outf = (std::filesystem::path(_project_dir) / (target._file_out + "rle")).string();
                compress::rle rle_out(glo_buf);
                compress::rle_io rio(rle_out, meta);
                rio.to_file(rle_outf);
            }
            if(target._file_ext_out.count("raw")) {
                std::string raw_outf = (std::filesystem::path(_project_dir) / (target._file_out + "raw")).string();
                std::ofstream fout(raw_outf, std::ofstream::binary);
                fout.write((char*)&glo_buf[0], glo_buf.size());
                fout.close();
                // generate info file
                std::string raw_infof = (std::filesystem::path(_project_dir) / (target._file_out + "info")).string();
                write_info(raw_infof, {meta[0], meta[1], meta[2]}, meta[3]);
            }
            return true;
        }

        bool run_fast_rle(const cfg::merge_target &target) {
            if(_rle.size() < 2) return false;
            if(_num_threads < 1) return false;
            benchmark::timer t("run_fast_rle::run() - merge took");

            const auto meta_first = _meta.begin();
            const size_t num_voxels = (size_t)meta_first->at(0) * meta_first->at(1) * meta_first->at(2);
            std::vector<uint8_t> glo_buf(num_voxels, 0);

            int perc = 0;
            size_t prog = 0;
            benchmark::timer ms("time");

#ifdef CMAKE_OMP_FOUND
            omp_set_num_threads(_num_threads);
#endif
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
                for(size_t id = 0; id < num_voxels; id++) {
                    if(loc_buf[id] == 0) continue;
#pragma omp critical
                    glo_buf[id] = smallest(target, glo_buf[id], loc_buf[id]);
                }
            }

            if(target._file_ext_out.count("rle")) {
                std::string rle_outf = (std::filesystem::path(_project_dir) / (target._file_out + "rle")).string();
                compress::rle rle_out(glo_buf);
                compress::rle_io rio(rle_out, *meta_first);
                rio.to_file(rle_outf);
            }
            if(target._file_ext_out.count("raw")) {
                std::string raw_outf = (std::filesystem::path(_project_dir) / (target._file_out + "raw")).string();
                std::ofstream fout(raw_outf, std::ofstream::binary);
                fout.write((char*)&glo_buf[0], glo_buf.size());
                fout.close();
                // generate info file
                std::string raw_infof = (std::filesystem::path(_project_dir) / (target._file_out + "info")).string();
                const glm::ivec3 dim = {meta_first->at(0), meta_first->at(1), meta_first->at(2)};
                write_info(raw_infof, dim, meta_first->at(3));
            }
            return true;
        }

        bool run_efficient_rle(const cfg::merge_target &target) const {
            if(_rle.size() < 2) return false;
            benchmark::timer t("run_efficient_rle::run() - merge took");

            const auto meta_first = _meta.begin();
            const size_t num_voxels = (size_t)meta_first->at(0) * meta_first->at(1) * meta_first->at(2);
            compress::rle<uint8_t> rle_out;

            int perc = 0;
            benchmark::timer ms("time");
            for(size_t id = 0; id < num_voxels; id++) {
                const int cur_perc = (int)((float)id/num_voxels*100);
                if(perc != cur_perc) {
                    perc = cur_perc;
                    std::cout << "progress: " << perc << "/" << 100 << " ";
                    ms.reset();
                }
                // search tissue with highest value (highest == highest priority)
                uint8_t winner_mat = 0;
#pragma omp parallel for reduction (max: winner_mat)
                for(size_t i = 0; i < _rle.size(); i++) {
                    const compress::rle<uint8_t> &r = _rle[i];
                    const uint8_t mat = *r[id];
                    winner_mat = smallest(target, mat, winner_mat);
                }
                rle_out << winner_mat;
            }

            if(target._file_ext_out.count("rle")) {
                std::string rle_outf = (std::filesystem::path(_project_dir) / (target._file_out + "rle")).string();
                compress::rle_io rio(rle_out, *meta_first);
                rio.to_file(rle_outf);
            }
            if(target._file_ext_out.count("raw")) {
                std::string raw_outf = (std::filesystem::path(_project_dir) / (target._file_out + "raw")).string();
                std::ofstream f(raw_outf, std::ofstream::binary);
                std::vector<uint8_t> buf = rle_out.decode();
                f.write((char*)&buf[0], buf.size());
                f.close();
                // generate info file
                std::string raw_infof = (std::filesystem::path(_project_dir) / (target._file_out + "info")).string();
                const glm::ivec3 dim = {meta_first->at(0), meta_first->at(1), meta_first->at(2)};
                write_info(raw_infof, dim, meta_first->at(3));
            }
            return true;
        }
    };
};