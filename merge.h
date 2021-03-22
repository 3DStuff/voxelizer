#pragma once

#include <cassert>
#include <string>
#include <set>
#include <limits>
#include <filesystem>
#include <vector>
#include <regex>
#include <fstream>

#include "flatten_index.h"
#include "xml_config.h"
#include "enums.h"
#include "polyhedron/glm_ext/glm_extensions.h"
#include "rle/rle.h"
#include "rle/rle_io.h"
#include "timer.h"

#ifdef __GNUC__
    #include "merge_out_gnu.h"
#elif defined(_MSC_VER)
    #include "merge_out_msv.h"
#endif

struct file_header {
    glm::ivec3 glo_dim;
    glm::ivec3 loc_dim;
    glm::ivec3 loc_ofs;
    int order;

    file_header() = default;
    file_header(const glm::ivec3 &glo, const glm::ivec3 &loc, const glm::ivec3 &ofs, const int ord) 
    : glo_dim(glo), loc_dim(loc), loc_ofs(ofs), order(ord) {}
    file_header(const std::vector<int> &v) {
        assert(v.size() == 10);
        glo_dim = {v[0], v[1], v[2]};
        loc_dim = {v[3], v[4], v[5]};
        loc_ofs = {v[6], v[7], v[8]};
        order = v[9];
    }

    std::vector<int> to_arr() const {
        return {
            glo_dim.x, glo_dim.y, glo_dim.z, 
            loc_dim.x, loc_dim.y, loc_dim.z, 
            loc_ofs.x, loc_ofs.y, loc_ofs.z, 
            order
        };
    }
};

struct raw_file {
    std::string             raw_path;
    file_header             raw_info;
};

template <typename base_t>
struct rle_file {
    compress::rle<base_t>   rle_data;
    file_header             rle_info;
};

namespace {
    inline void write_info(std::string file, const file_header &h) {
        // generate a text file whichs stores some additional info about the file
        std::ofstream f_size_info(file, std::ofstream::out);
        f_size_info << "[xyz] " << h.glo_dim.x << " " << h.glo_dim.y << " " << h.glo_dim.z << "\n";
        const std::string s_byte_order = (array_order)h.order == array_order::row_major ? "row_major" : "column_major";
        f_size_info << "[byte order] " << s_byte_order << "\n";
        f_size_info.close();
    }
};

namespace voxelize {
    template<typename base_t>
    class rle_merge {
    private:
        std::map<std::string, int> byte_order = {
            { "row_major", (int)array_order::row_major }, 
            { "column_major", (int)array_order::column_major }, 
        };

        const cfg::xml_project &_project_cfg;
        const mesh::bbox<float> &_glob_bbox;
        std::string _project_dir;

        std::vector<rle_file<base_t>> _rle;
        std::vector<raw_file> _raw;

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
                    _rle.push_back( { rle_inp.get(), file_header(rle_inp.meta()) } );
                }
                // raw files have an extra info file
                if(".raw" == entry.path().extension()) {
                    auto raw_f = entry.path();
                    auto info_f = entry.path(); 
                    info_f.replace_extension(".info");
                    assert(std::filesystem::exists(info_f));

                    // parse the info file
                    std::regex rglo("^\\[glo\\]\\s*([0-9]+)\\s*([0-9]+)\\s*([0-9]+)$");
                    std::regex rloc("^\\[loc\\]\\s*([0-9]+)\\s*([0-9]+)\\s*([0-9]+)$");
                    std::regex rofs("^\\[ofs\\]\\s*([0-9]+)\\s*([0-9]+)\\s*([0-9]+)$");
                    std::regex rorder("^\\[order\\]\\s*([A-z]+)$");

                    std::ifstream f(info_f.string());
                    std::string line;
                    file_header raw_info;
                    while(getline(f, line)){ //read data from file object and put it into string.
                        std::smatch mglo, mloc, mofs, morder;
                        std::regex_match(line, mglo, rglo);
                        std::regex_match(line, mloc, rloc);
                        std::regex_match(line, mofs, rofs);
                        std::regex_match(line, morder, rorder);
                        if(mglo.size() == 4) {
                            raw_info.glo_dim.x = std::stoi(mglo[1]);
                            raw_info.glo_dim.y = std::stoi(mglo[2]);
                            raw_info.glo_dim.z = std::stoi(mglo[3]);
                        }
                        if(mloc.size() == 4) {
                            raw_info.loc_dim.x = std::stoi(mloc[1]);
                            raw_info.loc_dim.y = std::stoi(mloc[2]);
                            raw_info.loc_dim.z = std::stoi(mloc[3]);
                        }
                        if(mofs.size() == 4) {
                            raw_info.loc_ofs.x = std::stoi(mofs[1]);
                            raw_info.loc_ofs.y = std::stoi(mofs[2]);
                            raw_info.loc_ofs.z = std::stoi(mofs[3]);
                        }
                        if(morder.size() == 2) {
                            const std::string sorder = morder[1];
                            raw_info.order = byte_order[sorder];
                        }
                    }
                    std::cout << raw_f << "\n";
                    _raw.push_back( { raw_f.string(), raw_info } );
                }
            }

            // rle sanity checks
            if(!_rle.empty()) {
                auto &mf = _rle[0].rle_info;
                std::ignore = mf;
                for(const auto &m : _rle) {
                    const auto &cur_info = m.rle_info;
                    std::ignore = cur_info;
                    assert(mf.glo_dim.x == cur_info.glo_dim.x && "rle_merge::run(): *.rle files invalid :("); // glo bbox x
                    assert(mf.glo_dim.y == cur_info.glo_dim.y && "rle_merge::run(): *.rle files invalid :("); // glo bbox y
                    assert(mf.glo_dim.z == cur_info.glo_dim.z && "rle_merge::run(): *.rle files invalid :("); // glo bbox z 
                    assert(mf.order == cur_info.order && "rle_merge::run(): *.rle files invalid :("); // order
                }
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

                if(run_fast_rle(target)) continue;
                if(run_efficient_rle(target)) continue;
                if(run_fast_raw(target)) continue;
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

    private:
        bool save_rle_fast(const cfg::merge_target &target, const file_header &header, const std::vector<uint8_t> &buf) const {
            const file_header new_header = file_header(header.glo_dim, header.glo_dim, glm::ivec3(0), header.order);
            if(target._file_ext_out.count("rle")) {
                std::string rle_outf = (std::filesystem::path(_project_dir) / (target._file_out + "rle")).string();
                compress::rle rle_out(buf);
                compress::rle_io rio(rle_out, new_header.to_arr());
                rio.to_file(rle_outf);
                return true;
            }
            return false;
        }
        bool save_raw_fast(const cfg::merge_target &target, const file_header &header, const std::vector<uint8_t> &buf) const {
            const file_header new_header = file_header(header.glo_dim, header.glo_dim, glm::ivec3(0), header.order);
            if(target._file_ext_out.count("raw")) {
                std::string raw_outf = (std::filesystem::path(_project_dir) / (target._file_out + "raw")).string();
                std::ofstream fout(raw_outf, std::ofstream::binary);
                fout.write((char*)&buf[0], buf.size());
                fout.close();
                std::string raw_infof = (std::filesystem::path(_project_dir) / (target._file_out + "info")).string();
                write_info(raw_infof, new_header);
                return true;
            }
            return false;
        }

        bool run_fast_raw(const cfg::merge_target &target) {
            if(_raw.size() < 2) return false;
            if(_num_threads < 1) return false;
            benchmark::timer t("run_fast_raw::run() - merge took");

            const auto &raw_info = _raw[0].raw_info;
            std::vector<uint8_t> glo_buf((size_t)raw_info.glo_dim.x*raw_info.glo_dim.y*raw_info.glo_dim.z, 0);

paral_prog_decl // used for progress output in parallel region
            for(size_t i = 0; i < _raw.size(); i++) {
paral_prog_report(_raw)
                std::vector<uint8_t> loc_buf;
                const std::string &f_path = _raw[i].raw_path;
                std::ifstream f_in = std::ifstream(f_path, std::ofstream::binary);
                std::copy(std::istreambuf_iterator<char>(f_in), std::istreambuf_iterator<char>(), std::back_inserter(loc_buf));
                f_in.close();

                const auto &meta = _raw[i].raw_info;
                const glm::ivec3 loc_bbox = meta.loc_dim;
                const glm::ivec3 loc_ofs = meta.loc_ofs;
                const array_order order = (array_order)meta.order;

                const size_t loc_size = (size_t)loc_bbox.x*loc_bbox.y*loc_bbox.z;
                assert(loc_buf.size() == loc_size && "run_fast_raw() - size not matching");
                std::ignore = loc_size;

                for(int z = 0; z < loc_bbox.z; z++)
                for(int y = 0; y < loc_bbox.y; y++)
                for(int x = 0; x < loc_bbox.x; x++) {
                    const glm::ivec3 loc_pos = glm::ivec3(x,y,z);
                    const int64_t lid = flatten_3dindex(loc_bbox, loc_pos, order);
                    if(loc_buf[lid] == 0) continue;

                    const glm::ivec3 glo_pos = loc_pos + loc_ofs;
                    const int64_t gid = flatten_3dindex(raw_info.glo_dim, glo_pos, order);
paral_crit
                    glo_buf[gid] = smallest(target, glo_buf[gid], loc_buf[lid]);
                }
            }

            std::cout << "save file" << std::endl;
            save_rle_fast(target, raw_info, glo_buf);
            save_raw_fast(target, raw_info, glo_buf);
            return true;
        }

        bool run_fast_rle(const cfg::merge_target &target) {
            if(target._type != "fast") return false;
            if(_rle.size() < 2) return false;
            if(_num_threads < 1) return false;
            benchmark::timer t("run_fast_rle::run() - merge took");

            const auto &rle_info = _rle[0].rle_info;
            std::vector<uint8_t> glo_buf((size_t)rle_info.glo_dim.x*rle_info.glo_dim.y*rle_info.glo_dim.z, 0);

paral_prog_decl // used for progress output in parallel region
            for(size_t i = 0; i < _rle.size(); i++) {
paral_prog_report(_rle)
                const auto &rle = _rle[i].rle_data;
                const auto &meta = _rle[i].rle_info;
                const glm::ivec3 loc_bbox = meta.loc_dim;
                const glm::ivec3 loc_ofs = meta.loc_ofs;
                const array_order order = (array_order)meta.order;

                const std::vector<uint8_t> loc_buf = rle.decode();
                const size_t loc_size = (size_t)loc_bbox.x*loc_bbox.y*loc_bbox.z;
                assert(loc_buf.size() == loc_size && "run_fast_rle() - size not matching");
                std::ignore = loc_size;

                for(int z = 0; z < loc_bbox.z; z++)
                for(int y = 0; y < loc_bbox.y; y++)
                for(int x = 0; x < loc_bbox.x; x++) {
                    const glm::ivec3 loc_pos = glm::ivec3(x,y,z);
                    const int64_t lid = flatten_3dindex(loc_bbox, loc_pos, order);
                    if(loc_buf[lid] == 0) continue;

                    const glm::ivec3 glo_pos = loc_pos + loc_ofs;
                    const int64_t gid = flatten_3dindex(rle_info.glo_dim, glo_pos, order);
paral_crit
                    glo_buf[gid] = smallest(target, glo_buf[gid], loc_buf[lid]);
                }
            }

            std::cout << "save file" << std::endl;
            save_rle_fast(target, rle_info, glo_buf);
            save_raw_fast(target, rle_info, glo_buf);
            return true;
        }

        bool run_efficient_rle(const cfg::merge_target &target) const {
            if(target._type != "efficient") return false;
            if(_rle.size() < 2) return false;
            benchmark::timer t("run_efficient_rle::run() - merge took");

            compress::rle<uint8_t> rle_out;
            const auto &rle_info = _rle[0].rle_info;

lin_prog_decl // used for not so important progress output which can be easily commented
            for(int z = 0; z < rle_info.glo_dim.z; z++)
            for(int y = 0; y < rle_info.glo_dim.y; y++)
            for(int x = 0; x < rle_info.glo_dim.x; x++) {
lin_prog_report
                const glm::ivec3 glo_pos = {x,y,z};
                uint8_t winner_mat = 0;

#pragma omp parallel for reduction (max: winner_mat)
                for(size_t i = 0; i < _rle.size(); i++) {
                    const auto &rle = _rle[i].rle_data;
                    const auto &meta = _rle[i].rle_info;

                    const glm::ivec3 &loc_bbox = meta.loc_dim;
                    const glm::ivec3 &loc_ofs = meta.loc_ofs;
                    const glm::ivec3 loc_pos = glo_pos - loc_ofs;

                    if(glm::any(glm::lessThan(loc_pos, glm::ivec3(0)))) continue;
                    if(glm::any(glm::greaterThanEqual(loc_pos, loc_bbox))) continue;

                    const int64_t lid = flatten_3dindex(loc_bbox, loc_pos, (array_order)meta.order);
                    const uint8_t mat = *rle[lid];
                    winner_mat = smallest(target, mat, winner_mat);
                }
                rle_out << winner_mat;
            }

            std::cout << "save file" << std::endl;
            const file_header head = file_header(rle_info.glo_dim, rle_info.glo_dim, glm::ivec3(0), rle_info.order);
            if(target._file_ext_out.count("rle")) {
                std::string rle_outf = (std::filesystem::path(_project_dir) / (target._file_out + "rle")).string();
                compress::rle_io rio(rle_out, head.to_arr());
                rio.to_file(rle_outf);
            }
            if(target._file_ext_out.count("raw")) {
                save_raw_fast(target, rle_info, rle_out.decode());
            }
            return true;
        }
    };
};