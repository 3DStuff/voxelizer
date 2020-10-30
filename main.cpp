#include "rasterizer_safe.h"
#include "rasterizer_hollow.h"
#include "rasterizer_fast.h"
#include "rules.h"
#include "voxelizer.hpp"

#include <omp.h>


char* getCmdOption(char ** begin, char ** end, const std::string & option) {
    if (auto itr = std::find(begin, end, option); itr != end && ++itr != end) {
        return *itr;
    }
    return nullptr;
}

bool cmdOptionExists(char** begin, char** end, const std::string& option) {
    return std::find(begin, end, option) != end;
}

void show_help() {
    std::cout << "Parameters:" << std::endl;
    std::cout << "\t--safe: (default) memory efficient rasterizer implementation using edge collision checks and quad search trees" << std::endl;
    std::cout << "\t--fast: faster rasterizer implementation (especially for smaller models) but without edge collision checks" << std::endl;
    std::cout << "\t--hollow: shell only implementation (less artifacts in low resolution models)" << std::endl;
    std::cout << "\t--dir: (default is the current) directory where config *.xml file is searched" << std::endl;
    std::cout << "Example: ./VoxelMagick --dir <directory> --fast " << std::endl;
    std::cout << std::endl;
}

int main(int argc, char * argv[]) {
#ifdef _OPENMP
    size_t num_threads = omp_get_max_threads();
    std::cout << "omp_get_num_threads(): " << num_threads << std::endl;
    omp_set_num_threads(num_threads);
#endif

    int mode = 2;
    std::string dir = ".";
    
    if(cmdOptionExists(argv, argv+argc, "--help") || cmdOptionExists(argv, argv+argc, "--h")) {
        show_help();
        return 0;
    }
    
    if(cmdOptionExists(argv, argv+argc, "--fast")) {
        mode = 1;
    }
    if(cmdOptionExists(argv, argv+argc, "--safe")) {
        mode = 2;
    }
    if(cmdOptionExists(argv, argv+argc, "--hollow")) {
        mode = 3;
    }
    if(cmdOptionExists(argv, argv+argc, "--dir")) {
        auto cstr = getCmdOption(argv, argv + argc, "--dir");
        if(cstr != nullptr)
            dir = std::string(cstr);
    }
    
    std::vector<cfg::xml_project> projects = cfg::xml_project::init(dir);
    for(cfg::xml_project &p : projects) {
        if(mode == 1) {
            voxelize::voxelizer<rasterize::solid_fast<float>> a(p);
            a.run();
        }
        if(mode == 2) {
            voxelize::voxelizer<rasterize::solid_safe<float>> b(p);
            b.run();
        }
        if(mode == 3) {
            voxelize::voxelizer<rasterize::hollow<float>> c(p);
            c.run();
        }
    }

    return 0;
}
