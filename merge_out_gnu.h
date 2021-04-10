#pragma once

#define paral_prog_decl\
    int perc = 0;\
    size_t prog = 0;\
    benchmark::timer ms("time");\
    _Pragma("omp parallel for")

#define paral_prog_report(arr)\
    const int cur_perc = (int)((float)(prog+1)/arr.size()*100);\
    if(perc != cur_perc)\
    {\
        perc = cur_perc;\
        _Pragma("omp critical")\
        {\
            std::cout << "progress: " << perc << "/" << 100 << " ";\
            ms.reset();\
        }\
    }\
    _Pragma("omp atomic")\
    prog++;

#define paral_crit\
    _Pragma("omp critical") 

#define lin_prog_decl\
    int prog = 0;\
    int perc = 0;\
    size_t num_voxels = (size_t)rle_info.glo_dim.x*rle_info.glo_dim.y*rle_info.glo_dim.z;\
    benchmark::timer ms("time");

#define lin_prog_report\
    const int cur_perc = (int)((float)prog++/num_voxels*100);\
    if(perc != cur_perc) {\
        perc = cur_perc;\
        std::cout << "progress: " << perc << "/" << 100 << " ";\
        ms.reset();\
    }
