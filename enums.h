#pragma once


enum voxel_type {
    background  = 0,
    interior    = 1 << 2,
    shell       = 1 << 4
};
    
enum swizzle_mode {
    xyz = 0,
    xzy,
    yxz,
    yzx,
    zxy,
    zyx
};

enum array_order {
    row_major = 0,
    column_major,
    undefined
};
