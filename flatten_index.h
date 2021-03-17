#pragma once

inline int64_t flatten_2dindex(const glm::ivec2 &dim, const glm::ivec2 &pos, const array_order order) {
    int64_t id = -1;
    if (order == array_order::row_major) {
        id = (int64_t)pos.y * dim.x + pos.x;
    }
    else if (order == array_order::column_major) {
        id = (int64_t)pos.x * dim.y + pos.y;
    }
    assert(id >= 0 && "Index is invalid");
    return id;
}

inline int64_t flatten_3dindex(const glm::ivec3 &dim, const glm::ivec3 &pos, const array_order order) {
    int64_t id = -1;
    if (order == array_order::row_major) {
        id = (int64_t)pos.z * dim.x*dim.y + pos.y * dim.x + pos.x;
    }
    else if (order == array_order::column_major) {
        id = (int64_t)pos.x * dim.y*dim.z + pos.y * dim.z + pos.z;
    }
    assert(id >= 0 && "Index is invalid");
    return id;
}