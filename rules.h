#pragma once

#include "polyhedron/glm_ext/glm_extensions.h"
#include "polyhedron/stl/stl_import.h"

#include <tuple>
#include <vector>
#include <array>
#include "checks.h"
#include "xml_config.h"

struct build_stl_cube {
    static std::array<stl::face, 12> mesh(const glm::vec3 &pos, const glm::vec3 cube_size) {
        auto get_corners = [=](const glm::vec3 &size) {
            std::array<glm::vec3, 8> arr = {
                glm::vec3(pos.x - size.x/2, pos.y - size.y/2, pos.z - size.z/2),
                glm::vec3(pos.x + size.x/2, pos.y - size.y/2, pos.z - size.z/2),
                glm::vec3(pos.x + size.x/2, pos.y + size.y/2, pos.z - size.z/2),
                glm::vec3(pos.x - size.x/2, pos.y + size.y/2, pos.z - size.z/2),
                glm::vec3(pos.x - size.x/2, pos.y - size.y/2, pos.z + size.z/2),
                glm::vec3(pos.x + size.x/2, pos.y - size.y/2, pos.z + size.z/2),
                glm::vec3(pos.x + size.x/2, pos.y + size.y/2, pos.z + size.z/2),
                glm::vec3(pos.x - size.x/2, pos.y + size.y/2, pos.z + size.z/2)
            };
            return arr;
        };

        std::array<glm::vec3, 8> c = get_corners(cube_size);
        return {
            stl::face(c[1], c[0], c[3] ),
            stl::face(c[3], c[2], c[1] ),

            stl::face(c[5], c[1], c[2] ),
            stl::face(c[2], c[6], c[5] ),

            stl::face(c[5], c[6], c[7] ),
            stl::face(c[7], c[4], c[5] ),

            stl::face(c[3], c[0], c[4] ),
            stl::face(c[4], c[7], c[3] ),

            stl::face(c[6], c[2], c[3] ),
            stl::face(c[3], c[7], c[6] ),

            stl::face(c[0], c[1], c[5] ),
            stl::face(c[5], c[4], c[0] )
        };
    }
};

