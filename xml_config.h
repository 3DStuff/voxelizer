#pragma once

#include "polyhedron/glm_ext/glm_extensions.h"
#include "enums.h"

#include <pugixml.hpp>
#include <string>
#include <iostream>
#include <filesystem>
#include <limits>
#include <vector>
#include <regex>
#include <set>
#include <iostream>

namespace detail
{
    template <typename T, std::size_t ... Is>
    constexpr std::array<T, sizeof...(Is)>
    create_array(T value, std::index_sequence<Is...>) {
        return {{(static_cast<void>(Is), value)...}};
    }
}

template <std::size_t N, typename T>
constexpr std::array<T, N> create_array(const T& value) {
    return detail::create_array(value, std::make_index_sequence<N>());
}

namespace cfg {
    namespace hidden {
        bool endsWith(const std::string& str, const std::string& suffix) {
            return str.size() >= suffix.size() && 0 == str.compare(str.size()-suffix.size(), suffix.size(), suffix);
        }   
    }
    
    struct merge_target {
        std::string _file_out;
        std::string _type;
        std::set<std::string> _file_ext_out;
        // for byte like voxels, a prio table like this is fine
        // initialize with lowest prio
        std::array<uint8_t, 256> _prio_map = create_array<256, uint8_t>(255);
    };

    //! shape settings
    struct shape_settings {
        std::string _file_in;
        std::string _file_out;
        std::string _file_ext_inp;
        std::set<std::string> _file_ext_out;
        array_order _byte_order;
        int _material_inside;
        int _material_shell;
        float _voxel_size;
    };

    //! project settings
    class xml_project {
        std::string                 _project_file;
        std::vector<shape_settings> _shapes;
        std::vector<merge_target>   _merge_targets;
        
        int                         _grid_size;     // maximum number of voxels (either along: w, h, d)
        float                       _voxel_size;    // alternatively use voxel size
        
        std::string                 _target_dir = "";
        std::string                 _raw_fname = "";
        
        // internal state members
        bool                        _voxel_size_defined = false;
        bool                        _grid_size_defined = false;

        xml_project read_project(const std::string &xml) {
            auto split = [](const std::string in, const std::string &sep) {
                std::regex regex{"([" + sep + "]+)"};
                std::sregex_token_iterator it{in.begin(), in.end(), regex, -1};
                return std::set<std::string>() = {it, {}};
            };

            if(xml.empty()) {
                std::cerr << "*.xml file undefined" << std::endl;
                return {};
            }
        
            _project_file = xml;
            xml_project proj = *this;
            
            pugi::xml_document doc;
            pugi::xml_parse_result result = doc.load_file(xml.c_str());
            if (result) {            
                pugi::xml_node g = doc.child("project").child("grid");
                if(!g.empty()) {
                    _grid_size_defined = true;
                    _grid_size = g.attribute("max_voxels").as_int();
                }
                pugi::xml_node r = doc.child("project").child("voxel_size");
                if(!r.empty()) {
                    _voxel_size_defined = true;
                    _voxel_size = r.attribute("cube_size").as_float();
                }                
                pugi::xml_node t = doc.child("project").child("target");
                if(!t.empty()) {
                    _target_dir = t.attribute("dir_out").as_string();
                    _raw_fname = t.attribute("raw_fname").as_string();
                }

                for(pugi::xml_node tool : doc.child("project").children("merge")) {
                    std::string file_out = tool.attribute("file_out").as_string();
                    std::string type = tool.attribute("type").as_string();

                    std::string ext = "";
                    const size_t pos_out = file_out.find_last_of(".");
                    if(pos_out != std::string::npos) 
                        ext = file_out.substr(pos_out+1);

                    auto start_ext = file_out.find(ext);
                    file_out.erase(start_ext, ext.size());
                    std::set<std::string> file_ext_out = split(ext, "|");

                    merge_target trg = {file_out, type, file_ext_out};

                    // read optional priority table
                    for (pugi::xml_node mat : tool.children("mat")) {
                        const uint32_t id = mat.attribute("id").as_uint();
                        const uint32_t prio = mat.attribute("prio").as_uint();
                        if(id <= 255 && prio <= 255) {
                            trg._prio_map[id] = prio;
                        }
                        else {
                            std::cerr << "Prios need to be constrained: 0 <= x <= 255" << std::endl;
                        }
                    }

                    _merge_targets.push_back(trg);
                }

                for (pugi::xml_node tool : doc.child("project").children("file")) {
                    std::string file_in = tool.attribute("file_in").as_string();
                    std::string file_out = tool.attribute("file_out").as_string();
                    std::string alignment = tool.attribute("byte_order").as_string();

                    array_order align = array_order::undefined;
                    if(alignment == "row_major") {
                        align = array_order::row_major;
                    }
                    else if(alignment == "column_major") {
                        align = array_order::column_major;
                    }
                    else {
                        align = array_order::row_major;
                    }

                    std::string file_ext_in = "";
                    const size_t pos_in = file_in.find_last_of(".");
                    if(pos_in != std::string::npos) 
                        file_ext_in = file_in.substr(pos_in+1);

                    std::string ext = "";
                    const size_t pos_out = file_out.find_last_of(".");
                    if(pos_out != std::string::npos) 
                        ext = file_out.substr(pos_out+1);

                    auto start_ext = file_out.find(ext);
                    file_out.erase(start_ext, ext.size());
                    std::set<std::string> file_ext_out = split(ext, "|");

                    if(file_ext_out.empty()) {
                        std::cerr << "No file extension defined for " << file_out << ". Maybe not well supported types are *.stl, *.obj, *.vox, *.raw. ";
                        std::cerr << "There will be no export, but the rasterizer will run anyway" << std::endl;
                    }
                    const int mat_in = tool.attribute("material_interior").as_int();
                    const int mat_out = tool.attribute("material_shell").as_int();
                    _shapes.push_back({file_in, file_out, file_ext_in, file_ext_out, align, mat_in, mat_out, _voxel_size});
                }    

                // error cases
                if(_shapes.empty()) {
                    std::cerr << "No *.stl files defined" << std::endl;
                    return {};
                }
                if(t.empty()) {
                    std::cerr << "No target directory defined" << std::endl;
                    return {};
                }
                if(r.empty() && g.empty()) {
                    std::cerr << "No grid size or voxel resolution defined" << std::endl;
                    return {};
                }
            }
            
            if(!_target_dir.empty() && !std::filesystem::exists(_target_dir)) {
                std::cout << _target_dir << " does not exist. Create new directory" << std::endl; 
                std::filesystem::create_directory(_target_dir);
            }
            
            return proj;
        }

    public:
        bool voxel_size_defined() const {
            return _voxel_size_defined;
        }
        bool grid_size_defined() const {
            return _grid_size_defined;
        }
        float voxel_size() const {
            return _voxel_size;
        }
        int max_grid_size() const {
            return _grid_size;
        }
        const std::vector<shape_settings> &shapes() const {
            return _shapes;
        }
        const std::vector<merge_target> &merge_targets() const {
            return _merge_targets;
        }
        const std::string target_dir() const {
            return _target_dir;
        }
        const std::string raw_fname() const {
            return _raw_fname;
        }
        const std::string project_file() const {
            return _project_file;
        }
        const std::string project_path() const {
            return std::filesystem::path(_project_file).parent_path().string();
        }

        static std::vector<xml_project> init(const std::string &project_dir) {
            if(!std::filesystem::exists(project_dir)) {
                std::cerr << "xml_project::init() - invalid project config: " << project_dir << std::endl;
                return {};
            }
            
            std::vector<xml_project> projects;
            for (const auto& entry : std::filesystem::directory_iterator(project_dir)) {
                if(hidden::endsWith(entry.path().string(), ".xml")) {
                    std::cout << "found possible project file: " << entry.path().string() << std::endl;
                    projects.push_back(xml_project(entry.path().string()));
                }
            }

            return projects;
        }

        xml_project() = default;
        xml_project(const std::string &xml) {
            read_project(xml);
        }
    };
};
