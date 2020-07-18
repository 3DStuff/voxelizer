#pragma once

#include "polyhedron/glm_ext/glm_extensions.h"
#include "enums.h"

#include <pugixml.hpp>
#include <string>
#include <iostream>
#include <filesystem>
#include <vector>
#include <regex>
#include <set>


namespace cfg {
    namespace hidden {
        bool endsWith(const std::string& str, const std::string& suffix) {
            return str.size() >= suffix.size() && 0 == str.compare(str.size()-suffix.size(), suffix.size(), suffix);
        }   
    }
    
    //! shape settings
    struct shape_settings {
        std::string _file_in;
        std::string _file_out;
        std::string _file_extension;
        array_order _byte_order;
        int _material_inside;
        int _material_shell;
    };

    //! project settings
    class xml_project {
        std::string                 _project_file;
        std::vector<shape_settings> _shapes;
        
        int                         _grid_size;     // maximum number of voxels (either along: w, h, d)
        float                       _voxel_size;    // alternatively use voxel size
        
        std::string                 _target_dir = "";
        std::string                 _raw_fname = "";
        
        // internal state members
        bool                        _voxel_size_defined = false;
        bool                        _grid_size_defined = false;

        xml_project read_project(const std::string &xml) {
            if(xml.empty()) {
                std::cerr << "*.xml file undefined" << std::endl;
                return {};
            }
            
            _project_file = xml;
            xml_project proj = *this;
            
            pugi::xml_document doc;
            pugi::xml_parse_result result = doc.load_file(xml.c_str());
            if (result) {
                for (pugi::xml_node tool : doc.child("project").children("file")) {
                    const std::string file_in = tool.attribute("file_in").as_string();
                    const std::string file_out = tool.attribute("file_out").as_string();
                    const std::string alignment = tool.attribute("byte_order").as_string();

                    array_order align = array_order::undefined;
                    if(alignment == "row_major") {
                        align = array_order::row_major;
                    }
                    else if(alignment == "column_major") {
                        align = array_order::column_major;
                    }

                    std::string file_ext = "";
                    const size_t pos = file_out.find_last_of(".");
                    if(pos != std::string::npos) 
                        file_ext = file_out.substr(pos+1);

                    if(file_ext.empty()) {
                        std::cerr << "No file extension defined for " << file_out << ". Maybe not well supported types are *.stl, *.obj, *.vox, *.raw. ";
                        std::cerr << "There will be no export, but the rasterizer will run anyway" << std::endl;
                    }

                    const int prio = tool.attribute("merge_priority").as_int();
                    const int mat_in = tool.attribute("material_interior").as_int();
                    const int mat_out = tool.attribute("material_shell").as_int();
                    _shapes.push_back({file_in, file_out, file_ext, align, mat_in, mat_out});
                }                
                pugi::xml_node g = doc.child("project").child("grid");
                if(!g.empty()) {
                    _grid_size_defined = true;
                    _grid_size = g.attribute("max_voxels").as_int();
                    _voxel_size = g.attribute("stl_cube_size").as_float();
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
        const bool voxel_size_defined() const {
            return _voxel_size_defined;
        }
        const bool grid_size_defined() const {
            return _grid_size_defined;
        }
        const float voxel_size() const {
            return _voxel_size;
        }
        const int max_grid_size() const {
            return _grid_size;
        }
        const std::vector<shape_settings> &shapes() const {
            return _shapes;
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
