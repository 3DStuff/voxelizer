template <typename rasterizer_t> std::filesystem::path voxelizer<rasterizer_t>::make_fname(const voxel_data_t &d) const {
    return  std::filesystem::path(_project_cfg.target_dir()) / (d.cfg._file_out);
}

template <typename rasterizer_t> 
template<typename rule_t> 
void voxelizer<rasterizer_t>::to_fs_obj(const voxel_data_t &mdata) const {
    if(mdata.cfg._file_extension != "obj") return;

    const std::filesystem::path p = make_fname(mdata);
    mdata.mesh.to_obj(p.string());
}

template <typename rasterizer_t>
template <typename rule_t>
void voxelizer<rasterizer_t>::to_fs_stl(const voxel_data_t &mdata) const {
    if(mdata.cfg._file_extension != "stl") return;

    const glm::vec3 proj_dim = (_prj_bbox._max - _prj_bbox._min);
    const float scalef = _max_grid_size / glm::compMax(proj_dim);  

    bool voxel_size_defined = false;
    if(_project_cfg.voxel_size_defined()) {
        voxel_size_defined = true;
    }
    else if(_project_cfg.grid_size_defined()) {
        voxel_size_defined = false;
    }
    else {
        std::cerr << "Neither grid nor voxel size was defined. Break export." << std::endl;
        return;
    }

    const std::filesystem::path p = make_fname(mdata);
    benchmark::timer t("voxelizer::to_stl() - " + p.string() + " export took");

    auto stlf = stl::format::open(p.string());
    for(int i = 0; i < 80; i++)
        stl::format::append(stlf, char(0));
                    
    glm::vec3 offset;
    if(voxel_size_defined) {
        offset = glm::round(mdata.bbox._min * scalef) * _voxel_size;
    }
    else {
        offset = glm::round((mdata.bbox._min - _prj_bbox._min) * scalef) * _voxel_size;
    }
    
    // write a zero in the face count first as a placeholder
    uint32_t faces = 0;
    stl::format::append(stlf, faces);
        
    // now write faces of hull cubes into stl
    for(int x = 0; x < (int)mdata.data._arr_dim.x; x++) {
        constexpr size_t faces_per_cube = 12;
        using cube_t = std::array<stl::face, faces_per_cube>;
        std::vector<std::array<stl::face, faces_per_cube>> write_buf;
        
        // reduce number of file operations by writing in larger blocks
#pragma omp parallel for
        for(int y = 0; y < (int)mdata.data._arr_dim.y; y++)
        for(int z = 0; z < (int)mdata.data._arr_dim.z; z++) {
            if(mdata.data._voxels[x][y][z] != voxel_type::shell) continue;
#pragma omp critical 
{
            cube_t arr = rule_t::mesh(glm::vec3(x,y,z)*_voxel_size+offset, glm::vec3(_voxel_size));
            write_buf.push_back(arr);
            faces += faces_per_cube;
}
        }
        stl::format::append(stlf, write_buf, write_buf.size() * sizeof(cube_t));
    }
    
    stl::format::close(stlf);
    
    // replace face count
    stlf = std::ofstream(p.string(), std::ios::in | std::ios::out | std::ios::binary);
    stlf.seekp(80, std::ios::beg);
    stlf.write((const char*)&faces, 4);
    stlf.close();
}

template <typename rasterizer_t> std::vector<uint8_t> voxelizer<rasterizer_t>::to_bytes(const voxel_data_t &mdata, glm::ivec3 &out_proj_voxels) const {
    glm::vec3 prj_dim_unit = _prj_bbox._max - _prj_bbox._min;
    const float scalef = _max_grid_size / glm::compMax(prj_dim_unit);   
    
    float voxel_size;
    if(_project_cfg.voxel_size_defined()) {
        voxel_size = _voxel_size;
    }
    else if(_project_cfg.grid_size_defined()) {
        voxel_size = glm::compMax(prj_dim_unit) / _max_grid_size;
    }
    else {
        std::cerr << "Neither grid nor voxel size was defined. Break export." << std::endl;
        return {};
    }
    
    out_proj_voxels = glm::ceil(prj_dim_unit / voxel_size);
    const glm::ivec3 prj_dim_bound = out_proj_voxels - 1;
    std::vector<uint8_t> buffer(glm::compMul(out_proj_voxels), 0);
    
    benchmark::timer t("voxelizer::to_bytes() - " + mdata.cfg._file_in + " export took");
    
    const glm::vec3 offset = glm::round(mdata.bbox._min * scalef) * _voxel_size;    
    const auto &vox_size = mdata.data._arr_dim;
    const auto &cfg = mdata.cfg;
    
    for(int x = 0; x < vox_size.x; x++)
    for(int y = 0; y < vox_size.y; y++)
    for(int z = 0; z < vox_size.z; z++) {
        // calculate position in project wide voxel model
        // position in project unit
        const glm::vec3 pos_unit = glm::vec3(x,y,z)*voxel_size+offset;
        // voxel coordinate
        glm::ivec3 pos_vox = glm::round(pos_unit / voxel_size);
        // ensure boundaries are not exceeded
        pos_vox = constrain(glm::ivec3(0), prj_dim_bound, pos_vox);
        // 1d index
        size_t id;
        if (cfg._byte_order == array_order::row_major) {
            id = pos_vox.z * out_proj_voxels.x * out_proj_voxels.y + pos_vox.y * out_proj_voxels.x + pos_vox.z;
        }
        else if (cfg._byte_order == array_order::column_major) {
            id = pos_vox.x * out_proj_voxels.y * out_proj_voxels.z + pos_vox.y * out_proj_voxels.z + pos_vox.x;
        }
        else {
            std::cerr << "Byte order is undefined in project *.xml file" << std::endl;
            return {};
        }
        
        switch(mdata.data._voxels[x][y][z]) {
            case voxel_type::interior:
                buffer[id] = cfg._material_inside;
                break;
            case voxel_type::shell:
                buffer[id] = cfg._material_shell;
                break;
            default:
                break;
        };
    }
    return buffer;
}

template <typename rasterizer_t> void voxelizer<rasterizer_t>::to_fs_bytes(const voxel_data_t &mdata) const {
    if(mdata.cfg._file_extension != "raw") return;

    const std::filesystem::path p = make_fname(mdata);
    benchmark::timer t("voxelizer::to_fs_bytes() - " + p.string() + " export took");
    
    glm::ivec3 proj_voxels;
    std::vector<uint8_t> buffer = to_bytes(mdata, proj_voxels);
    std::ofstream f(p, std::ios::out | std::ios::binary);
    f.write((char*)&buffer[0], buffer.size());
    f.close();

    const std::filesystem::path p_size_info =  p.string() + ".size_info.txt";
    std::ofstream f_size_info(p_size_info, std::ofstream::out);
    f_size_info << "[xyz]: " << proj_voxels.x << " " << proj_voxels.y << " " << proj_voxels.z << "\n";
    const std::string byte_order = mdata.cfg._byte_order == array_order::row_major ? "row major" : "column major";
    f_size_info << "[byte order]: " << byte_order << "\n";
    f_size_info << "[interior voxel value]: " << mdata.cfg._material_inside << "\n";
    f_size_info << "[shell voxel value]: " << mdata.cfg._material_shell << "\n";
    f_size_info.close();
}

template <typename rasterizer_t> void voxelizer<rasterizer_t>::to_fs_vox(const voxel_data_t &mdata) const {
    if(mdata.cfg._file_extension != "vox") return;

    const std::filesystem::path p = make_fname(mdata);
    const auto &vox_size = mdata.data._arr_dim;
    const auto &cfg = mdata.cfg;
    
    vox::chunk::MAIN chunk_main;                
    vox::chunk::SIZE chunk_size(vox_size);   
    vox::chunk::XYZI chunk_xyzi;
    
    for(int x = 0; x < vox_size.x; x++)
    for(int y = 0; y < vox_size.y; y++)
    for(int z = 0; z < vox_size.z; z++) {
        uint8_t v = mdata.data._voxels[x][y][z];
        switch(v) {
            case voxel_type::shell:
                chunk_xyzi.data.push_back(vox::xyzi_t(x, y, z, cfg._material_shell));
                break;
            case voxel_type::interior:
                chunk_xyzi.data.push_back(vox::xyzi_t(x, y, z, cfg._material_inside));
                break;
            default:
                break;
        };
    }
    
    chunk_main.models.push_back({chunk_size, chunk_xyzi});
    
    std::ofstream f(p.string(), std::ios::out | std::ios::binary);
    chunk_main.write(f);
    f.close();
}

template <typename rasterizer_t> void voxelizer<rasterizer_t>::to_fs(const voxel_data_t &mdata) const {
    to_fs_obj(mdata);
    to_fs_stl<build_stl_cube>(mdata);
    to_fs_vox(mdata);
    to_fs_bytes(mdata);
}

template <typename rasterizer_t> voxelizer<rasterizer_t>::voxelizer(const cfg::xml_project &cfg) {
    _project_cfg = cfg;
    
    // calc the meshes and the project bbox
    project_bbox();
    
    if(_project_cfg.grid_size_defined()) {
        _max_grid_size = _project_cfg.max_grid_size();
        _voxel_size = _project_cfg.voxel_size();
    }
    else if(_project_cfg.voxel_size_defined()) {
        _voxel_size = _project_cfg.voxel_size();
        const float max_bbox_edge = glm::compMax(_prj_bbox._max - _prj_bbox._min);
        _max_grid_size = glm::ceil(max_bbox_edge / _voxel_size);
    }
}

template <typename rasterizer_t> void voxelizer<rasterizer_t>::clear() {
    _rasterizer_res.clear();
}

//! calculates the project bbox
//! buffers the meshes after creation
template <typename rasterizer_t> void voxelizer<rasterizer_t>::project_bbox() {
    const std::filesystem::path path = _project_cfg.project_path();
    
    glm::vec3 glob_min(FLT_MAX);
    glm::vec3 glob_max(-FLT_MAX);
    for(const cfg::shape_settings &shape : _project_cfg.shapes()) {
        const std::filesystem::path file = path / shape._file_in;
        const stl::format stl(file.string());
        
        mesh::polyhedron<float> p_flt = stl.to_polyhedron(stl.faces());
        const stl::bbox<float> bbox = p_flt.bounding_box();
        
        glob_min = glm::min(glob_min, bbox._min);
        glob_max = glm::max(glob_max, bbox._max);
        
        _rasterizer_res.push_back({p_flt, shape, bbox});
    }
    _prj_bbox = { glob_min, glob_max };
}

template <typename rasterizer_t> void voxelizer<rasterizer_t>::run() {
    if(!_project_cfg.voxel_size_defined() && !_project_cfg.grid_size_defined()) {
        std::cerr << "Neither maximum grid size, nor voxel size defined in *.xml file" << std::endl;
        return;
    }
    
    const glm::vec3 proj_dim = (_prj_bbox._max - _prj_bbox._min);
    const float scalef = _max_grid_size / glm::compMax(proj_dim);
    
    for(voxel_data_t &mdata : _rasterizer_res) {
        // calculate uniorm scale factor (+/- 1 voxel)
        // keeping the size ratio of each voxel model in the project constant to each other
        const glm::vec3 mesh_dim = (mdata.bbox._max - mdata.bbox._min) * scalef;
        const float dim = glm::round(glm::compMax(mesh_dim));
        
        std::cout << "Process file: " << mdata.cfg._file_in << std::endl;
        mdata.data = rasterizer_t(mdata.mesh, glm::ivec3(dim)).rasterize();

        // export data to fs
        to_fs(mdata);
        // release data
        mdata.clear();
    }
}