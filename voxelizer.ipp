namespace {
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

    inline std::filesystem::path make_fname(const cfg::xml_project &prj, const project_cfg &cfg, const std::string &ext) {
        return  std::filesystem::path(prj.target_dir()) / (cfg.xml._file_out+ext);
    }

    template <typename project_cfg, typename raster_out_t>
    class VoxelByte {
    private:
        const project_cfg &_cfg;
        const raster_out_t &_raster_out;

        glm::ivec3 _dim_global_mesh_vox;
        glm::ivec3 _offset_vox;

    public:
        VoxelByte(const project_cfg &cfg, const raster_out_t &raster_out) 
        : _cfg(cfg), _raster_out(raster_out)
        {
            const glm::vec3 prj_dim = _cfg.glo_bbox.dim();
            float max_grid_size = glm::compMax(glm::ceil(prj_dim / cfg.xml._voxel_size));

            _offset_vox = glm::round((_cfg.loc_bbox._min - _cfg.glo_bbox._min) / _cfg.xml._voxel_size);    
            _dim_global_mesh_vox = glm::ceil(prj_dim / cfg.xml._voxel_size);
        }

        glm::ivec3 loc2glob_offset_voxel() const {
            return _offset_vox;
        }

        //! size of current mesh in voxel 
        glm::ivec3 dim_local_voxel() const {
            return _raster_out._arr_dim;
        }

        //! size of the whole project bbox (all meshes)
        glm::ivec3 dim_global_voxel() const {
            return _dim_global_mesh_vox;
        }

        uint8_t get_voxel(const glm::ivec3 &pos_vox) const {
            switch(_raster_out._voxels[pos_vox.x][pos_vox.y][pos_vox.z]) {
                case voxel_type::interior:
                    return _cfg.xml._material_inside;
                case voxel_type::shell:
                    return _cfg.xml._material_shell;
                default:
                    break;
            };
            return 0;
        }

        std::vector<uint8_t> get_voxels() const {
            const size_t buf_size = (size_t)_dim_global_mesh_vox.x * _dim_global_mesh_vox.y * _dim_global_mesh_vox.z;
            assert((buf_size < std::numeric_limits<uint32_t>::max()) && "to_bytes(): Array too huge");
            std::vector<uint8_t> buffer(buf_size, 0);

            for(size_t z = 0; z < (size_t)dim_local_voxel().z; z++)
            for(size_t y = 0; y < (size_t)dim_local_voxel().y; y++)
            for(size_t x = 0; x < (size_t)dim_local_voxel().x; x++) {
                // calculate position in project wide voxel model
                // position in project unit
                glm::ivec3 pos_vox = glm::ivec3(x,y,z) + _offset_vox;
                // ensure boundaries are not exceeded
                pos_vox = constrain(glm::ivec3(0), (_dim_global_mesh_vox - 1), pos_vox);

                int64_t id = flatten_3dindex(_dim_global_mesh_vox, pos_vox, _cfg.xml._byte_order);
                buffer[id] = get_voxel(glm::ivec3(x,y,z));
            }
            return buffer;
        }
    };
};

template <typename rasterizer_t> 
template<typename rule_t> 
void voxelizer<rasterizer_t>::to_fs_obj(const project_cfg &cfg, const raster_out_t &raster_out) const {
    if(!cfg.xml._file_ext_out.count("obj")) return;
    const std::filesystem::path p = make_fname(_project_cfg, cfg, "obj"); 
    benchmark::timer t("voxelizer::to_obj() - " + p.string() + " export took");
                   
    // write a zero in the face count first as a placeholder
    std::cout << "Generate Mesh and find duplicate faces" << std::endl;
    // now write faces of hull cubes into stl
    std::map<stl::face, size_t> cont_buf;
    std::map<stl::face, size_t> shell_buf;

    const glm::vec3 offset = glm::round(cfg.loc_bbox._min - cfg.glo_bbox._min);
    for(int x = 0; x < (int)raster_out._arr_dim.x; x++)
    for(int y = 0; y < (int)raster_out._arr_dim.y; y++)
    for(int z = 0; z < (int)raster_out._arr_dim.z; z++) {
        constexpr size_t faces_per_cube = 12;
        using cube_t = std::array<stl::face, faces_per_cube>;
        if(raster_out._voxels[x][y][z] == voxel_type::shell) {
            cube_t arr = rule_t::mesh(glm::vec3(x,y,z)*cfg.xml._voxel_size+offset, glm::vec3(cfg.xml._voxel_size));
            for(auto &f : arr) {
                shell_buf[f]++;
            }
        }
        if(raster_out._voxels[x][y][z] == voxel_type::interior) {
            cube_t arr = rule_t::mesh(glm::vec3(x,y,z)*cfg.xml._voxel_size+offset, glm::vec3(cfg.xml._voxel_size));
            for(auto &f : arr) {
                cont_buf[f]++;
            }
        }
    }

    // delete duplicate faces
    auto rem_duplicates = [](auto &inout_buf) {
        auto itr = inout_buf.begin();
        while(itr != inout_buf.end()) {
            if(itr->second > 1) {
                itr = inout_buf.erase(itr);
                continue;
            } 
            itr++;
        }
    };
    // build new cube meshes
    auto build_mesh = [](const std::string &in_name, const auto &in_buf, auto &inout_mesh) {
        inout_mesh._name = in_name;
        uint32_t i = 0;
        for(const auto &f : in_buf) {
            inout_mesh._vertices.push_back(f.first._vert_1);
            inout_mesh._vertices.push_back(f.first._vert_2);
            inout_mesh._vertices.push_back(f.first._vert_3);
            inout_mesh._indices.add(mesh::face(i*3+0, i*3+1, i*3+2));        
            i++;
        }
        inout_mesh.compress();
    };

    mesh::polyhedron<float> shell_mesh;
    mesh::polyhedron<float> cont_mesh;
#pragma omp parallel    // parallel region
#pragma omp single      // one thread per task
{
    #pragma omp task
    {
    rem_duplicates(shell_buf);
    build_mesh("shell", shell_buf, shell_mesh);
    }

    #pragma omp task
    {
    rem_duplicates(cont_buf);
    build_mesh("interior", cont_buf, cont_mesh);
    }
}
    obj::format f;
    f.save({shell_mesh, cont_mesh}, p.string());
}

template <typename rasterizer_t>
template <typename rule_t>
void voxelizer<rasterizer_t>::to_fs_stl(const project_cfg &cfg, const raster_out_t &raster_out) const {
    if(!cfg.xml._file_ext_out.count("stl")) return;

    const std::filesystem::path p = make_fname(_project_cfg, cfg, "stl");  
    benchmark::timer t("voxelizer::to_stl() - " + p.string() + " export took");

    auto stlf = stl::format::open(p.string());
    for(int i = 0; i < 80; i++)
        stl::format::append(stlf, char(0));

    // write a zero in the face count first as a placeholder
    uint32_t faces = 0;
    stl::format::append(stlf, faces);

    std::cout << "Generate Mesh and find duplicate faces" << std::endl;
    // now write faces of hull cubes into stl
    const glm::vec3 offset = glm::round(cfg.loc_bbox._min - cfg.glo_bbox._min);
    std::map<stl::face, size_t> write_buf;
    for(int x = 0; x < (int)raster_out._arr_dim.x; x++)
    for(int y = 0; y < (int)raster_out._arr_dim.y; y++)
    for(int z = 0; z < (int)raster_out._arr_dim.z; z++) {
        if(raster_out._voxels[x][y][z] != voxel_type::shell) continue;
        constexpr size_t faces_per_cube = 12;
        using cube_t = std::array<stl::face, faces_per_cube>;
        cube_t arr = rule_t::mesh(glm::vec3(x,y,z)*cfg.xml._voxel_size+offset, glm::vec3(cfg.xml._voxel_size));
        for(auto &f : arr) {
            write_buf[f]++;
        }
    }

    std::cout << "Write Mesh" << std::endl;
    benchmark::timer tw("voxelizer::to_stl() - " + p.string() + " write took");
    faces = 0;
    for(auto &pair : write_buf) {
        auto &face = pair.first;
        auto &num = pair.second;
        // duplicate faces are always inside, 
        // because cubes touch each other
        // skip them! 
        if(num > 1) continue;

        stl::format::append(stlf, face, sizeof(stl::face));
        faces++;
    }
    stl::format::close(stlf);
    
    // replace face count
    stlf = std::ofstream(p.string(), std::ios::in | std::ios::out | std::ios::binary);
    stlf.seekp(80, std::ios::beg);
    stlf.write((const char*)&faces, 4);
    stlf.close();
}

template <typename rasterizer_t> 
std::vector<uint8_t> voxelizer<rasterizer_t>::to_bytes(const project_cfg &cfg, const raster_out_t &raster_out) const {
    VoxelByte byte(cfg, raster_out);
    return byte.get_voxels();
}

template <typename rasterizer_t> 
void voxelizer<rasterizer_t>::to_fs_bytes(const project_cfg &cfg, const raster_out_t &raster_out) const {
    if(!cfg.xml._file_ext_out.count("raw")) return;
    std::filesystem::path p = make_fname(_project_cfg, cfg, "raw");
    benchmark::timer t("voxelizer::to_fs_bytes() - " + p.string() + " export took");

    // we write in small chunks and never create a huge array
    // saves memory and avoids slow writes
    std::ofstream f(p, std::ofstream::binary);
    const VoxelByte byte(cfg, raster_out);
    const glm::ivec3 dim_glo = byte.dim_global_voxel();
    const glm::ivec3 dim_loc = byte.dim_local_voxel();
    const glm::ivec3 ofs = byte.loc2glob_offset_voxel();

    for(size_t z = 0; z < (size_t)dim_glo.z; z++) {
        std::vector<uint8_t> buf(dim_glo.x*dim_glo.y, 0);
        for(size_t y = 0; y < (size_t)dim_glo.y; y++)
        for(size_t x = 0; x < (size_t)dim_glo.x; x++) {
            const glm::ivec3 pos_vox = glm::ivec3(x,y,z) - ofs;
            // voxel in local boundary?
            const bool voxel_in = glm::all(glm::greaterThanEqual(pos_vox, glm::ivec3(0))) && glm::all(glm::lessThan(pos_vox, dim_loc));
            const uint8_t mat = voxel_in ? byte.get_voxel(pos_vox) : 0;
            const int64_t loc_id = flatten_2dindex(dim_glo.xy(), glm::ivec2(x,y), cfg.xml._byte_order);
            buf[loc_id] = mat;
        }
        f.write((char*)&buf[0], buf.size());
    }
    f.close();

    // generate a text file whichs stores some additional info about the file
    p.replace_extension("info"); // replace file extension with info
    std::ofstream f_size_info(p.string(), std::ofstream::out);
    f_size_info << "[xyz]: " << dim_glo.x << " " << dim_glo.y << " " << dim_glo.z << "\n";
    const std::string byte_order = cfg.xml._byte_order == array_order::row_major ? "row major" : "column major";
    f_size_info << "[byte order]: " << byte_order << "\n";
    f_size_info << "[interior voxel value]: " << cfg.xml._material_inside << "\n";
    f_size_info << "[shell voxel value]: " << cfg.xml._material_shell << "\n";
    f_size_info.close();
}

template <typename rasterizer_t> 
void voxelizer<rasterizer_t>::to_fs_rle(const project_cfg &cfg, const raster_out_t &raster_out) const {
    if(!cfg.xml._file_ext_out.count("rle")) return;
    const std::filesystem::path p = make_fname(_project_cfg, cfg, "rle");
    benchmark::timer t("voxelizer::to_fs_rle() - " + p.string() + " export took");

    // we write in small chunks and never create a huge array
    // saves memory and avoids slow writes
    const VoxelByte byte(cfg, raster_out);
    const glm::ivec3 dim_glo = byte.dim_global_voxel();
    const glm::ivec3 dim_loc = byte.dim_local_voxel();
    const glm::ivec3 ofs = byte.loc2glob_offset_voxel();
    compress::rle<uint8_t> rle;

    // local chunk buffer (profit from  parallelization)
    using chunk_t = std::pair<size_t/*reps*/, uint8_t/*value*/>;
    std::vector<chunk_t> buf;

#pragma omp parallel
{
    std::vector<chunk_t> loc_buf;
#pragma omp for nowait schedule(static)
    for(size_t z = 0; z < (size_t)dim_glo.z; z++)
    for(size_t y = 0; y < (size_t)dim_glo.y; y++)
    for(size_t x = 0; x < (size_t)dim_glo.x; x++) {
        const glm::ivec3 pos_vox = glm::ivec3(x,y,z) - ofs;
        // voxel in local boundary?
        const bool voxel_in = glm::all(glm::greaterThanEqual(pos_vox, glm::ivec3(0))) && glm::all(glm::lessThan(pos_vox, dim_loc));
        const uint8_t mat = voxel_in ? byte.get_voxel(pos_vox) : 0;
        
        // init the array
        if(loc_buf.empty()) {
            loc_buf.push_back({1, mat});
            continue;
        }

        // either increase counter or append new chunk
        auto &last = loc_buf.back();
        if(last.second == mat) {
            last.first++;
        }
        else {
            loc_buf.push_back({1, mat});
        }
    }
#pragma omp for schedule(static) ordered
    for(int i = 0; i < omp_get_num_threads(); i++) {
#pragma omp ordered
        buf.insert(buf.end(), loc_buf.begin(), loc_buf.end());
    }
}
    rle.encode(buf);

    const std::vector<size_t> meta = { (size_t)dim_glo.x, (size_t)dim_glo.y, (size_t)dim_glo.z, (size_t)cfg.xml._byte_order};
    compress::rle_io rle_io(rle, meta);
    rle_io.to_file(p.string());
}

template <typename rasterizer_t> 
void voxelizer<rasterizer_t>::to_fs_vox(const project_cfg &cfg, const raster_out_t &raster_out) const {
    if(!cfg.xml._file_ext_out.count("vox")) return;

    const std::filesystem::path p = make_fname(_project_cfg, cfg, "vox");
    const auto &vox_size = raster_out._arr_dim;
    
    vox::chunk::MAIN chunk_main;                
    vox::chunk::SIZE chunk_size(vox_size);   
    vox::chunk::XYZI chunk_xyzi;
    
    for(int x = 0; x < vox_size.x; x++)
    for(int y = 0; y < vox_size.y; y++)
    for(int z = 0; z < vox_size.z; z++) {
        uint8_t v = raster_out._voxels[x][y][z];
        switch(v) {
            case voxel_type::shell:
                chunk_xyzi.data.push_back(vox::xyzi_t(x, y, z, cfg.xml._material_shell));
                break;
            case voxel_type::interior:
                chunk_xyzi.data.push_back(vox::xyzi_t(x, y, z, cfg.xml._material_inside));
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

template <typename rasterizer_t> 
void voxelizer<rasterizer_t>::to_fs(const project_cfg &cfg, const raster_out_t &raster_out) const {
    to_fs_obj<build_stl_cube>(cfg, raster_out);
    to_fs_stl<build_stl_cube>(cfg, raster_out);
    to_fs_vox(cfg, raster_out);
    to_fs_bytes(cfg, raster_out);
    to_fs_rle(cfg, raster_out);
}

template <typename rasterizer_t> 
voxelizer<rasterizer_t>::voxelizer(const cfg::xml_project &cfg) {
    _project_cfg = cfg;
    
    // calc the meshes and the project bbox
    if(!load()) {
        exit(0);
    }
}

template <typename rasterizer_t> 
void voxelizer<rasterizer_t>::clear() {
    _rasterizer_res.clear();
}

//! calculates the project bbox
//! buffers the meshes after creation
template <typename rasterizer_t> 
bool voxelizer<rasterizer_t>::load() {
    const std::filesystem::path path = _project_cfg.project_path();
    
    _glob_bbox = mesh::bbox<float>();
    for(const cfg::shape_settings &cfg : _project_cfg.shapes()) {
        const std::filesystem::path file = path / cfg._file_in;
        mesh::polyhedron<float> mesh;

        if(cfg._file_ext_inp == "stl") {
            stl::format stl(file.string());
            mesh = stl.to_polyhedron(stl.faces());
        }
        else if(cfg._file_ext_inp == "obj") {
            obj::format obj(file.string());
            mesh = obj.to_polyhedron();
        }
        else {
            std::cerr << "load(): File format not supported" << std::endl;
            return false;
        }

        const mesh::bbox<float> local_bbox = mesh.bounding_box();
        _rasterizer_res.push_back({cfg, mesh, local_bbox});

        // calculate the gloabel project bounding box on the fly
        _glob_bbox.extend(local_bbox);
    }

    // update global bounding box in the config containers
    for(auto &cfg : _rasterizer_res) {
        cfg.glo_bbox = _glob_bbox;
    }
    // update voxel size in the config containers if not defined
    if(_project_cfg.grid_size_defined()) {
        float max_grid_size = _project_cfg.max_grid_size();
        for(auto &cfg : _rasterizer_res) {
            cfg.xml._voxel_size = glm::compMax(glm::ceil(_glob_bbox.dim())) / max_grid_size;
        }
    }
    return true;
}

template <typename rasterizer_t> 
void voxelizer<rasterizer_t>::run() {
    if(!_project_cfg.voxel_size_defined() && !_project_cfg.grid_size_defined()) {
        std::cerr << "Neither maximum grid size, nor voxel size defined in *.xml file" << std::endl;
        return;
    }

    for(project_cfg &cfg : _rasterizer_res) {
        // calculate uniorm scale factor (+/- 1 voxel)
        // keeping the size ratio of each voxel model in the project constant to each other
        const glm::vec3 cur_dim_voxel = cfg.loc_bbox.dim() / cfg.xml._voxel_size;
        const float dim = glm::round(glm::compMax(cur_dim_voxel));
        
        std::cout << "Process file: " << cfg.xml._file_in << std::endl;
        auto raster_out = rasterizer_t(cfg.mesh, glm::ivec3(dim)).rasterize();

        // export data to fs
        to_fs(cfg, raster_out);
        // release data
        cfg.clear();
    }

    rle_merge<uint8_t> merger(_project_cfg, _glob_bbox);
    merger.run();
}
