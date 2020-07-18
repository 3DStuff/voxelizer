# A batch voxelizer
This program can be used to convert meshes into hexaeder meshes (*.stl, *.obj), *.vox (if size is withing 127 voxel boundary), and 8 bit binary byte files. 
The rasterizer is parallelized and generally very fast. <code>--fast</code> in single thread mode is comparable with ([VoxSurf](https://github.com/sylefeb/VoxSurf)).

| Complete                                                                             | Detail                                                                         |
|--------------------------------------------------------------------------------------|--------------------------------------------------------------------------------|
| ![file](https://github.com/3DStuff/ressources/blob/master/eiffel_hex_mesh_compl.png) | ![file](https://github.com/3DStuff/ressources/blob/master/eiffel_hex_mesh.png) |

# Project configuration

A project comprehends how the mesh will be rasterized. 

```
<project>
```
We start by a definition of the target resolution (unit depends on the related input file).

```
    <voxel_size cube_size="0.2"></voxel_size>
```

alternatively, we can define a voxel limit. The limit relates to the longest axis.
Here we define that along any axis the voxel number cannot exceed 256.

```
    <grid 
        max_voxels="256"
        stl_cube_size="0.5"
    >
    </grid>
```

Now we define the input files and the output files.
The voxelizer supports currently (not much tested) *.stl (hex mesh), *.obj (hex mesh), *raw (plain binary), *.vox ([Magica](https://voxel-magic.com/)) files. 

```
    <target
        dir_out="Result"
    >
    </target>
```
Generate a hexahedron mesh with *.stl as output format.
```
    <file 
        file_in="roof.stl"
        file_out="hex_roof.stl"
        material_interior="200"
        material_shell="127"
    >
    </file>
```
Generate a raw voxel file. Here all voxels are written as int8_t (char) after each other.
An additional info file with size information and byte alignment is generated automatically. 

```
    <file 
        file_in="roof.stl"
        file_out="hex_roof.raw"
        byte_order="row_major"
        material_interior="200"
        material_shell="127"
    >
    </file>
</project>
```

With the raw files it is very easy to subsequently merge the voxel models into one shape. 

# Build
Requires a C++20 compiler (Clang or GCC), CMake and Git.
```
cd voxelizer
mkdir build
cd build
cmake ..
make
```

# Apply on project
There are three execution modes:
* <code>--safe</code> is memory efficient, uses quad trees instead of a lookup tables (best with huge models or high target resolutions)
* <code>--fast</code> is optimzed for speed but not memory efficient and not as accurate as <code>--safe</code>
* <code>--hollow</code> suggested for shapes intended to be hollow or if the mesh has fine structures, but target resolution is set low

Standard mode is <code>--safe</code>
```
VoxelMagick --dir "folder/with/project/xml"
```

If you don't go wild with voxel numbers and don't care for ocassionally missing voxels.
```
VoxelMagick --dir "folder/with/project/xml" --fast
```

Best if target voxel models shall be hollow
``` 
VoxelMagick --dir "folder/with/project/xml" --hollow
```


