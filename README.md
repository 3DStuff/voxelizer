# A batch voxelizer
This project started as a funny playground for testing some c++20 compiler features (such as static checks for speed up). 
Now it can be used to convert meshes into solid or hollow hexahedron meshes: (*.stl), or voxel files: *.vox (if size is within the 127 voxel boundary), 8 bit binary (*.raw). 

There are three rasterizer implementations which are parallelized and probably (among?) the fastest CPU rasterizers around. 
<code>--safe</code> has additional checks for e.g. edge collisions, is optimized for a lower memory footprint (uses projections, quad trees) 
and continues where most other voxelizers stop because of memory issues and is a nice trade-off regarding performance. 
<code>--fast</code> is optimzed for pure speed and yields a solid voxel model (shell and interior can be assigned)
<code>--hollow</code> is very fast, relatively memory efficient and yields a voxel shell.

| Eiffel tower input mesh                                                        | Output hexahedron mesh                                                         |
|--------------------------------------------------------------------------------|--------------------------------------------------------------------------------|
| ![file](https://github.com/3DStuff/ressources/blob/master/eiffel_mesh.png)     | ![file](https://github.com/3DStuff/ressources/blob/master/eiffel_hex_mesh.png) |

# performance

Performance: i7 4770k (4 physical cores) based on the stanford bunny (112k faces).

<img src="https://github.com/3DStuff/ressources/blob/master/stanford_1024.png" alt="stanford bunny" width="256"/><br>Mesh was rasterized with ~1024³ voxels.

| Volume	| --safe (ms)	| --fast (ms)	| --hollow (ms) 	|
|-------	|--------	    |--------	    |----------	|
| 64³   	| 118    	    | 59     	    | 43       	|
| 128³  	| 298    	    | 71     	    | 58       	|
| 256³  	| 1089   	    | 134    	    | 122      	|
| 512³  	| 4483   	    | 409       	| 398      	|
| 1024³ 	| 17933  	    | 2918      	| 1956     	|
| 2048³ 	| 76225  	    | 28509     	| 11794    	|

# Project configuration

A project is based on a *.xml file which comprehends how the mesh will be rasterized. 

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
The voxelizer supports currently (not much tested) *.stl (hex mesh), *.obj (hex mesh, _not yet finished_), *.raw (plain binary), *.vox ([Magica](https://voxel-magic.com/)) files. 

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


