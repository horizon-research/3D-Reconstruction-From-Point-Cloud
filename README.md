# 3D-Reconstruction-From-Point-Cloud

## Reconstruct Mesh from Point Cloud (Last updated: MeshLab v2016.12)

1. Open [MeshLab](https://www.meshlab.net)

2. Goto ```File -> Import Mesh...``` and select your input **point cloud file**.

3. Goto ```Filters -> Point Set -> Compute normals for point sets```. This is required for Screened Poisson Reconstruction

4. Goto ```Filters -> Remeshing, Simplification and Reconstruction -> Screened Poisson Surface Reconstruction```

   1. You can adjust *Reconstruction Depth* to obtain meshes of different resolution (triangle count). The deeper the reconstruction depth is, the higher the resolution.
   
   2. We used *Depth = 8* for low resolution and *Depth = 10* for high resolution.

5. In project window (by default is on top right of the window), select *Poisson mesh*, goto ```File -> Export Mesh As...``` to save the reconstructed mesh

   1. in *Choose saving options* window, check *Color*, *Normal* for *Vert* , and uncheck *Binary encoding* for *Additional parameters*


## Direct Point Transfer

### Compiling ```pointsTransfer.cpp``` with CmakeList (in ```C++/```)

#### Step 1: Create cmake files

##### Option 1: Use CCMake

```ccmake .``` 

Then you get empty cache page to modify settings.

Press C to configure ==> set: **CGAL_DIR** (the path to CGAL Library, for example, /Users/ting/PointCloud/CGAL-5.0),   **CMAKE_BUILD_TYPE** (RELEASE)

Note: press enter to modify an option, and press enter to save an option

Press C to configure ==> set  **boost_DIR** (/usr/local/Cellar/boost@1.60/1.60.0/include)

If no errors, press G to generate, then press Q to quit

##### Option 2: Use CMake

```cmake .```

#### Step 2: Compile

```make```

#### Step 3: Run! 

```./pointsTransfer <input-point-cloud> <input-triangular-mesh>```

The output would be ```texture.png``` under ```C++/```







