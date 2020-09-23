# 3D Reconstruction From Point Cloud with Color Transfer

## 1. Reconstruct Mesh from Point Cloud 

### Option 1: MeshLab (Open source. Last updated for: MeshLab v2016.12)

1. Open [MeshLab](https://www.meshlab.net)

2. Goto ```File -> Import Mesh...``` and select your input **point cloud file**

3. Goto ```Filters -> Point Set -> Compute normals for point sets```. This is required for Screened Poisson Reconstruction

4. Goto ```Filters -> Remeshing, Simplification and Reconstruction -> Screened Poisson Surface Reconstruction```

   1. You can adjust *Reconstruction Depth* to obtain meshes of different resolution (triangle count). The deeper the reconstruction depth is, the higher the resolution.
   
   2. We used *Depth = 8* for low resolution and *Depth = 10* for high resolution

5. In project window (by default is on top right of the window), select *Poisson mesh*, goto ```File -> Export Mesh As...``` to save the reconstructed mesh

   1. in *Choose saving options* window, check *Color*, *Normal* for *Vert* , and uncheck *Binary encoding* for *Additional parameters*

## Color Transfer

### Option 1: Color transfer from high resolution mesh using Blender (Last updated for v2.79b, using defualt layout)

1. Open [Blender](https://www.blender.org)

2. Press *Esc* to dismiss the launch screen and press *Delete* (macOS: *fn + delete*) to remove the default cube

3. Use ```File -> Import -> Stanford (.ply)``` to import both high and low resolution meshes. Note: files can only be opened one by one

4. Select (on top right of the window) the low resolution mesh

5. Click *Object Mode* (on buttom left of the view) and switch to *Edit Mode*

6. Select *Shading/UVs* (on the left side of the view), under *UV Mapping*, select *Smart UV Project* and click OK

7. Open a new image view

   1. Goto the top right corner of the 3D view and drag left to create a new area
   
   2. On buttom left of the view, click *Object Mode* and switch to *UV/Image Editor*

8. On the bottm of the view, click *New* and set the *Width* and *Height* to 8192px

9. Select the high resolution mesh, then, while holding *shift*, select the low resolution mesh

10. While in *Edit Mode*, goto *Properties Editor* (buttom right of the window), select *Render* (camera icon), and find *Bake*.

    1. Change *Bake Mode* to *Vertex Color*.
   
    2. Check *Selected to Active*.
    
    3. Hit *Bake*
    
    4. After baking is done, at bottom of image view, goto ```Image -> Save As Image```
    
11. Select low resolution mesh and goto ```File -> Export -> Stanford (.ply)```. In *Export PLY*, check *UVs*, then click *Export PLY* to save the low resolution mesh with its UVs.

### Option 2: Direct Color Transfer from Point Cloud (in ```C++/```)

#### 0: Prepare low resolution mesh with UV

1. Open [Blender](https://www.blender.org)

2. Press *Esc* to dismiss the launch screen and press *Delete* (macOS: *fn + delete*) to remove the default cube

3. Use ```File -> Import -> Stanford (.ply)``` to import the low resolution mesh

4. Click *Object Mode* (on buttom left of the view) and switch to *Edit Mode*

5. Select *Shading/UVs* (on the left side of the view), under *UV Mapping*, select *Smart UV Project* and click OK

6. Goto ```File -> Export -> Stanford (.ply)```. In *Export PLY*, check *UVs*, then click *Export PLY* to save the low resolution mesh with its UVs.

#### 1: Create CMake files

##### Option 1: Use CCMake

```ccmake .``` 

Then you get empty cache page to modify settings.

Press C to configure ==> set: **CGAL_DIR** (the path to CGAL Library, for example, /Users/ting/PointCloud/CGAL-5.0),   **CMAKE_BUILD_TYPE** (RELEASE)

Note: press enter to modify an option, and press enter to save an option

Press C to configure ==> set  **boost_DIR** (/usr/local/Cellar/boost@1.60/1.60.0/include)

If no errors, press G to generate, then press Q to quit

##### Option 2: Use CMake

```cmake .```

#### 2: Compile

```make```

#### 3: Run! 

```./pointsTransfer <input-point-cloud> <input-triangular-mesh>```

The output would be ```texture.png``` under ```C++/```

## 3. Prepare for use in Unity

1. Open [Blender](https://www.blender.org)

2. Press *Esc* to dismiss the launch screen and press *Delete* (macOS: *fn + delete*) to remove the default cube

3. Use ```File -> Import -> Stanford (.ply)``` to import the low resolution mesh with UVs.

4. Select *Tools* (on the left side of the view), under *Edit*, click on *Set Origin* and select *Geometry to Origin* so the object is easier to find in Unity

5. Optional: some meshes may require normal flipping to be rendered properly in Unity.

   0. For example, most meshes have normals pointing outwards and will not be visible if the camera is placed inside the mesh
   
   1. Click *Object Mode* (on buttom left of the view) and switch to *Edit Mode*
   
   2. Select *Shading/UVs* (on the left side of the view), under *Shading*, *Normals:*, click *Flip Direction*
   
6. Goto ```File -> Export -> Wavefront (.obj)```. In *Export OBJ*, check *Include UVs* and uncheck *Write Materials*, then click *Export OBJ* to save the low resolution mesh with its UVs.

## 4. Import to Unity





