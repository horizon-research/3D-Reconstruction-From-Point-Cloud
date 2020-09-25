# 3D Reconstruction From Point Cloud with Color Transfer

## 1. Reconstruct Mesh from Point Cloud 

### Option 1: MeshLab (Open source. Last updated for v2016.12)

1. Open [MeshLab](https://www.meshlab.net)

2. Goto ```File -> Import Mesh...``` and select your input **point cloud file**

3. Goto ```Filters -> Point Set -> Compute normals for point sets```. This is required for Screened Poisson Reconstruction

4. Goto ```Filters -> Remeshing, Simplification and Reconstruction -> Screened Poisson Surface Reconstruction```

   1. You can adjust *Reconstruction Depth* to obtain meshes of different resolution (triangle count). The deeper the reconstruction depth is, the higher the resolution.
   
   2. We used *Depth = 8* for low resolution and *Depth = 10* for high resolution

5. In project window (by default is on top right of the window), select *Poisson mesh*, goto ```File -> Export Mesh As...``` to save the reconstructed mesh
 
   <img width="891" alt="Screen Shot 2020-09-25 at 03 18 57" src="https://user-images.githubusercontent.com/25496380/94256030-01866080-fede-11ea-87c4-a4ef023b2bcf.png">

   1. in *Choose saving options* window, check *Color*, *Normal* for *Vert* , and uncheck *Binary encoding* for *Additional parameters*
   
### Option 2: Agisoft Metashape (Propreitary. Last updated for v1.6.1)

1. Open [Metashape](https://www.agisoft.com).

2. Goto ```File -> Import -> Import Points...``` and select your input **point cloud file**

3. Goto ```Model -> Transform Region -> Reset Region``` and select yes

4. Goto ```Workflow -> Build Mesh...```, in *Quality* select/enter desired *Face count* and click *OK*

5. Goto ```File -> Export -> Export Model...```, uncheck *Binary Encoding* and click *OK*

## Color Transfer

### Option 1: Color transfer from high resolution mesh using Blender (Open Source. Last updated for v2.79b, using defualt layout)

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

#### 0: Prepare low resolution mesh with UV using Blender (Open Source. Last updated for v2.79b, using defualt layout)

1. Open [Blender](https://www.blender.org)

2. Press *Esc* to dismiss the launch screen and press *Delete* (macOS: *fn + delete*) to remove the default cube

3. Use ```File -> Import -> Stanford (.ply)``` to import the low resolution mesh

4. Click *Object Mode* (on buttom left of the view) and switch to *Edit Mode*

5. Select *Shading/UVs* (on the left side of the view), under *UV Mapping*, select *Smart UV Project* and click OK

<img width="987" alt="Screen Shot 2020-09-25 at 03 34 34" src="https://user-images.githubusercontent.com/25496380/94257364-0ba95e80-fee0-11ea-9140-c372000a7e17.png">

6. Goto ```File -> Export -> Stanford (.ply)```. In *Export PLY*, check *UVs*, then click *Export PLY* to save the low resolution mesh with its UVs.

#### 1: Create CMake files

##### Option 1: Use CCMake

1. ```ccmake .``` 

2. Terminal will now display an empty cache page to modify settings

3. Press *c* to configure, and after auto configuration, check if the following are set. You can press enter to modify an option, and press enter again to save an option

   1. **CGAL_DIR**: the path to CGAL (e.g. /usr/local/lib/cmake/CGAL).
   2. **Boost_INCLUDE_DIR**: the path to boost library (e.g. /usr/local/include)

4. If no errors or warnings are emitted, continue to press *c* to confingure until the option for press *g* to generate is available

5. Press *g* to generate

6. CCMake will exit. Continue to compilation.

##### Option 2: Use CMake

1. ```cmake .```

2. If no errors or warnings are emitted, continue to compilation

#### 2: Compile

```make```

#### 3: Run! 

```./pointsTransfer <input-point-cloud> <input-triangular-mesh>```

The output would be ```texture.png``` under ```C++/```

## 3. Prepare for use in Unity using Blender (Open Source. Last updated for v2.79b, using defualt layout)

1. Open [Blender](https://www.blender.org)

2. Press *Esc* to dismiss the launch screen and press *Delete* (macOS: *fn + delete*) to remove the default cube

3. Use ```File -> Import -> Stanford (.ply)``` to import the low resolution mesh with UVs.

4. Select *Tools* (on the left side of the view), under *Edit*, click on *Set Origin* and select *Geometry to Origin* so the object is easier to find in Unity

<img width="990" alt="Screen Shot 2020-09-25 at 03 32 20" src="https://user-images.githubusercontent.com/25496380/94257176-c08f4b80-fedf-11ea-8d40-d2a5da4c1160.png">

5. Optional: some meshes may require normal flipping to be rendered properly in Unity.

   0. For example, most meshes have normals pointing outwards and will not be visible if the camera is placed inside the mesh
   
   1. Click *Object Mode* (on buttom left of the view) and switch to *Edit Mode*
   
   <img width="991" alt="Screen Shot 2020-09-25 at 03 33 32" src="https://user-images.githubusercontent.com/25496380/94257282-e61c5500-fedf-11ea-8f5e-d565f56b10c0.png">
   
   2. Select *Shading/UVs* (on the left side of the view), under *Shading*, *Normals:*, click *Flip Direction*
   
   <img width="985" alt="Screen Shot 2020-09-25 at 03 34 09" src="https://user-images.githubusercontent.com/25496380/94257332-ff250600-fedf-11ea-84ce-9234e77a6a8d.png">
   
6. Goto ```File -> Export -> Wavefront (.obj)```. In *Export OBJ*, check *Include UVs* and uncheck *Write Materials*, then click *Export OBJ* to save the low resolution mesh with its UVs.

## 4. Import to Unity (Free Personal Use. Last Updated for v2018.4.1f1)

1. Open [Unity](https://unity.com)

2. Create new project/open exisiting project

3. Open project folder. You can right click in project view and click *Reveal in Finder/Show in Explorer*.

<img width="1003" alt="Screen Shot 2020-09-24 at 01 11 43" src="https://user-images.githubusercontent.com/25496380/94118852-f0b4ec80-fe02-11ea-8046-0f34a14eb788.png">

4. Copy the mesh (.obj) and texture file into ```Assets``` folder.

5. Go back to Unity and wait for Unity to finish importing the asset.

6. Select the mesh in project explorer and in insepctor, goto *Materials* tab, switch to *Use External Materials (Legacy)* for *Location*. Click *Apply*.

<img width="397" alt="Screen Shot 2020-09-25 at 01 40 09" src="https://user-images.githubusercontent.com/25496380/94246447-9b471100-fed0-11ea-91fd-836922dd149a.png">

7. After Unity finishes reimporting the mesh, you will find a new *Materials* folder which contains a new *defaultMat* material

8. Select the material, click the circle next to *Albedo* under *Main Maps* and select your texture file.

<img width="786" alt="Screen Shot 2020-09-25 at 02 15 23" src="https://user-images.githubusercontent.com/25496380/94249627-04308800-fed5-11ea-8531-f2bb0890428c.png">

9. Drag and drop the mesh into hierarchy to add the mesh to the scene.

<img width="1440" alt="Screen Shot 2020-09-25 at 02 16 40" src="https://user-images.githubusercontent.com/25496380/94249859-5376b880-fed5-11ea-9165-864102d9481f.png">

10. Done.

<img width="726" alt="Screen Shot 2020-09-25 at 02 18 14" src="https://user-images.githubusercontent.com/25496380/94249924-65585b80-fed5-11ea-991a-208d4eabdb97.png">



