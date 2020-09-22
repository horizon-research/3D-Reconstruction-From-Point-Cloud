# 3D-Reconstruction-From-Point-Cloud

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







