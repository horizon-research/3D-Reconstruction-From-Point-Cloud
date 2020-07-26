# 3D-Reconstruction-From-Point-Cloud

### How to compile pointsTransfer.cpp with CmakeList (in ```C++/```)

#### Step 1: Create cmake files

```ccmake .``` 

Then you get empty cache page to modify settings.

Press C to configure ==> set: **CGAL_DIR** (the path to CGAL Library, for example, /Users/ting/PointCloud/CGAL-5.0),   **CMAKE_BUILD_TYPE** (RELEASE)

Note: press enter to modify an option, and press enter to save an option

Press C to configure ==> set  **boost_DIR** (/usr/local/Cellar/boost@1.60/1.60.0/include)

If no errors, press G to generate, then press Q to quit

#### Step 2: Compile

```make```

#### Step 3: Run! 

```./pointsTransfer```







