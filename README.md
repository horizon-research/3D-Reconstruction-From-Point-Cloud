# 3D Reconstruction From Point Cloud using Point-based Detail Transfer

This project builds a 3D reconstruction system from massive point cloud targetting mobile VR rendering. The defining feature of our project is to enable a highly efficient reconstruction workflow that significantly lowers the barrier to entry for reconstructing from huge point clouds (billions of points) while at the same time generating 3D models that are visually pleasing and can be rendered in real time (60 FPS) on today's mobile VR devices, e.g., Oculus Go.

While our system is general, we evaluate it by reconstructing [Elmina Castle](https://en.wikipedia.org/wiki/Elmina_Castle), a UNESCO world heritage site in Ghana and is of historical and archaeological significance. The datasets are collected through the [Digital Elmina project](http://digitalelmina.org/) in several field trips to Ghana over a span of three years.

## Repository structure:
* ```src```: source code
* ```imgs```: screenshots of rendered images to compare different existing systems and our system
* ```videos```: video recordings from first-person perspective to compare different existing systems and our system

Please see the [wiki](https://github.com/horizon-research/3D-Reconstruction-From-Point-Cloud/wiki) for a detailed, end-to-end walk-through to reconstruct a 3D model from original point clouds and import the model to Unity for VR rendering. It consists of four steps.
