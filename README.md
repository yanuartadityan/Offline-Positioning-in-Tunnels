## About

This is a project which aims to solve the positioning problem in the absence of an accurate GPS solution. We will show how high density tunnel model in 3D point clouds, together with only a single vehicle-mounted camera, can estimate the vehicle's position throughout tunnels without the assistance of other positioning systems.
We developed a solution by analysing an image sequence where feature detection and image to point cloud backprojection were used to generate an accurate positioning system. The feature detection was performed using the well-known SIFT algorithm and back-projection onto the point cloud is done through repeated closest-neighbour search along a ray we build from the feature's coordinates.

## Requirements

* OpenCV (C++)
* Matlab
* Python with Numpy

## Can we really use everything here?

All the functions share same copyright as OpenCV has however some of the codes are limited only for educational purposes. For this usage, remain transparent and kindly let us know:

* [Bennehag, **David**](david.bennehag@gmail.com) - Sigma Technologies
* [Nugraha, **Yanuar** T. Aditya](yanuart.adityan@gmail.com) - Volvo Cars

## Keywords

Camera positioning, visual localizations (VSLAM), autonomous and coooperative vehicular, active safety, computer vision, structure-from-motions, kitti
