# Inclinometer and compass based on GY-511 board for embedded Linux systems
![](https://github.com/forestcv/Inclinometer-and-Compass-based-on-GY-511-board-for-Embedded-Linux/blob/master/description/gy_511.jpg)

Implementation of a compass and inclinometer based on the GY-511 module for use in programs deployed on embedded linux systems.

## Features
1. Two method of calibration avaliable: ellipsoid-fit and min-max methods.  
2. C++ implementation of ellipsoid-fit calibration based on [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page). It's just a C++ implementation of the ideas from  [sailbaotinstruments](https://sites.google.com/view/sailboatinstruments1/a-download-magneto-v1-2).

## Requirements
1. [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page)
2. [Boost.JSON](https://www.boost.org/doc/libs/1_75_0/libs/json/doc/html/index.html)
