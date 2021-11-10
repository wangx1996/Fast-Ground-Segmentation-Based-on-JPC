# Fast-Ground-Segmentation-Based-on-JPC

[![build passing](https://img.shields.io/badge/build-passing-brightgreen.svg)](https://github.com/wangx1996/Fast-Ground-Segmentation-Based-on-JPC) [![velodyne_HDL_64 compliant](https://img.shields.io/badge/velodyne_HDL_64-compliant-red.svg)](https://github.com/wangx1996/Fast-Ground-Segmentation-Based-on-JPC) [![ros kinetic](https://img.shields.io/badge/ros-kinetic-blue.svg)](https://github.com/wangx1996/Fast-Ground-Segmentation-Based-on-JPC)

An implementation on ["Shen Z, Liang H, Lin L, Wang Z, Huang W, Yu J. Fast Ground Segmentation for 3D LiDAR Point Cloud Based on Jump-Convolution-Process. Remote Sensing. 2021; 13(16):3239. https://doi.org/10.3390/rs13163239"](https://www.mdpi.com/2072-4292/13/16/3239/xml#cite)

![Image text](https://github.com/wangx1996/Fast-Ground-Segmentation-Based-on-JPC/blob/main/test.gif)

## update on 20211110

1. Support ros

2. Fix some bug, but still need to be improved


## 1.Priciple of Algorithm

#### (1) RECM for Coarse Ground Segmentation

##### Image
![Image text](https://github.com/wangx1996/Fast-Ground-Segmentation-Based-on-JPC/blob/main/image/RECM_priciple.png)

##### Algorithm
![Image text](https://github.com/wangx1996/Fast-Ground-Segmentation-Based-on-JPC/blob/main/image/algorithm1.png)

#### (2) JPC for Refine

##### Image
![Image text](https://github.com/wangx1996/Fast-Ground-Segmentation-Based-on-JPC/blob/main/image/JPC_principle.png)

##### Algorithm
![Image text](https://github.com/wangx1996/Fast-Ground-Segmentation-Based-on-JPC/blob/main/image/algorithm2.png)

More detail please check in the paper.

## 2.How to Use

    mkdir build
    cd build
    make -j
    ./main your lidarfile
    
## 3.Result

#### (1) RECM
![Image text](https://github.com/wangx1996/Fast-Ground-Segmentation-Based-on-JPC/blob/main/image/RECM.png)

#### (2) Dilate
![Image text](https://github.com/wangx1996/Fast-Ground-Segmentation-Based-on-JPC/blob/main/image/dilate.png)

#### (3) JPC
![Image text](https://github.com/wangx1996/Fast-Ground-Segmentation-Based-on-JPC/blob/main/image/JPC.png)

#### (4) Pointcloud
![Image text](https://github.com/wangx1996/Fast-Ground-Segmentation-Based-on-JPC/blob/main/image/lidar_result.png)
