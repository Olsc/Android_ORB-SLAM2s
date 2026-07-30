# HBST: Hamming Binary Search Tree | [Wiki](https://gitlab.com/srrg-software/srrg_hbst/wikis/home) | [Example code](https://gitlab.com/srrg-software/srrg_hbst_examples)
Lightweight and lightning-fast header-only library for binary descriptor-based VPR: `85 kB with 1'000 lines of C++ code`
  
[<img src="https://img.youtube.com/vi/N6RspfFdrOI/0.jpg" width="250">](https://www.youtube.com/watch?v=N6RspfFdrOI)
[<img src="https://img.youtube.com/vi/MwmzJygl8XE/0.jpg" width="250">](https://www.youtube.com/watch?v=MwmzJygl8XE)
[<img src="https://img.youtube.com/vi/f3h398t_zWo/0.jpg" width="250">](https://www.youtube.com/watch?v=f3h398t_zWo)

[![pipeline status](https://gitlab.com/srrg-software/srrg_hbst/badges/master/pipeline.svg)](https://gitlab.com/srrg-software/srrg_hbst/commits/master) Contributors: Dominik Schlegel, Giorgio Grisetti

Supported (i.e. tested) platforms:
- Linux: Ubuntu 14.04 LTS, Ubuntu 16.04 LTS, Ubuntu 18.04 LTS

Minimum requirements:
- [CMake](https://cmake.org) 2.8 or higher
- [C++ 11](http://en.cppreference.com) or higher
- [GCC](https://gcc.gnu.org) 5 or higher

Optional:
- [Eigen3](http://eigen.tuxfamily.org/) for probabilisticly enhanced search access (add the definition `-DSRRG_HBST_HAS_EIGEN` in your cmake project).
- [OpenCV2/3](http://opencv.org/) for the automatic build of wrapped constructors and OpenCV related example code (add the definition `-DSRRG_HBST_HAS_OPENCV` in your cmake project).
- [libQGLViewer](http://libqglviewer.com/) for visual odometry examples ([viewers](examples))
- [catkin Command Line Tools](https://catkin-tools.readthedocs.io/en/latest/) for easy CMake project integration
- [ROS Indigo/Kinetic/Melodic](http://wiki.ros.org/ROS/Installation) for live ROS nodes (make sure you have a sane OpenCV installation)

## Installation instructions (Linux)
- The easy way:

Use the CMake install rule to register HBST in your system:

    cd /directory/where/you/downloaded/srrg_hbst/
    cmake .
    sudo make install
    
To remove HBST from your system we provide a *ballistic* command (use at own risk):

    cd /directory/where/you/downloaded/srrg_hbst/
    sudo make uninstall

---
- The clean way:

Add your local library path to your local environment variables:

    echo "export HBST_ROOT=/directory/where/you/downloaded/srrg_hbst/" >> ~/.profile

In your CMake project, include the header-only library as such without using any CMake modules:

    include_directories($ENV{HBST_ROOT})

---
- The [ROS](http://www.ros.org/) way:

In your catkin workspace, create a symbolic link to the downloaded library directory:

    cd /your/catkin/workspace/src/
    ln -s /directory/where/you/downloaded/srrg_hbst/
    
In your catkin CMake project, retrieve the HBST package by adding the line:

    find_package(catkin REQUIRED COMPONENTS srrg_hbst)

## [Example code repository](https://gitlab.com/srrg-software/srrg_hbst_examples) ([catkin](https://catkin-tools.readthedocs.io) ready!)
Due to the large number of provided example binaries and since we want 
to keep this repository as lightweight as possible, <br>
we created a separate repository for HBST example code. <br>
The example binaries include fully self-contained integrations with `Eigen`, `OpenCV`, `QGLViewer` and `ROS`.

## Build your own Descriptor/Node types!
The 2 base classes: `BinaryNode` and `BinaryMatchable` (see `srrg_hbst/types/`) can easily be inherited. <br>
Users might specify their own, augmented binary descriptor and node classes with specific leaf spawning. <br>
Two variants of subclassing are already provided in `srrg_hbst/types_probabilistic/`. <br>
Others are available in the [example code](https://gitlab.com/srrg-software/srrg_hbst_examples).

## It doesn't work?
[Open an issue](https://gitlab.com/srrg-software/srrg_hbst/issues) or contact the maintainer (see package.xml)

## Related publications
Please cite our most recent article when using the HBST library: <br>

    @article{2018-schlegel-hbst, 
      author  = {D. Schlegel and G. Grisetti}, 
      journal = {IEEE Robotics and Automation Letters}, 
      title   = {{HBST: A Hamming Distance Embedding Binary Search Tree for Feature-Based Visual Place Recognition}}, 
      year    = {2018}, 
      volume  = {3}, 
      number  = {4}, 
      pages   = {3741-3748}
    }

> RA-L 2018 'HBST: A Hamming Distance Embedding Binary Search Tree for Feature-Based Visual Place Recognition' <br>
> https://ieeexplore.ieee.org/document/8411466/ (DOI: 10.1109/LRA.2018.2856542)

Prior works:

    @inproceedings{2016-schlegel-hbst, 
      author    = {D. Schlegel and G. Grisetti}, 
      booktitle = {2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
      title     = {Visual localization and loop closing using decision trees and binary features}, 
      year      = {2016}, 
      pages     = {4616-4623}, 
    }

> IROS 2016 'Visual localization and loop closing using decision trees and binary features' <br>
> http://ieeexplore.ieee.org/document/7759679/ (DOI: 10.1109/IROS.2016.7759679)
