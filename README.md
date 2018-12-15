# Contents
## std::thread
1. thread01_HelloWorld  \
    Ref: http://www.cnblogs.com/haippy/p/3235560.html
1. thread02_Basic   \
    Basic use of thread
    Ref: http://www.cnblogs.com/haippy/p/3236136.html
1. thread03_Mutex   \
    Use mutex
    Ref: http://www.cnblogs.com/haippy/p/3237213.html
1. thread04_TimeMutex   \
    Use timed_mutex
    Ref: http://www.cnblogs.com/haippy/p/3237213.html
1. thread05_LockGuard   \
    Use lock_guard
    Ref: http://www.cnblogs.com/haippy/p/3237213.html
1. thread06_UniqueLock  \
    Use unique_lock
    Ref: http://www.cnblogs.com/haippy/p/3237213.html, http://www.cnblogs.com/haippy/p/3346477.html
1. thread07_ConditionVariable01 \
    Use condition variable
    Ref: http://www.cnblogs.com/haippy/p/3252041.html
1. thread08_ConditionVariable02 \
    Use condition variable
    Ref: http://www.cnblogs.com/haippy/p/3252041.html
1. thread09_Async01 \
    Use async, and some comparison
    Ref: https://blog.csdn.net/lijinqi1987/article/details/78909479
1. thread20_ComplexProject  \
    One complex project, class have 2 thread, and the child thread has some callback in class main thread


## Boost
1. Boost01_FileSystem   \
    Study code about boost filesystem: list file and folder, create folder
1. Boost02_TimeStudy    \
    Study code about chrono time and boost time
1. Boost03_Interpolation    \
    Rational interpolation and cubic B-spline interpolation
1. Boost04_Tokenizer    \
    Boost Tokenizer, seperate string with specified seperator
1. Boost05_LexicalCast  \
    Convert string to int/long/double and vice verse
1. Boost06_Serialization    \
    Serialization class


## YAML
1. YamlStudy    \
    Write and read yaml file using yaml-cpp.


## Eigen
1. Eigen01_Basic    \
    Basic using about Eigen library
1. Eigen02_LinearEquationSolver \
    Solve linear system equations use Eigen
1. Eigen03_Interpolation    \
    Use interpolation method in Eigen


## Sophus
1. SophusStudy  \
    study code about Sophus library


## Ceres
1. Ceres01_HelloWorld  \
    ceres tutorials, hello world. http://ceres-solver.org/nnls_tutorial.html#hello-world
1. Ceres02_Powell   \
    ceres tutorials, powell function.a http://ceres-solver.org/nnls_tutorial.html#powell-s-function
1. Ceres03_CurveFitting \
    ceres tutorials, curve fitting, with robust to reduce outliers.
    http://ceres-solver.org/nnls_tutorial.html#curve-fitting
1. Ceres04_BundleAdjustment \
    ceres tutorials, bundle adjustment.
    http://ceres-solver.org/nnls_tutorial.html#bundle-adjustment
1. Ceres05_libMVBundleAdjuster    \
    ceres tutorials, bundle adjustment algorithm used by Blender/libmv.
    https://ceres-solver.googlesource.com/ceres-solver/+/master/examples/libmv_bundle_adjuster.cc
1. Ceres06_RobotPoseMLE \
    Simulate a robot traversing down a 1-dimension hallway with noise odometry readings and noise range readings of the
    end of the hallway. By fusing the noisy odometry and sensor readings this example demonstrates how to compute the
    maximum likelihood estimate (MLE) of the robot's pose at each timestamp.
    https://github.com/ceres-solver/ceres-solver/blob/master/examples/robot_pose_mle.cc
1. Ceres07_PoseGraph2D \
    An example of solving a graph-based formulation of SLAM. It reads a 2D pose graph problem definition file in the g2o
    format, formulates and solves the Ceres optimization problem, and outputs the original and optimized poses to file
    for plotting.
    https://github.com/ceres-solver/ceres-solver/tree/master/examples/slam/pose_graph_2d
1. Ceres08_PoseGraph3D  \
    An example formulate the pose graph based SLAM problem in 3-Dimensions with relative pose constraints. The example
    also illustrates how to use Eigen’s geometry module with Ceres’s automatic differentiation functionality.
    https://github.com/ceres-solver/ceres-solver/tree/master/examples/slam/pose_graph_3d
1. Ceres09_SophusPoseGraph3D    \
    Same as Ceres08_PoseGraph3D, but use Sophus::SO3 to represent the rotation.

## Alglib
1. Alglib01_Interpolation   \
    Study interpolation method in alglib

## Panoglin
1. Pangolin01_HelloPangolin \
    Draw and show cube using pangolin.
    Ref: https://github.com/stevenlovegrove/Pangolin/blob/master/examples/HelloPangolin/main.cpp
1. Pangolin02_SimplePlot    \
    Plot sin(t), cos(t) and sin(t) + cos(t) and show in pangolin.
    Ref: https://github.com/stevenlovegrove/Pangolin/blob/master/examples/SimplePlot/main.cpp
1. Pangolin03_SimpleDisplay \
    Display and some UI widgets
    Ref: https://github.com/stevenlovegrove/Pangolin/blob/master/examples/SimpleDisplay/main.cpp
1. Pangolin04_MultiDisplay  \
    Multiple display.
    Ref:https://github.com/stevenlovegrove/Pangolin/blob/master/examples/SimpleMultiDisplay/main.cpp

## Others
1. DataSize\
    print the data size and give a reference when to use const& and value parameters in function
1. GeometryTransform    \
    Transform among rotation matrix, euler angle, rotation vector and SO3, include Matlab Script
    Ref: https://blog.csdn.net/mulinb/article/details/51227597