# Contents

[TOC]

## common
Some common library, especially for heading info.

## syntax
Some C++ syntax demo
1. syntax01_move    \
    Some code for `std::move`.
1. syntax02_dataSize    \
    Print the data size and give a reference when to use `const&` and value parameters in function
1. syntax03_stringSplit \
    Split string using C++11.
1. syntax04_optional    \
    Some example about `std::optional`.

## cxxopts
some study using [cxxopts](https://github.com/jarro2783/cxxopts) library

## std::thread
1. thread01_HelloWorld  \
    Ref: http://www.cnblogs.com/haippy/p/3235560.html
1. thread02_Basic   \
    Basic use of `thread`.
    Ref: http://www.cnblogs.com/haippy/p/3236136.html
1. thread03_Mutex   \
    Use `mutex`
    Ref: http://www.cnblogs.com/haippy/p/3237213.html
1. thread04_TimeMutex   \
    Use `timed_mutex`.
    Ref: http://www.cnblogs.com/haippy/p/3237213.html
1. thread05_LockGuard   \
    Use `lock_guard`.
    Ref: http://www.cnblogs.com/haippy/p/3237213.html
1. thread06_UniqueLock  \
    Use `unique_lock`.
    Ref: http://www.cnblogs.com/haippy/p/3237213.html, http://www.cnblogs.com/haippy/p/3346477.html
1. thread07_ConditionVariable01 \
    Use `condition variable`.
    Ref: http://www.cnblogs.com/haippy/p/3252041.html
1. thread08_ConditionVariable02 \
    Use `condition variable`.
    Ref: http://www.cnblogs.com/haippy/p/3252041.html
1. thread09_Async01 \
    Use `async`, and some comparison.
    Ref: https://blog.csdn.net/lijinqi1987/article/details/78909479
1. thread10_JobQueue    \
    A job queue for the producer-consumer paradigm.
    Ref: https://github.com/colmap/colmap/blob/dev/src/util/threading.h 
1. thread20_ComplexProject  \
    One complex project, class have 2 thread, and the child thread has some callback in class main thread


## boost
1. boost01_FileSystem   \
    Study code about `boost filesystem`: list file and folder, create folder
1. boost02_TimeStudy    \
    Study code about `chrono` time and boost time
1. boost03_Interpolation    \
    Rational interpolation and cubic B-spline interpolation
1. boost04_Tokenizer    \
    Boost tokenizer, separate string with specified separator
1. boost05_LexicalCast  \
    Convert string to int/long/double and vice verse
1. boost06_Serialization    \
    Serialization basic type or class data to text, xml and binary.
1. boost07_MulticastSender  \
    Multicast UDP sender using `asio`, based on boost1.58.
1. boost08_MulticastReceiver    \
    Multicast UDP receiver using `asio`, based on boost1.58.
    

## Eigen
1. Eigen01_Basic    \
    Basic using about Eigen library
1. Eigen02_LinearEquationSolver \
    Solve linear system equations use Eigen
1. Eigen03_Interpolation    \
    Use interpolation method in Eigen
    

## Ceres
1. ceres01_HelloWorld  \
    ceres tutorials, hello world. http://ceres-solver.org/nnls_tutorial.html#hello-world
1. ceres02_Powell   \
    ceres tutorials, powell function.a http://ceres-solver.org/nnls_tutorial.html#powell-s-function
1. ceres03_CurveFitting \
    ceres tutorials, curve fitting, with robust to reduce outliers.
    http://ceres-solver.org/nnls_tutorial.html#curve-fitting
1. ceres04_BundleAdjustment \
    ceres tutorials, bundle adjustment.
    http://ceres-solver.org/nnls_tutorial.html#bundle-adjustment
1. ceres05_libMVBundleAdjuster    \
    ceres tutorials, bundle adjustment algorithm used by Blender/libmv.
    https://ceres-solver.googlesource.com/ceres-solver/+/master/examples/libmv_bundle_adjuster.cc
1. ceres06_RobotPoseMLE \
    Simulate a robot traversing down a 1-dimension hallway with noise odometry readings and noise range readings of the
    end of the hallway. By fusing the noisy odometry and sensor readings this example demonstrates how to compute the
    maximum likelihood estimate (MLE) of the robot's pose at each timestamp.
    https://github.com/ceres-solver/ceres-solver/blob/master/examples/robot_pose_mle.cc
1. ceres07_PoseGraph2D \
    An example of solving a graph-based formulation of SLAM. It reads a 2D pose graph problem definition file in the g2o
    format, formulates and solves the Ceres optimization problem, and outputs the original and optimized poses to file
    for plotting.
    https://github.com/ceres-solver/ceres-solver/tree/master/examples/slam/pose_graph_2d
1. ceres08_PoseGraph3D  \
    An example formulate the pose graph based SLAM problem in 3-Dimensions with relative pose constraints. The example
    also illustrates how to use Eigen’s geometry module with Ceres’s automatic differentiation functionality.
    https://github.com/ceres-solver/ceres-solver/tree/master/examples/slam/pose_graph_3d
1. ceres09_SophusPoseGraph3D    \
    Same as ceres08_PoseGraph3D, but use Sophus::SO3 to represent the rotation.
1. ceres10_CovarianceEstimation \
    Calculate the covariance of estimated value. Here we only take 1 parameter as example, more parameters and complex problem(have information matrix in problem, local parameterization) could also be calculated.
    Ref: http://ceres-solver.org/nnls_covariance.html#covariance
1. ceres11_Pose2DEstimation \
    Consider we have two matched 2D points pair $\mathbf{p}_A \leftrightarrow \mathbf{p}_B$, and want to estimate the 2D transformation $\mathbf{R} \in SO(2), \mathbf{p} \in \mathbb{R}^2$ between the points, i.e.,
    $$
    \mathbf{e} = \sum_{n=0}^{N} (\mathbf{R} \mathbf{p}_{A,n}+ \mathbf{p} - \mathbf{p}_{B,n})
    $$
    The code using two method to optimize this problem,
    - Analytical Jacobian with pose parameterization (R, p)
    - ceres auto Jacobian with pose parameterization (R, p)
    - ceres auto Jacobian with angle parameterization (theta, p). This method don't work correctly.
1. ceres12_Pose3DEstimation \
    Similar to ceres11_Pose2DEstimation but with 3D transformation. Consider we have two matched 3D points pair $\mathbf{p}_A \leftrightarrow \mathbf{p}_B$, and want to estimate the 3D transformation $\mathbf{R} \in SO(3), \mathbf{p} \in \mathbb{R}^3$ between the points, i.e.,
    $$
    \mathbf{e} = \sum_{n=0}^{N} (\mathbf{R} \mathbf{p}_{A,n}+ \mathbf{p} - \mathbf{p}_{B,n})
    $$
    The problem is also named after ICP, the code using two method to optimize this problem,
    - Analytical Jacobian with pose parameterization (R, p)
    - ceres auto Jacobian with pose parameterization (R, p)

## Pangolin
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
    Ref: https://github.com/stevenlovegrove/Pangolin/blob/master/examples/SimpleMultiDisplay/main.cpp
    
    
## Sophus
1. SophusStudy  \
    study code about `Sophus` library
   
## json
some usage for `nlohmann json` library. TODO: not finished.

## YAML
1. YamlStudy    \
    Write and read yaml file using `yaml-cpp`.

## Sqlite
Basic SQLite usage using `SQLiteCpp`.


## pybind
Some code about `pybind11``
1. pybind01_Basic \
    Basic usage. Ref: https://pybind11.readthedocs.io/en/stable/basics.html
1. pybind02_Class \
    Use `class`. Ref: https://pybind11.readthedocs.io/en/stable/classes.html

## Others
1. Alglib01_Interpolation   \
    Study interpolation method in `alglib`.
1. GeometryTransform    \
    Transform among rotation matrix, euler angle, rotation vector and SO3, include Matlab Script
    Ref: https://blog.csdn.net/mulinb/article/details/51227597