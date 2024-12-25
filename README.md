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
1. syntax05_any    \
    Some example about `std::any`.
1. syntax06_dataConversion  \
    Data conversion, `uint8_t[8] => uint64_t`

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
1. thread11_PackagedTask \
    Use `packaged_task` example
1. thread12_ComplexPackagedTask \
    A complex example using `packaged_task`, see code note for details.
1. thread13_AtomicBasic \
    Basic code using `atomic`
1. thread20_ComplexProject  \
    One complex project, class have 2 thread, and the child thread has some callback in class main thread

## Eigen
1. Eigen01_Basic    \
    Basic using about Eigen library
1. Eigen02_LinearEquationSolver \
    Solve linear system equations use Eigen
1. Eigen03_Interpolation    \
    Using interpolation method in Eigen
1. Eigen04_Format   \
    Using fmt::format to format Eigen::Vector and Eigen::Matrix
        
## Sophus
1. SophusStudy  \
    study code about `Sophus` library

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
    The code don't run correctly, see `ceres12_Pose3DEstimation` for better usage
1. ceres10_CovarianceEstimation \
    Calculate the covariance of estimated value. Here we only take 1 parameter as example, more parameters and complex problem(have information matrix in problem, manifold) could also be calculated.
    Ref: http://ceres-solver.org/nnls_covariance.html#covariance
1. ceres11_Pose2DEstimation \
    Consider we have two matched 2D points pair $\mathbf{p}_A \leftrightarrow \mathbf{p}_B$, and want to estimate the 2D transformation $\mathbf{R} \in SO(2), \mathbf{p} \in \mathbb{R}^2$ between the points, i.e.,
    $$
    \mathbf{e} = \sum_{n=0}^{N} (\mathbf{R} \mathbf{p}_{A,n}+ \mathbf{p} - \mathbf{p}_{B,n})
    $$
    The code using two method to optimize this problem,
    - Analytical Jacobian with pose manifold (R, p)
    - ceres auto Jacobian with pose manifold (R, p)
    - ceres auto Jacobian with angle manifold (theta, p).
1. ceres12_Pose3DEstimation \
    Similar to ceres11_Pose2DEstimation but with 3D transformation. Consider we have two matched 3D points pair $\mathbf{p}_A \leftrightarrow \mathbf{p}_B$, and want to estimate the 3D transformation $\mathbf{R} \in SO(3), \mathbf{p} \in \mathbb{R}^3$ between the points, i.e.,
    $$
    \mathbf{e} = \sum_{n=0}^{N} (\mathbf{R} \mathbf{p}_{A,n}+ \mathbf{p} - \mathbf{p}_{B,n})
    $$
    The problem is also named after ICP, the code using two method to optimize this problem,
    - Analytical Jacobian with pose manifold (R, p)
    - ceres auto Jacobian with pose manifold (R, p)
### Note
1. Migrate code from `ceres 1.x` to `ceres 2.x`, the main difference is using `Manifold` instead of `LocalParameterization`
    - `GlobalSize` => `AmbientSize`
    - `LocalSize` => `TangentSize`
    - `ComputeJacobian` => `PlusJacobian`
    - add `Minus` and `MinusJacobian`, but these two method don't used currently, could just `return true;` right now(see [github issue](https://github.com/ceres-solver/ceres-solver/issues/1003))

## g2o
1. g2o01_CircleFit \
    Using g2o for circle fitting
    Ref: https://github.com/RainerKuemmerle/g2o/blob/master/g2o/examples/data_fitting/circle_fit.cpp
1. g2o02_CircleFitWithoutRegister \
    Same as g2o01_CircleFit, but don't use `OptimizationAlgorithmFactory` to create solver.
1. g2o03_CurveFit \
    Use g2o for curve fitting
    Ref: https://github.com/RainerKuemmerle/g2o/blob/master/g2o/examples/data_fitting/curve_fit.cpp
   
## PCL
1. pcl01_Basic \
    Using a matrix to transform a point cloud
    Ref: https://pcl.readthedocs.io/projects/tutorials/en/latest/matrix_transform.html#matrix-transform
1. pcl02_PassThroughFilter \
    Filtering a point cloud using pass through filter
    Ref: https://pcl.readthedocs.io/projects/tutorials/en/latest/passthrough.html#passthrough
1. pcl03_VoxelGridFilter \
    Ref: https://pcl.readthedocs.io/projects/tutorials/en/latest/voxel_grid.html#voxelgrid
1. pcl04_IterativeClosestPoint \
    Using ICP(iterative closest point) to align two point clouds
    Ref: https://pcl.readthedocs.io/projects/tutorials/en/latest/iterative_closest_point.html
1. pcl05_IncrementalRegisterClouds \
    Incrementally register pairs of clouds
    Ref: https://pcl.readthedocs.io/projects/tutorials/en/latest/pairwise_incremental_registration.html#pairwise-incremental-registration


## algorithm
Some algorithm
1. alg01_DBSCAN \
    DBSCAN implementation using C++
    Ref: https://en.wikipedia.org/wiki/DBSCAN
1. alg02_BinarySearch \
    Search value $y_1$, $y_2$ in data to ensure $y_1 \le x < y_2$


## math
1. math01_CatmullRomSpline \
    Curve fitting using Catmull-Rom Spline
    - `scripts/analyze.py`: draw the fitting resulting
    - `scripts/analyze.ipynb`: symbol verification: cannot insert a control point into CR spline and also keep curve unchanged

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
1. boost07_Process \
    Start multiple process from current process using `boost::process`

### geometry
1. geometry01_ConvexHull \
    Using `boost::geometry` to calculate the 2D convex hull
    Ref: 
        - https://zhuanlan.zhihu.com/p/508781004
        - https://live.boost.org/doc/libs/1_85_0/libs/geometry/doc/html/geometry/reference/algorithms/convex_hull/convex_hull_2.html

### asio
1. asio01_Basic \
    Basic skills and concepts required for asio.
    Ref: https://www.boost.org/doc/libs/1_81_0/doc/html/boost_asio/tutorial.html, basic skills
1. asio02_SyncTcpClient \
    Synchronous TCP client.
    Ref: https://www.boost.org/doc/libs/1_81_0/doc/html/boost_asio/tutorial/tutdaytime1.html
1. asio03_SyncTcpServer \
    Synchronous TCP server.
    Ref: https://www.boost.org/doc/libs/1_81_0/doc/html/boost_asio/tutorial/tutdaytime2.html
1. asio04_AsyncTcpServer    \
    Asynchronous TCP server.
    Ref: https://www.boost.org/doc/libs/1_81_0/doc/html/boost_asio/tutorial/tutdaytime3.html
1. asio05_SyncUdpClient \
    Synchronous UDP client.
    Ref: https://www.boost.org/doc/libs/1_81_0/doc/html/boost_asio/tutorial/tutdaytime4.html
1. asio06_SyncUdpServer \
    Synchronous UDP server.
    Ref: https://www.boost.org/doc/libs/1_81_0/doc/html/boost_asio/tutorial/tutdaytime5.html
1. asio07_AsyncUdpServer    \
    Asynchronous UDP server.
    Ref: https://www.boost.org/doc/libs/1_81_0/doc/html/boost_asio/tutorial/tutdaytime6.html
1. asio08_AsyncTcpUdpServer \
    A combined TCP/UDP asynchronous server.
    Ref: https://www.boost.org/doc/libs/1_81_0/doc/html/boost_asio/tutorial/tutdaytime7.html
1. asio09_MulticastSender  \
    Multicast UDP sender using `asio`, based on boost1.58.
1. asio10_MulticastReceiver    \
    Multicast UDP receiver using `asio`, based on boost1.58.
1. asio11_ObtainWebPage \
    Obtain web page content using `boost::asio`

### beast
1. beast01_SimpleHttpClient   \
    Simple HTTP client.
    Ref: https://www.boost.org/doc/libs/1_81_0/libs/beast/doc/html/beast/quick_start/http_client.html
1. beast02_SimpleWebSocketClient    \
    Simple WebSocket client.
    Ref: https://www.boost.org/doc/libs/1_81_0/libs/beast/doc/html/beast/quick_start/websocket_client.html
1. beast03_SyncWebSocketClient  \
    Synchronous WebSocket client, don't work success.
    Ref: 
    - https://www.boost.org/doc/libs/1_81_0/libs/beast/example/websocket/server/sync/websocket_server_sync.cpp
    - https://www.ningto.com/post/3F43B74209B4DC0D11BCCADAC3E041CE
1. beast04_SyncWebSocketServer  \
    Synchronous WebSocket server.
    Ref: 
    - https://www.boost.org/doc/libs/1_81_0/libs/beast/example/websocket/client/sync/websocket_client_sync.cpp
    - https://www.ningto.com/post/3F43B74209B4DC0D11BCCADAC3E041CE
1. beast05_AsyncWebSocketClient \
    Asynchronous WebSocket client.
    Ref: https://www.boost.org/doc/libs/1_81_0/libs/beast/example/websocket/client/async/websocket_client_async.cpp
1. beast06_AsyncWebSocketServer \
    Asynchronous WebSocket server.
    Ref: https://www.boost.org/doc/libs/1_81_0/libs/beast/example/websocket/server/async/websocket_server_async.cpp
1. beast07_SyncHttpClient   \
    Synchronous HTTP client.
    Ref: https://www.boost.org/doc/libs/1_81_0/libs/beast/example/http/client/sync/http_client_sync.cpp
1. beast08_SyncHttpServer   \
    Synchronous HTTP server.
    Ref: https://www.boost.org/doc/libs/1_81_0/libs/beast/example/http/server/sync/http_server_sync.cpp
1. beast09_AsyncHttpClient  \
    Asynchronous HTTP client.
    Ref: https://www.boost.org/doc/libs/1_81_0/libs/beast/example/http/client/async/http_client_async.cpp
1. beast10_AsyncHttpServer  \
    Asynchronous HTTP server.
    Ref: https://www.boost.org/doc/libs/1_81_0/libs/beast/example/http/server/async/http_server_async.cpp

## cxxopts
Some study using [cxxopts](https://github.com/jarro2783/cxxopts) library

## spdlog
Some study using [spdlog](https://github.com/gabime/spdlog) library
1. spdlog01_Basic \
    Some basic log
1. spdlog02_MultiSink \
    Use `spdlog` with multi sink, the console only write warning log, and file write all
1. spdlog03_CustomerFormat \
    Add customer format flag.
1. spdlog04_UseInDll \
    Use spdlog in dll
### Programming Practice
1. "spdlog::info()" could not add "sourfile:location" flag in logging, while "SPDLOG_INFO" does.
1. No module name could be add to log, should setting it manually(just input into log message).

## json
some usage for `nlohmann json` library.
1. json01_Basic \
    Basic usage

## jsoncpp
some usage for `jsoncpp` library.
1. jsoncpp01_Basic \
    Basic usage

## YAML
1. YamlStudy    \
    Write and read yaml file using `yaml-cpp`.

## protobuf
Some usage about protobuf
1. protobuf01_Basic \
    Basic example
    Code to generate python file:
    ```sh
    protoc --proto_path ./proto --python_out . ./proto/cc/AddressBook.proto
    ```

## SqliteCpp
Basic SQLite usage using `SQLiteCpp`.

## SqliteOrm
Some usage about [SQLite ORM](https://github.com/fnc12/sqlite_orm)
1. SqliteOrm01_Basic    \
    Basic usage

## SqliteModernCpp
Some study code about [sqlite_modern_cpp](https://github.com/SqliteModernCpp/sqlite_modern_cpp)
1. SqliteModernCpp01_Basic \
    Basic usage

## archive
Compress or decompress file using `libarchive`
1. archive01_basic \
    Using libarchive to compress/decompress files

## httplib
Basic usage for httplib
1. http01_Basic \
    Basic client/server
1. http02_WebClient \
    Get web page content from url

## libhv
Some study code about [libhuv](https://github.com/ithewei/libhv)
1. hv01_TcpServer \
    Basic TCP server
1. hv02_TcpClient \
    Basic TCP Client
1. hv03_HttpServer \
    Basic HTTP server
1. hv04_WebSocketServer \
    Basic web socket server
1. hv05_WebSocketClient \
    Basic web socket client

## TurboJpeg
Some study using [TurboJpeg](https://libjpeg-turbo.org/)
1. TurboJpeg01_CompressBenchmark \
    Compress YUYV image to JPEG using different method, and calculate the average used time.


## Video Process
Some example about video process
1. VideoProcess01_SimpleX264Encoder \
    Compress YUYV image to H264 video using libx264
1. VideoProcess02_X264Encoder \
    Video encoder(YUV => H264) using libx264
1. VideoProcess03_FFmpegEncoder \
    Video encoder(YUV => H264) using FFmpeg
1. VideoProcess04_FFmpegDecoder \
    Video dncoder(H264 => YUV) using FFmpeg

## system
Some system applications
1. system01_GetNetDeviceInfo \
    Get the net device info, include IPv4 address, mac address
    Ref:
        1. https://segmentfault.com/a/1190000005138358

## pybind
Some code about `pybind11`
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