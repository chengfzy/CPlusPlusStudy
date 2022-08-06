# g2o Study

# find package
find_package(g2o QUIET)
find_package(Eigen3 QUIET)
find_package(Sophus QUIET)

if(${g2o_FOUND} AND ${Eigen3_FOUND} AND ${Sophus_FOUND})
    list(APPEND DEPEND_INCLUDES ${EIGEN3_INCLUDE_DIRS} ${SOPHUS_INCLUDE_DIRS})
    list(APPEND DEPEND_LIBS ${g2o_LIBS} ${PRJ_LIB_COMMON})
    # add all g2o lib to list, please note that the ${g2o_INCLUDE_DIRS}/${G2O_INCLUDE_DIRS}/${G2O_LIBRARIES}/${g2o_LIBS} 
    # are not defined in the cmake file, only ${g2o_FOUND} are defined
    list(APPEND g2o_LIBS g2o::freeglut_minimal g2o::g2o_ceres_ad g2o::stuff g2o::opengl_helper g2o::core
        g2o::g2o_cli_library g2o::g2o_hierarchical_library g2o::g2o_simulator_library
        g2o::types_slam2d g2o::types_slam2d_addons g2o::types_data g2o::types_sclam2d
        g2o::types_slam3d g2o::types_slam3d_addons g2o::types_sba g2o::types_icp g2o::types_sim3 g2o::solver_pcg
        g2o::solver_dense g2o::solver_slam2d_linear g2o::solver_structure_only g2o::solver_csparse g2o::csparse_extension
        g2o::solver_cholmod g2o::solver_eigen)
    list(APPEND g2o_LIBS gomp)

    add_subdirectory(g2o01_CircleFit)
    add_subdirectory(g2o02_CircleFitWithoutRegister)
    add_subdirectory(g2o03_CurveFit)
endif()