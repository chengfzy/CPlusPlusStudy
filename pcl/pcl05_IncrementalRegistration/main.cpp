#include <fmt/format.h>
#include <fmt/ostream.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <common/common.hpp>
#include <iostream>

using namespace std;
using namespace fmt;
using namespace Eigen;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::visualization;
using namespace ::common;
namespace fs = boost::filesystem;

using MyPoint = PointXYZ;
using MyPointCloud = PointCloud<MyPoint>;
using MyPointNormal = PointNormal;
using MyPointCloudWithNormals = PointCloud<MyPointNormal>;

struct PCD {
    string name;
    MyPointCloud::Ptr cloud;

    PCD() : cloud(new MyPointCloud) {}
};

// point representation for (x,y,z,curvature)
struct MyPointRepresentation : public PointRepresentation<MyPointNormal> {
    MyPointRepresentation() { nr_dimensions_ = 4; }

    void copyToFloatArray(const MyPointNormal& p, float* out) const override {
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[3] = p.curvature;
    }
};

void pairAlign(const MyPointCloud::Ptr& source, const MyPointCloud::Ptr& target, const MyPointCloud::Ptr& output,
               Eigen::Matrix4f& transform, PCLVisualizer& viewer, int viewpoint, bool downSample = false) {
    VoxelGrid<MyPoint> filter;
    MyPointCloud::Ptr sourcePoint(new MyPointCloud);
    MyPointCloud::Ptr targetPoint(new MyPointCloud);
    if (downSample) {
        // down sample for consistency and speed, if load large datasets
        filter.setLeafSize(0.05, 0.05, 0.05);
        filter.setInputCloud(source);
        filter.filter(*sourcePoint);
        filter.setInputCloud(target);
        filter.filter(*targetPoint);
    } else {
        sourcePoint = source;
        targetPoint = target;
    }

    // compute surface normals and curvature
    MyPointCloudWithNormals::Ptr sourcePointWithNormals(new MyPointCloudWithNormals);
    MyPointCloudWithNormals::Ptr targetPointWithNormals(new MyPointCloudWithNormals);
    NormalEstimation<MyPoint, MyPointNormal> normEstimation;
    search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>());
    normEstimation.setSearchMethod(tree);
    normEstimation.setKSearch(30);
    normEstimation.setInputCloud(sourcePoint);
    normEstimation.compute(*sourcePointWithNormals);
    copyPointCloud(*sourcePoint, *sourcePointWithNormals);
    normEstimation.setInputCloud(targetPoint);
    normEstimation.compute(*targetPointWithNormals);
    copyPointCloud(*target, *targetPointWithNormals);

    // instantiate our custom point representation
    MyPointRepresentation pointRepresentation;
    float alpha[4] = {1.0, 1.0, 1.0, 1.0};
    pointRepresentation.setRescaleValues(alpha);

    // align
    IterativeClosestPointNonLinear<MyPointNormal, MyPointNormal> reg;
    reg.setTransformationEpsilon(1E-6);
    reg.setMaxCorrespondenceDistance(0.1);
    reg.setPointRepresentation(std::make_shared<MyPointRepresentation>(pointRepresentation));
    reg.setInputSource(sourcePointWithNormals);
    reg.setInputTarget(targetPointWithNormals);

    // run the same optimization in a loop and visualize the results
    Matrix4f Ti = Matrix4f::Identity(), prev = Matrix4f::Zero();
    MyPointCloudWithNormals::Ptr result = sourcePointWithNormals;
    reg.setMaximumIterations(2);
    for (size_t i = 0; i < 30; ++i) {
        LOG(INFO) << format("iteration {}...", i);
        sourcePointWithNormals = result;

        // estimate
        reg.setInputSource(sourcePointWithNormals);
        reg.align(*result);

        // accumulate transformation between each iteration
        Ti = reg.getFinalTransformation() * Ti;

        // if the difference between this transform and the previous one is smaller than the threshold, refine the
        // process by reducing the maximal correspondence distance
        if (abs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon()) {
            reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);
        }
        prev = reg.getLastIncrementalTransformation();

        // visualize current state
        viewer.removePointCloud("source");
        viewer.removePointCloud("target");
        PointCloudColorHandlerGenericField<MyPointNormal> sourceResultHandler(sourcePointWithNormals, "curvature");
        if (!sourceResultHandler.isCapable()) {
            LOG(WARNING) << "cannot create curvature color handler";
        }
        viewer.addPointCloud(sourcePointWithNormals, sourceResultHandler, "source", viewpoint);
        PointCloudColorHandlerGenericField<MyPointNormal> targetResultHandler(targetPointWithNormals, "curvature");
        if (!targetResultHandler.isCapable()) {
            LOG(WARNING) << "cannot create curvature color handler";
        }
        viewer.addPointCloud(targetPointWithNormals, targetResultHandler, "target", viewpoint);
        viewer.spinOnce();
    }

    // get the transformation from target to source
    Matrix4f targetToSource = Ti.inverse();

    // transform target back in source frame
    transformPointCloud(*target, *output, targetToSource);

    // add to second viewpoint
    viewer.removePointCloud("source");
    viewer.removePointCloud("target");
    PointCloudColorHandlerCustom<MyPoint> sourceHandler(source, 0, 255, 0);
    PointCloudColorHandlerCustom<MyPoint> targetHandler(output, 255, 0, 0);
    viewer.addPointCloud(source, sourceHandler, "source", viewpoint);
    viewer.addPointCloud(output, targetHandler, "target", viewpoint);
    LOG(INFO) << format("press \"q\" to begin the registration");
    viewer.spin();
    viewer.removePointCloud("source");
    viewer.removePointCloud("target");

    // add the source to the transformed target
    *output += *source;

    transform = targetToSource;
}

int main(int argc, const char* argv[]) {
    initLog(argc, argv);

    // input
    fs::path inputFolder{"./data/PointCloud"};
    string fileNamePrefix{"capture"};

    // load data
    LOG(INFO) << Paragraph("Load Data");
    vector<PCD, Eigen::aligned_allocator<PCD>> data;
    inputFolder = fs::weakly_canonical(inputFolder);
    CHECK(fs::exists(inputFolder)) << format("input data folder {} don't exist", inputFolder);
    LOG(INFO) << format("load point cloud from folder: {}", inputFolder);
    for (auto it = fs::directory_iterator(inputFolder); it != fs::directory_iterator(); ++it) {
        if (fs::is_regular_file(it->path()) && it->path().extension() == ".pcd" &&
            boost::starts_with(it->path().stem().string(), "capture")) {
            LOG(INFO) << format("load point cloud from file: {}", it->path());
            PCD m;
            m.name = it->path().stem().string();
            loadPCDFile(it->path().string(), *m.cloud);
            Indices index;
            removeNaNFromPointCloud(*m.cloud, *m.cloud, index);
            data.emplace_back(move(m));
        }
    }
    LOG(INFO) << format("load {} point clouds", data.size());
    CHECK(!data.empty()) << format("cannot found any valid point clouds");

    // create a PCL visualizer object
    PCLVisualizer viewer("Pairwise Incremental Registration");
    int viewport1{0}, viewport2{0};
    viewer.createViewPort(0.0, 0, 0.5, 1.0, viewport1);
    viewer.createViewPort(0.5, 0, 1.0, 1.0, viewport2);

    // incremental registration
    MyPointCloud::Ptr result(new MyPointCloud), source, target;
    Matrix4f globalTransform = Matrix4f::Identity();
    Matrix4f pairTransform = Matrix4f::Zero();
    for (size_t i = 1; i < data.size(); ++i) {
        source = data[i - 1].cloud;
        target = data[i].cloud;

        // display source and target point in first viewport
        viewer.removePointCloud("vp1 source");
        viewer.removePointCloud("vp1 target");
        PointCloudColorHandlerCustom<MyPoint> sourceHandler(source, 0, 255, 0);
        PointCloudColorHandlerCustom<MyPoint> targetHandler(target, 255, 0, 0);
        viewer.addPointCloud(source, sourceHandler, "vp1 source", viewport1);
        viewer.addPointCloud(target, targetHandler, "vp1 target", viewport1);
        LOG(INFO) << format("press \"q\" to begin the registration");
        viewer.spin();

        // pair alignment
        LOG(INFO) << format("[{}] aligning {}({}) with {}({})", i, data[i - 1].name, source->size(), data[i].name,
                            source->size());
        MyPointCloud::Ptr temp(new MyPointCloud);
        pairAlign(source, target, temp, pairTransform, viewer, viewport2, true);

        // transform current pair into the global transform
        transformPointCloud(*temp, *result, globalTransform);
        LOG(INFO) << format("result size: {}", result->size());

        // update global transform
        globalTransform *= pairTransform;

        // save aligned pair, transformed into the first cloud's frame
        auto savePath = inputFolder / format("result{:02d}.pcd", i);
        savePCDFile(savePath.string(), *result, true);
    }

    closeLog();
    return 0;
}