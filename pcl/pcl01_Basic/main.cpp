#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>

using namespace std;
using namespace Eigen;
using namespace pcl;

// display the help
void showHelp(const char* programName) {
    cout << endl << "Usage: " << programName << " cloud_filename.[pcd|ply]" << endl;
    cout << "-h:    Show this help." << endl;
}

int main(int argc, const char* argv[]) {
    // show help
    if (console::find_switch(argc, argv, "-h") || console::find_switch(argc, argv, "--help")) {
        showHelp(argv[0]);
        return 0;
    }

    // fetch point cloud filename in arguments
    vector<int> filenames;
    bool fileIsPcd{false};
    filenames = console::parse_file_extension_argument(argc, argv, ".ply");
    if (filenames.size() != 1) {
        filenames = console::parse_file_extension_argument(argc, argv, ".pcd");
        if (filenames.size() != 1) {
            showHelp(argv[0]);
            return -1;
        } else {
            fileIsPcd = true;
        }
    }

    // load file
    PointCloud<PointXYZ>::Ptr sourceCloud(new PointCloud<PointXYZ>());
    if (fileIsPcd) {
        if (io::loadPCDFile(argv[filenames[0]], *sourceCloud) < 0) {
            cout << "Error loading point cloud " << argv[filenames[0]] << endl << endl;
            showHelp(argv[0]);
            return -1;
        }
    } else {
        if (io::loadPLYFile(argv[filenames[0]], *sourceCloud) < 0) {
            cout << "Error loading point cloud " << argv[filenames[0]] << endl << endl;
            showHelp(argv[0]);
            return -1;
        }
    }

    // METHOD 01, using a Matrix4f, perfect to understand but error prone!
    Matrix4f transform01 = Matrix4f::Identity();
    float theta = M_PI / 4;  // the angle of rotation in radians
    transform01(0, 0) = cos(theta);
    transform01(0, 1) = -sin(theta);
    transform01(1, 0) = sin(theta);
    transform01(1, 1) = cos(theta);
    transform01(0, 3) = 2.5;  // translation on the x axis
    cout << "Method #1: using a Matrix4f" << endl;
    cout << transform01 << endl;

    // METHOD 02, using Affine3f, this method is easier and less error prone
    Affine3f transform02 = Affine3f::Identity();
    transform02.translation() << 2.5, 0, 0;
    transform02.rotate(AngleAxisf(theta, Vector3f::UnitZ()));
    cout << endl << "Method #2: using an Affine3f" << endl;
    cout << transform02.matrix() << endl;

    // executing the transformation
    PointCloud<PointXYZ>::Ptr transformedCloud(new PointCloud<PointXYZ>());
    transformPointCloud(*sourceCloud, *transformedCloud, transform02);

    // visualization
    cout << endl
         << "Point cloud colors: \twhite = original point cloud" << endl
         << "\t\t\t red  = transformed point cloud" << endl;
    visualization::PCLVisualizer viewer("Matrix Transformation Example");
    // define RGB colors for the point cloud
    visualization::PointCloudColorHandlerCustom<PointXYZ> sourceCloudColorHandler(sourceCloud, 255, 255, 255);
    viewer.addPointCloud(sourceCloud, sourceCloudColorHandler, "Original Cloud");
    visualization::PointCloudColorHandlerCustom<PointXYZ> transformedCloudColorHandler(transformedCloud, 230, 20, 20);
    viewer.addPointCloud(transformedCloud, transformedCloudColorHandler, "Transformed Cloud");
    viewer.addCoordinateSystem(1.0, "Cloud", 0);
    viewer.setSize(800, 600);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);  // setting background to a dark gray
    viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Original Cloud");
    viewer.setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Transformed Cloud");

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }

    return 0;
}