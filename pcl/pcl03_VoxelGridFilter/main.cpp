#include <fmt/format.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>

using namespace std;
using namespace fmt;
using namespace Eigen;
using namespace pcl;

int main(int argc, const char* argv[]) {
    PCLPointCloud2::Ptr cloud(new PCLPointCloud2);
    PCLPointCloud2::Ptr filteredCloud(new PCLPointCloud2);

    // fill in the cloud data
    PCDReader reader;
    reader.read("./data/PointCloud/table_scene_lms400.pcd", *cloud);
    cout << format("PointCloud before filtering: {} data points ({})", cloud->width * cloud->height,
                   getFieldsList(*cloud))
         << endl;

    // create the filtering object
    VoxelGrid<PCLPointCloud2> filter;
    filter.setInputCloud(cloud);
    filter.setLeafSize(0.01, 0.01, 0.01);
    filter.filter(*filteredCloud);
    cout << format("PointCloud after filtering: {} data points ({})", filteredCloud->width * filteredCloud->height,
                   getFieldsList(*filteredCloud))
         << endl;

    // write to file
    PCDWriter writer;
    writer.write("./data/PointCloud/table_scene_lms400_downsampled.pcd", *filteredCloud, Vector4f::Zero(),
                 Quaternionf::Identity(), false);

    return 0;
}