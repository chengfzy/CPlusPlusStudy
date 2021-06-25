#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>

using namespace std;
using namespace Eigen;
using namespace pcl;

int main(int argc, const char* argv[]) {
    PCLPointCloud2::Ptr cloud(new PCLPointCloud2);
    PCLPointCloud2::Ptr filteredCloud(new PCLPointCloud2);

    // fill in the cloud data
    PCDReader reader;
    reader.read("./data/table_scene_lms400.pcd", *cloud);
    cout << "PointCloud before filttering: " << cloud->width * cloud->height << "data points (" << getFieldsList(*cloud)
         << ")." << endl;

    // create the filtering object
    VoxelGrid<PCLPointCloud2> filter;
    filter.setInputCloud(cloud);
    filter.setLeafSize(0.01, 0.01, 0.01);
    filter.filter(*filteredCloud);

    cout << "PointCloud after filttering: " << filteredCloud->width * filteredCloud->height << "data points ("
         << getFieldsList(*cloud) << ")." << endl;

    // write to file
    PCDWriter writer;
    writer.write("./data/table_scene_lms400_downsampled.pcd", *filteredCloud, Vector4f::Zero(), Quaternionf::Identity(),
                 false);

    return 0;
}