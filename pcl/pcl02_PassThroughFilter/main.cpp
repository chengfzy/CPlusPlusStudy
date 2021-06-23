#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <iostream>

using namespace std;
using namespace Eigen;
using namespace pcl;

int main(int argc, const char* argv[]) {
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr filteredCloud(new PointCloud<PointXYZ>);

    // fill in the cloud data
    cloud->width = 5;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (auto& point : *cloud) {
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    cout << "Cloud before filtering:" << endl;
    for (const auto& point : *cloud) {
        cout << "\t" << point.x << ", " << point.y << ", " << point.z << endl;
    }

    // create the filtering object
    PassThrough<PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 1.0);
    pass.filter(*filteredCloud);

    cout << endl << "Cloud after filtering:" << endl;
    for (const auto& point : *filteredCloud) {
        cout << "\t" << point.x << ", " << point.y << ", " << point.z << endl;
    }

    return 0;
}