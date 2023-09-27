//下采样，改变点云的密度
#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/filters/voxel_grid.h>

int main(){
    
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2());

    pcl::PCDReader reader;
    reader.read("../pcd/table_scene_lms400.pcd", *cloud);

    std::cerr << "point cloud befor flitering:" << cloud->width * cloud->height <<
    " data points (" << pcl::getFieldsList(*cloud) << ")" << std::endl;

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud);
    // sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.setFilterLimits(200.0, 1000.0);
    sor.filter(*cloud_filtered);

    std::cerr << "point cloud after flitering:" << cloud_filtered->width * cloud_filtered->height <<
    " data points (" << pcl::getFieldsList(*cloud_filtered) << ")" << std::endl;

    pcl::PCDWriter writer;
    writer.write("../pcd/table_downsample.pcd", *cloud_filtered, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity (), false);

    return 0;
}