//点云的切割功能（按照给定的坐标轴和区间切割） 
#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/filters/passthrough.h>

int main(){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../pcd/result_2.pcd", *cloud);
    // cloud->width = 5;
    // cloud->height = 1;
    // cloud->resize(cloud->width * cloud->height);

    // for(auto& point: *cloud){
    //     point.x = 1024 * rand() / (RAND_MAX + 1.0f);
    //     point.y = 1024 * rand() / (RAND_MAX + 1.0f);
    //     point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    // }

    // std::cerr << "cloud befor filtering:" << std::endl;
    // for(const auto& point: *cloud){
    //     std::cerr << "  " << point.x << " " << point.y << " " << point.z << " " << std::endl;
    // }
    
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-30,100);
    pass.setNegative(true);
    pass.filter(*cloud_filtered);
    pcl::io::savePCDFileASCII("../pcd/half.pcd", *cloud_filtered);

    // std::cerr << "cloud after filtering:" << std::endl;
    // for(const auto& point: *cloud_filtered){
    //     std::cerr << "  " << point.x << " " << point.y << " " << point.z << " " << std::endl;
    // }

    return 0;
}