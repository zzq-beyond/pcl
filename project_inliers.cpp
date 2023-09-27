#include<iostream>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/ModelCoefficients.h>
#include<pcl/filters/project_inliers.h>

int main(){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>());

    cloud->width = 5;
    cloud->height = 1;
    cloud->resize(cloud->width * cloud->height);

    for(auto& point: *cloud){
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    std::cerr << "cloud befor projection: " << std::endl;
    for(auto& point: *cloud)
        std::cerr << "  " << point.x << " " << point.y << " " << point.z << " " << std::endl;
    
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients::Ptr);
    
    
    return 0;
}