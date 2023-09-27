#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>

int main(int argc, char **argv){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::io::loadPCDFile("../pcd/cube.pcd", *cloud);
        
    for(int i = 0; i < cloud->size(); i++){
        std::cout << (*cloud)[i] << std::endl;
    }
    std::cout << "number of points: " << cloud->points.size() << std::endl;

    //随机生成点云数据
    // for(size_t i=0; i < cloud->points.size(); ++i){
    //     std::cout << " " << cloud->points[i].x
    //               << " " << cloud->points[i].y
    //               << " " << cloud->points[i].z << std::endl;
    // }
    // return 0;
}