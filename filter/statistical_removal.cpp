#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/filters/statistical_outlier_removal.h>

int main(){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>());

    pcl::PCDReader reader;
    reader.read<pcl::PointXYZ>("../pcd/table_scene_lms400.pcd", *cloud);

    std::cerr << "cloud befor filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);

    std::cerr << "cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> ("../pcd/table_scene_lms400_inliers.pcd", *cloud_filtered, false);

    sor.setNegative(true);
    sor.filter(*cloud_filtered);
    writer.write<pcl::PointXYZ> ("../pcd/table_scene_lms400_outliers.pcd", *cloud_filtered, false);

    return 0;
}