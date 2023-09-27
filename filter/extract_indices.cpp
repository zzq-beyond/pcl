#include<iostream>
#include<pcl/ModelCoefficients.h>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/sample_consensus/method_types.h>
#include<pcl/sample_consensus/model_types.h>
#include<pcl/segmentation/sac_segmentation.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/filters/extract_indices.h>


void func(){
    //加载点云数据
    pcl::PCLPointCloud2::Ptr cloud_blob (new  pcl::PCLPointCloud2), 
                             cloud_filter_blob (new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>),
                                        cloud_p (new pcl::PointCloud<pcl::PointXYZ>),
                                        cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCDReader reader;
    reader.read("../pcd/table.pcd", *cloud_blob);
    std::cerr << "PointCloud before filtering: " << cloud_blob->height * cloud_blob->width << " data points." << std::endl;
    //创建下采样滤波器
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud_blob);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_filter_blob);
    //转为模板点云
    pcl::fromPCLPointCloud2(*cloud_filter_blob, *cloud_filtered);
    std::cerr << "Pointcloud after filtering: " <<  cloud_filter_blob->height * cloud_filter_blob->width << " data points." << std::endl;
    //保存下采样后的点云
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>("../pcd/table_dawnsample2.pcd", *cloud_filtered, false);

    pcl::ModelCoefficients::Ptr cofficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    //创建分割对象
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);//可选
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);
    //创建提取对象
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    int i=0, nr_points = (int) cloud_filtered->size();
    //原点云的30%会留下来
    while(cloud_filtered->size() > 0.3*nr_points){
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *cofficients);
        if(inliers->indices.size() == 0){
            std::cerr << "Could not estimate a planner model for the given dataset" << std::endl;
            break;
        }

        //提取inliers
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_p);
        std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

        std::stringstream ss;
        ss << "../pcd/table_plane_" << i << ".pcd";
        writer.write<pcl::PointXYZ> (ss.str(), *cloud_p, false);

        extract.setNegative(true);
        extract.filter(*cloud_f);
        cloud_filtered.swap(cloud_f);
        i++;
    }
}

int main(){
    func();
}