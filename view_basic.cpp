#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>//点云可视化头文件
#include <pcl/io/pcd_io.h>//pcd文件输入/输出
#include <pcl/visualization/range_image_visualizer.h>//rangeImage可视化头文件
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>//命令行参数解析
#include <boost/thread/thread.hpp>//多线程文件

 
int main (int argc, char** argv){
 
  //读入点云数据
  const std::string file = argv[1];
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile(file, *cloud_in);

  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("CloudPointView"));
  viewer->initCameraParameters();
 
  int v1(0);
  //viewer->createViewPort(0,0,0.5,1,v1);
  viewer->setBackgroundColor(0,0,0,v1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(cloud_in->makeShared(),0,255,0);
  viewer->addPointCloud(cloud_in->makeShared(),color1,"pointCloud",v1);
  
  viewer->addCoordinateSystem();
  viewer->spin();     
 
  return  0 ;
}