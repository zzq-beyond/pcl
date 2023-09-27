#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/io/ply_io.h>
#include<pcl/point_cloud.h>
#include<pcl/console/parse.h>
#include<pcl/common/transforms.h>
#include<pcl/visualization/pcl_visualizer.h>

void showHelp(char *program_name){
    std::cout << std::endl;
    std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
    std::cout << "-h:  Show this help." << std::endl;
}

int main(int argc, char **argv){

    if(pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help")){
        showHelp(argv[0]);
        return 0;
    }
    
    std::vector<int> filenames;
    bool file_is_pcd = false;

    filenames = pcl::console::parse_file_extension_argument(argc, argv, ".ply");
    if(filenames.size() != 1){

        filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
        if(filenames.size() != 1){
            showHelp(argv[0]);
            return -1;
        }
        else{
            file_is_pcd = true;
        }
    }
   //在参数中寻找pcd或ply文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    if(file_is_pcd){
        if(pcl::io::loadPCDFile(argv[filenames[0]], *source_cloud) < 0){
            std::cout << "Error loading point cloud" << argv[filenames[0]] << std::endl;
            showHelp(argv[0]);
            return -1;
        }
    }
    else{
        if (pcl::io::loadPLYFile(argv[filenames[0]], *source_cloud) < 0)
        {
            std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl
                      << std::endl;
            showHelp(argv[0]);
            return -1;
        }
    }

    //创建4x4单位矩阵
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

    float theta = M_PI / 4;
    transform_1(0,0) = std::cos(theta);
    transform_1(0,1) = -sin(theta);
    transform_1(1,0) = sin(theta);
    transform_1(1,1) = std::cos(theta);

    transform_1(0,3) = 2.5;

    printf("Method #1: using a Matrix4f\n");
    std::cout << transform_1 << std::endl;

    //第二种方法
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.translation() << 2.5, 0.0, 0.0;
    transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));

    printf("Method #2: using Affine3f\n");
    std::cout << transform_2.matrix() << std::endl;

    //执行transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_1);

    //可视化
    printf("\npoint cloud colors: white = original point cloud\n "
    "                             red = transformed point cloud");
    pcl::visualization::PCLVisualizer viewer("Matrix transformed example");

    //定义点云的rgb颜色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(source_cloud, 255,255,255);
    viewer.addPointCloud(source_cloud, source_cloud_color_handler, "original cloud");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler(transformed_cloud, 230, 20, 20);
    viewer.addPointCloud(transformed_cloud, transformed_cloud_color_handler, "transformed cloud");

    viewer.addCoordinateSystem(1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05,0.05,0.05, 0);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");

    while(!viewer.wasStopped()){
        viewer.spinOnce();
    }

    return 0;


}