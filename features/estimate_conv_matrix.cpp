#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <Eigen/Core>
#include <typeinfo>

int main(){
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../pcd/table.pcd",*cloud);
    std::vector<int> index;
    index.push_back(2);
    Eigen::Matrix3f conv_matrix;
    Eigen::Vector4f xyz_centroid;

    pcl::compute3DCentroid(*cloud, xyz_centroid);
    pcl::computeCovarianceMatrix(*cloud, xyz_centroid, conv_matrix);
    std::cout <<"---------------------------"<< std::endl;
    std::cout << conv_matrix << std::endl;
    std::cout <<"---------------------------"<< std::endl;
    std::cout << xyz_centroid << std::endl;
}