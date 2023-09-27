#include <pcl/io/pcd_io.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>

int main(){

    //定义
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>());
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr out_fpfh (new pcl::PointCloud<pcl::FPFHSignature33>);

    //加载点云数据
    pcl::io::loadPCDFile("../pcd/table_plane_0.pcd", *cloud);

    //计算法线
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.05);
    ne.compute(*normals);

    //计算fpfh
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(cloud);
    fpfh.setInputNormals(normals);
    fpfh.setSearchMethod(tree);
    fpfh.setRadiusSearch(0.05);

    //验证法线有效性
    for(int i=0; i < normals->size(); i++){
        if(!pcl::isFinite<pcl::Normal>((*normals)[i]))
            PCL_WARN("normals[%d] is not finite\n", i);
    }

    //计算最终结果
    fpfh.compute(*out_fpfh);

    //打印每一个点的125维特征
    for (int i = 0; i < out_fpfh->size(); i++)
        std::cout << (*out_fpfh)[i] << std::endl;
}