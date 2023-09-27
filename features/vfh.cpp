#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include <pcl/io/pcd_io.h>


int main(){

    //定义
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>());
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::PointCloud<pcl::VFHSignature308>::Ptr out_vfh (new pcl::PointCloud<pcl::VFHSignature308>);

    //加载点云数据
    pcl::io::loadPCDFile("../pcd/table_plane_0.pcd", *cloud);

    //计算法线
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.05);
    ne.compute(*normals);

    //计算vfh
    pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
    vfh.setInputCloud(cloud);
    vfh.setInputNormals(normals);
    vfh.setSearchMethod(tree);
    vfh.setRadiusSearch(0.05);

    //验证法线有效性
    for(int i=0; i < normals->size(); i++){
        if(!pcl::isFinite<pcl::Normal>((*normals)[i]))
            PCL_WARN("normals[%d] is not finite\n", i);
    }

    //计算最终结果
    vfh.compute(*out_vfh);

    //打印每一个点的125维特征
    for (int i = 0; i < out_vfh->size(); i++)
        std::cout << (*out_vfh)[i] << std::endl;

}