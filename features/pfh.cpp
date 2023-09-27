#include <pcl/point_types.h>
#include <pcl/features/pfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <typeinfo>

int main(){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>());

    pcl::io::loadPCDFile("../pcd/table_plane_0.pcd", *cloud);
    // std::cout << cloud << std::endl;
    // std::cout << *cloud << std::endl;

    //compute normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    ne.setRadiusSearch(0.05);
    ne.compute(*normals);

    //compute PFH
    pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
    pcl::PointCloud<pcl::PFHSignature125>::Ptr out_pfh (new pcl::PointCloud<pcl::PFHSignature125>());

    pfh.setInputCloud(cloud);
    pfh.setInputNormals(normals);
    pfh.setSearchMethod(tree);
    pfh.setRadiusSearch(0.05);

    //compute之前先验证一下normals的有效性
    for(int i=0; i < normals->size(); i++){
        if(!pcl::isFinite<pcl::Normal>((*normals)[i]))
            PCL_WARN("normals[%d] is not finite\n", i);
    }

    //compute results
    pfh.compute(*out_pfh);

    //打印每一个点的125维特征
    for (int i = 0; i < out_pfh->size(); i++)
        std::cout << (*out_pfh)[i] << std::endl;
}