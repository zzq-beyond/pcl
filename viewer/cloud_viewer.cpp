#include <iostream>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

int user_data;

void viewerOneOff (pcl::visualization::PCLVisualizer& viewer){

    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    pcl::PointXYZ o;
    o.x = 0.0;
    o.y = 0.0;
    o.z = 0.0;
    viewer.addSphere(o, 0.01,"aphere", 0);
    std::cout << "I only run once" << std::endl;
}

void viewerPsycho (pcl::visualization::PCLVisualizer& viewer){
    
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop" << count++;
    viewer.removeShape("text", 0);
    viewer.addText(ss.str(), 200, 300, "text", 0);

    user_data++;
}

int main(){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../pcd/cube.pcd", *cloud);

    pcl::visualization::CloudViewer viewer("Cloud viewer");

    viewer.showCloud(cloud);

    viewer.runOnVisualizationThreadOnce(viewerOneOff);
    // viewer.runOnVisualizationThread(viewerPsycho);

    while (!viewer.wasStopped()){
        user_data++;
    }

    return 0;
}
    
