#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//pcd->txt
void pcd2txt(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const char *filename)
{
	FILE* wc = fopen(filename, "w");
 
	int sizepcd = cloud->points.size();
	for (int i = 0; i < sizepcd; i++){
        
		//fprintf(stdout, "%f\t%f\t%f\n", cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);  \t 相当于空格
		fprintf(wc, "%f\t%f\t%f\n", cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);  //标准输出流   stdio.h   写
	}
	fclose(wc);
}

int main(){
    const char* txt = "../pcd/half.txt";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../pcd/half.pcd", *cloud);
    pcd2txt(cloud, txt);
}