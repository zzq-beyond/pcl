#include<iostream>
#include<fstream>
#include<string>
#include<vector>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h> 
using namespace std;
 
int main()
{
	fstream modelRead;
	pcl::PointCloud<pcl::PointXYZ> cloud;
	// pcl::PointCloud<pcl::PointXYZRGBA> cloud; //根据不通的形式XYZ 或者 XYZRGBA 等
	pcl::PCDWriter writer;
 
	modelRead.open("../pcd/half.txt", std::ios_base::in);
	pcl::PointXYZ pclPnt;
	// pcl::PointXYZRGBA pclPnt;
	while (!modelRead.eof())
	{
		modelRead >> pclPnt.x >> pclPnt.y >> pclPnt.z;
		// modelRead >> pclPnt.x >> pclPnt.y >> pclPnt.z >> pclPnt.rgba;
		cloud.push_back(pclPnt);
	}
	modelRead.close();
	writer.write("../pcd/demo.pcd", cloud);
 
	return 0;
}
