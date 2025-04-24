//Name: BAO Sheng
//Date: 2023-07
#include <iostream> //标准输入输出流
#include <iomanip>
#include <string>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>

#include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件
#include <pcl/point_types.h> //PCL对各种格式的点的支持头文件
#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
//use: ./Riegl2pcd ./230629_074854.csv ./scans
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;                  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

struct PointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

int main(int argc, char** argv)
{
	if (argc < 3)
	{
		std::cerr << "Not enough parameters!" << std::endl;
		return (-1);
	}
	std::string file_path = argv[1];
	std::string save_dir = argv[2];
	//read csv file
	std::ifstream f;
	f.open(file_path.c_str());
	if (!f.is_open())
	{
		std::cerr << " can't open csv file " << std::endl;
		exit;
	}
	double time;
	double time_old;
	bool time_init = false;
	pcl::PointCloud<PointXYZIRT>::Ptr cloud_ptr(new pcl::PointCloud<PointXYZIRT>);
	int count = 0;
	while (!f.eof()) 
	{
		std::cout << "count: " << count++ << std::endl;
		//skip first line
		if (count == 1)
		{
			std::string s;
			std::getline(f, s);
			continue;
		}
		std::string s;
		std::getline(f, s);
		std::vector<std::string> words;
		if (!s.empty())
		{		
			std::stringstream ss;
			ss << s;
			std::string token;
			// read row data
			while (std::getline(ss, token, ','))
			{
				words.push_back(token);
				// std::cout << token << std::endl;
			}
			time = std::stod(words[0]);
			if (!time_init)
			{
				time_old = time;
				time_init = true;
			}
			//keep two decimal places
			double time_2 = floor(time * 100) / 100;
			double time_old_2 = floor(time_old * 100) / 100;
			if (time_2 == time_old_2)
			{
				PointXYZIRT Point;
				Point.x = std::stof(words[1]);
				Point.y = std::stof(words[2]);
				Point.z = std::stof(words[3]);
				Point.intensity = std::stof(words[4]);
				Point.ring = 0;
				Point.time = time - time_old;//relative time to the first point in the same scan
				cloud_ptr->push_back(Point);
			}
			else
			{
				pcl::io::savePCDFileBinary(save_dir + "/"+std::to_string(time_old_2)+".pcd", *cloud_ptr);
				//the time of the first point is 0.0 s and the time of the last point is 0.0099 s
				time_old = time;
				//new scan
				cloud_ptr->clear();
				PointXYZIRT Point;
				Point.x = std::stof(words[1]);
				Point.y = std::stof(words[2]);
				Point.z = std::stof(words[3]);
				Point.intensity = std::stof(words[4]);
				Point.ring = 0;
				Point.time = time - time_old;//relative time to the first point in the same scan
				cloud_ptr->push_back(Point);
			}
			
		}
	}
	f.close();

	return 0;
}
