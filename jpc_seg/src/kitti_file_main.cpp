#include <iostream>
#include <fstream>
#include <vector>
#include <ctime>
#include <stdlib.h>
#include <time.h> 
#include <algorithm>
#include <unordered_set>
#include <math.h>

//PCL
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp>//自定义点云类型时要加
#include <unordered_map>

//Eigen
#include <Eigen/Dense>
#include <queue> 

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>



//Boost
#include <boost/tokenizer.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>



//ROS
#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h> 
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

#include "jpc_groundremove.h"

using namespace std;
typedef boost::tokenizer<boost::char_separator<char> > tokenizer;


template <class Type>  
Type stringToNum(const string& str)  
{  
    istringstream iss(str);  
    Type num;  
    iss >> num;  
    return num;      
}  

template<typename T> string toString(const T& t) {
	ostringstream oss;
	oss << t;
	return oss.str();
}



int fileNameFilter(const struct dirent *cur) {
	std::string str(cur->d_name);
	if (str.find(".bin") != std::string::npos||str.find(".velodata") != std::string::npos
			|| str.find(".pcd") != std::string::npos
			|| str.find(".png") != std::string::npos
			|| str.find(".jpg") != std::string::npos
			|| str.find(".txt") != std::string::npos) {
		return 1;
	}
	return 0;
}


bool get_all_files(const std::string& dir_in,
		std::vector<std::string>& files) {

	if (dir_in.empty()) {
		return false;
	}
	struct stat s;
	stat(dir_in.c_str(), &s);
	if (!S_ISDIR(s.st_mode)) {
		return false;
	}
	DIR* open_dir = opendir(dir_in.c_str());
	if (NULL == open_dir) {
		std::exit(EXIT_FAILURE);
	}
	dirent* p = nullptr;
	while ((p = readdir(open_dir)) != nullptr) {
		struct stat st;
		if (p->d_name[0] != '.') {
			//因为是使用devC++ 获取windows下的文件，所以使用了 "\" ,linux下要换成"/"
			//cout<<std::string(p->d_name)<<endl;
			std::string name = dir_in + std::string("/")
					+ std::string(p->d_name);
			stat(name.c_str(), &st);
			if (S_ISDIR(st.st_mode)) {
				get_all_files(name, files);
			} else if (S_ISREG(st.st_mode)) {
				boost::char_separator<char> sepp { "." };
				tokenizer tokn(std::string(p->d_name), sepp);
				vector<string> filename_sep(tokn.begin(), tokn.end());
				string type_ = "." + filename_sep[1];
				break;
			}
		}
	}

	struct dirent **namelist;
	int n = scandir(dir_in.c_str(), &namelist, fileNameFilter, alphasort);
	if (n < 0) {
		return false;
	}
	for (int i = 0; i < n; ++i) {
		std::string filePath(namelist[i]->d_name);
		files.push_back(filePath);
		free(namelist[i]);
	};
	free(namelist);
	closedir(open_dir);
	return true;
}

bool Load_Sensor_Data_Path(std::vector<std::string>& lidarfile_name,  string& path){
	string lidar_file_path = path;
	cout<<lidar_file_path<<endl;
	if(!get_all_files(lidar_file_path, lidarfile_name))
		return false;

	return true;
}

int get_quadrant(pcl::PointXYZI point)
{
	int res = 0;
	float x = point.x;
	float y = point.y;
	if (x > 0 && y >= 0)
		res = 1;
	else if (x <= 0 && y > 0)
		res = 2;
	else if (x < 0 && y <= 0)
		res = 3;
	else if (x >= 0 && y < 0)
		res = 4;
	return res;
}



void add_ring_info(pcl::PointCloud<pcl::PointXYZI>& input, pcl::PointCloud<PointXYZIR>& output)
{
	int previous_quadrant = 0;
	uint16_t ring_ = (uint16_t)64-1;
	for (pcl::PointCloud<pcl::PointXYZI>::iterator pt = input.points.begin(); pt < input.points.end()-1; ++pt){
		int quadrant = get_quadrant(*pt);
		if (quadrant == 1 && previous_quadrant == 4 && ring_ > 0)
			ring_ -= 1;
		PointXYZIR point;
		point.x = pt->x;
		point.y = pt->y;
		point.z = pt->z;
		point.intensity = pt->intensity;
		point.ring = ring_;
		output.push_back(point);
		previous_quadrant = quadrant;
	}
}



bool LoadKittiPointcloud(pcl::PointCloud<pcl::PointXYZI>& cloud_IN, string path){
	string lidar_filename_path = path;
	ifstream inputfile;
	inputfile.open(lidar_filename_path, ios::binary);
	if (!inputfile) {
		cerr << "ERROR: Cannot open file " << lidar_filename_path
					<< "! Aborting..." << endl;
		return false;
	}

	inputfile.seekg(0, ios::beg);
	for (int i = 0; inputfile.good() && !inputfile.eof(); i++) {
		pcl::PointXYZI p;
		inputfile.read((char *) &p.x, 3 * sizeof(float));
		inputfile.read((char *) &p.intensity, sizeof(float));
		cloud_IN.points.push_back(p);
	}
	return true;
}


int main(int argc, char** argv){

 	ros::init(argc, argv, "lidar_read_node");

  	ros::NodeHandle nh;
	string datapath = "/media/wx/File/2011_09_30_drive_0027_sync/2011_09_30/2011_09_30_drive_0027_sync/velodyne_points/data";

 	ros::Publisher publidar = nh.advertise <sensor_msgs::PointCloud2> ("/pointcloud", 1);
 	ros::Publisher pubgrlidar = nh.advertise <sensor_msgs::PointCloud2> ("/h_ground_point", 1);
 	ros::Publisher puboblidar = nh.advertise <sensor_msgs::PointCloud2> ("/h_obstacle_point", 1);
  	image_transport::ImageTransport it(nh);
  	image_transport::Publisher pubimage = it.advertise("/seg_image", 1);

	vector<string> lidarname;
	if(!Load_Sensor_Data_Path(lidarname, datapath)){
		cout<<"Detecion file wrong!"<<endl;
		std::abort();
	}

	cout<<lidarname.size()<<endl;
	int maxframe = lidarname.size();
	vector<int> frame_num;
	boost::char_separator<char> sep { " " };
	/*if(!lidartype){
		for(int i=0; i<lidarname.size(); ++i){
			tokenizer tok_line(lidarname[i], sep);
			std::vector<std::string> lines(tok_line.begin(), tok_line.end());
			frame_num.push_back(stringToNum<int>(lines[0]));
		}
	}*/

	sort(frame_num.begin(), frame_num.end(), [](int a, int b){ return a<b; });


	int frame = 0;
	ros::Rate r(10);
	while(ros::ok() && frame < maxframe){
		string cloudpath = datapath+ "/" +lidarname[frame];
		sensor_msgs::PointCloud2 ros_cloud;
		ros_cloud.fields.resize(5);
		int tmpOffset = 0 ;
	

		pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud(
				new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<PointXYZIR>::Ptr cloud_ring(
				new pcl::PointCloud<PointXYZIR>); 	
		LoadKittiPointcloud(*Cloud, cloudpath);
		add_ring_info(*Cloud, *cloud_ring);
		pcl::toROSMsg(*cloud_ring, ros_cloud);
		ros_cloud.header.frame_id = "global_init_frame";
		publidar.publish(ros_cloud);

		pcl::PointCloud<PointXYZIR>::Ptr cloud_b(
			new pcl::PointCloud<PointXYZIR>);
		pcl::PointCloud<PointXYZIR>::Ptr cloud_g(
			new pcl::PointCloud<PointXYZIR>);
		cv::Mat range_image;

		JpcGroundRemove groundremove;
		groundremove.GroundRemove(*cloud_ring, *cloud_g, *cloud_b, range_image);

		sensor_msgs::ImagePtr imgmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",range_image).toImageMsg(); //转换为ros接受的消息
		pubimage.publish(imgmsg);

		sensor_msgs::PointCloud2 ros_cloud2;
		pcl::toROSMsg(*cloud_b, ros_cloud2);
		ros_cloud2.header.frame_id = "global_init_frame";
		puboblidar.publish(ros_cloud2);

		sensor_msgs::PointCloud2 ros_cloud3;
		pcl::toROSMsg(*cloud_g, ros_cloud3);
		ros_cloud3.header.frame_id = "global_init_frame";
		pubgrlidar.publish(ros_cloud3);
		frame++;
	}

return 0;
}
