#include <iostream>
#include <fstream>
#include <vector>
#include <ctime>
#include <stdlib.h>
#include <time.h> 
#include <algorithm>
#include <deque>
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
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h> 

#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp>//自定义点云类型时要加

//Eigen
#include <Eigen/Dense>
#include <queue> 

#include <unordered_map>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

//ROS
#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h> 
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>


using namespace std;

#include "jpc_groundremove.h"

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


class SubscribeAndPublish {
public:
	SubscribeAndPublish(ros::NodeHandle nh, std::string lidar_topic_name);


	~SubscribeAndPublish(){
	}

	void callback(const sensor_msgs::PointCloud2ConstPtr& cloudmsg) {

		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
			new pcl::PointCloud<pcl::PointXYZI>);

   		pcl::fromROSMsg(*cloudmsg, *cloud);

		pcl::PointCloud<PointXYZIR>::Ptr cloud_ring(new pcl::PointCloud<PointXYZIR>);
		add_ring_info(*cloud, *cloud_ring);

		ROS_INFO("Reciving data!");

		pcl::PointCloud<PointXYZIR>::Ptr cloud_gr(new pcl::PointCloud<PointXYZIR>);
		pcl::PointCloud<PointXYZIR>::Ptr cloud_ob(new pcl::PointCloud<PointXYZIR>);
		cv::Mat range_image;

		JpcGroundRemove groundremove;
		groundremove.GroundRemove(*cloud_ring, *cloud_gr, *cloud_ob, range_image);
		ROS_INFO("segmentation");

		sensor_msgs::ImagePtr imgmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",range_image).toImageMsg(); 
		pubimage_.publish(imgmsg);

		sensor_msgs::PointCloud2 ros_cloud2;
		pcl::toROSMsg(*cloud_gr, ros_cloud2);
		ros_cloud2.header.frame_id = "global_init_frame";
		pub_.publish(ros_cloud2);

		sensor_msgs::PointCloud2 ros_cloud3;
		pcl::toROSMsg(*cloud_ob, ros_cloud3);
		ros_cloud3.header.frame_id = "global_init_frame";
		pub2_.publish(ros_cloud3);

	}
private:
	ros::NodeHandle n_;
	ros::Publisher pub_;
	ros::Publisher pub2_;
	ros::Subscriber sub_;
  	image_transport::Publisher pubimage_; 
};

SubscribeAndPublish::SubscribeAndPublish(ros::NodeHandle nh,
		std::string lidar_topic_name) :
		n_(nh){

	pub_  = nh.advertise < sensor_msgs::PointCloud2 > ("/ground_point", 1);
	pub2_ = nh.advertise < sensor_msgs::PointCloud2 > ("/obstacle_point", 1);
	sub_ = nh.subscribe(lidar_topic_name, 10, &SubscribeAndPublish::callback, this);
  	image_transport::ImageTransport it(nh);
	pubimage_ = it.advertise("/seg_image", 1);
}



int main(int argc, char** argv){

 	ros::init(argc, argv, "jpc_seg_node");
	SubscribeAndPublish SAPObject(ros::NodeHandle(), "pointcloud");
	ROS_INFO("waiting for data!");
	ros::spin();
	return 0;
}
