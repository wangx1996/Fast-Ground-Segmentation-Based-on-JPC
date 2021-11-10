#ifndef JPC_GROUNDREMOVE_H
#define JPC_GROUNDREMOVE_H

#include <iostream>
#include <fstream>
#include <vector>
#include <ctime>
#include <stdlib.h>
#include <time.h> 
#include <algorithm>
#include <queue>
#include <unordered_set>
#include <math.h>
#include <unordered_map>
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
#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp>//自定义点云类型时要加

//Eigen
#include <Eigen/Dense>
#include <queue> 

#include <unordered_map>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
using namespace std;

struct PointXYZIR{
PCL_ADD_POINT4D;
float intensity;
uint16_t ring;
EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
(float,x,x)
(float,y,y)
(float,z,z)
(float,intensity,intensity)
(uint16_t,ring,ring)
)

struct Index{
	int x = 0;
	int y = 0;
};


class JpcGroundRemove{

public:
	JpcGroundRemove();
	~JpcGroundRemove(){}

	void RangeProjection();
	void RECM();
	cv::Mat JCP();
	void GroundRemove(pcl::PointCloud<PointXYZIR>& cloud_IN, 
				pcl::PointCloud<PointXYZIR>& cloud_gr, 
				pcl::PointCloud<PointXYZIR>& cloud_ob,
				cv::Mat& range_image);
	void calAngle(float x, float y, float &temp_tangle);
	void calRange(const PointXYZIR& p, float& range);
	void calRangeDiff(const PointXYZIR& p1, const PointXYZIR& p2, float& range);

private:

	pcl::PointCloud<PointXYZIR>::Ptr cloud_;
	float sensor_height = -1.73;

	int width_  = 2083;
	int height_ = 64;
	float max_range_ = 70.0;
	float min_range = 2.0;
	int length_ = 0;

	int neighborx_[24] = {-2, -1,  0,  1,  2,-2, -1,  0,  1,  2,-2,-1, 1, 2, -2,-1,0, 1, 2,-2,-1, 0, 1, 2};
	int neighbory_[24] = {-2, -2, -2, -2, -2,-1, -1, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2};

	vector<float> region_minz_;
	//vector<Index> cloud_index_;
	vector<int> cloud_index_;
	cv::Mat range_image_;
	cv::Mat region_;
	

	float th_g_ = 0.2;
	float sigma_ = 10.;
	float deltaR_ = 2.;
	float th_d_ = 1;
	
};

#endif


