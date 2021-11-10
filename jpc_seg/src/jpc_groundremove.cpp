#include "jpc_groundremove.h"

JpcGroundRemove::JpcGroundRemove(){
	pcl::PointCloud<PointXYZIR>::Ptr tempc(new pcl::PointCloud<PointXYZIR>);
	cloud_ = tempc;
	length_ = int((max_range_ - min_range)/deltaR_);
	region_minz_.assign(width_ * length_, 100);
	cloud_index_.assign(width_ * height_, -1);
	range_image_ = cv::Mat::zeros(height_, width_,  CV_8UC3);
	region_ = cv::Mat::zeros(height_, width_,  CV_8UC1);
}


void JpcGroundRemove::calAngle(float x, float y, float &temp_tangle){
	if (x == 0 && y == 0) {
		temp_tangle = 0;
	} else if (y >= 0) {
		temp_tangle = (float) atan2(y, x);
	} else if (y <= 0) {
		temp_tangle = (float) atan2(y, x) + 2 * M_PI;
	}
}


void JpcGroundRemove::calRange(const PointXYZIR& p, float& range){
	range = sqrt(p.x*p.x + p.y*p.y);
}

void JpcGroundRemove::calRangeDiff(const PointXYZIR& p1, const PointXYZIR& p2, float& range){
	range = sqrt((p1.x- p2.x)*(p1.x- p2.x)+(p1.y- p2.y)*(p1.y- p2.y) +(p1.z- p2.z)*(p1.z- p2.z));
}

void JpcGroundRemove::RangeProjection(){
	//cout<<cloud_->points.size()<<endl;
	for(int i=0; i<cloud_->points.size(); ++i){
		float u(0), range(0);

		calAngle(cloud_->points[i].x, cloud_->points[i].y, u);
		calRange(cloud_->points[i], range);

		int col = round((width_-1)*(u *180.0/M_PI)/360.0);
		int ind = cloud_->points[i].ring;
		if(range<2 || range>70 || col<0 || col > width_ || ind <0 || ind > height_ ||
				((cloud_->points[i].x < 3 && cloud_->points[i].x > -2)
				&&(cloud_->points[i].y < 1.5 && cloud_->points[i].y > -1.5))
				|| (cloud_->points[i].z < -3 && cloud_->points[i].z > 1))
			continue;

		int region = int((range-min_range)/deltaR_);

		int region_index = col * length_ + region;
		int index = col * height_ + ind;
		range_image_.at<cv::Vec3b>(ind, col) = cv::Vec3b(0,255,0);
		region_minz_[region_index] = min(region_minz_[region_index], cloud_->points[i].z);
		region_.at<uchar>(ind, col) = region;
		cloud_index_[index] = i;
	}
	// cv::Mat show_image;
	// cv::flip(range_image_,show_image, 0);
	// cv::imshow("TEst", show_image);
	// cv::waitKey(0);
}


void JpcGroundRemove::RECM(){

	bool flag = false;
	for(int i=0; i<region_minz_.size(); ++i){
		if( i%length_ == 0){
			flag = false;
			region_minz_[i] = min(region_minz_[i], sensor_height + th_g_);
			continue;
		}else{
			if((i+1)%length_ == 0)
				continue;	
			if(region_minz_[i] == 100 && !flag){
				region_minz_[i] = sensor_height + th_g_;
				continue;
			}
			if(region_minz_[i] == 100 && flag){
				region_minz_[i] = region_minz_[i-1];
			}

			flag = true;
			if(fabs(region_minz_[i] - region_minz_[i-1])>0.5 && fabs(region_minz_[i] - region_minz_[i + 1])>0.5)
				region_minz_[i] = (region_minz_[i-1] +region_minz_[i+1])/2;
		}
	}

	float pre_th = 0.;
	float region_num = 0;
	for(int i=0; i<region_minz_.size(); ++i){
		if( i%length_ == 0){
			pre_th = min(region_minz_[i], float(sensor_height));
		}else{
			region_minz_[i] = min(region_minz_[i], pre_th + deltaR_ * (float)tan(sigma_*M_PI/180));
			pre_th = region_minz_[i] ;
		}
	}		

	for(int i=0; i<width_; ++i){
		for(int j= 0; j<height_; ++j){
			
			int index = i * height_ +j;
			int region_i = region_.at<uchar>(j, i);
			float th_height = region_minz_[i*length_ + region_i];
			int id = cloud_index_[index];
			if(id == -1)
				continue;
			if(cloud_->points[id].z >= (th_height+th_g_)){
				range_image_.at<cv::Vec3b>(j, i) = cv::Vec3b(0,0,255);
			}

		}
	}
	//cv::Mat show_image;
	//cv::flip(range_image_,show_image, 0);
	//cv::imshow("RECM image", show_image);
	//cv::waitKey(1);
}


cv::Mat JpcGroundRemove::JCP(){
	vector<cv::Mat> channels;
	cv::split(range_image_, channels);
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
	cv::dilate(channels[2],channels[2],element);

	cv::merge(channels, range_image_);

	queue<Index> qt;
	for(int i=0; i<width_; ++i){
		for(int j= 0; j<height_; ++j){
			if(range_image_.at<cv::Vec3b>(j, i) == cv::Vec3b(0,255,255)){
				Index id;
				id.x = i;
				id.y = j;
				if(cloud_index_[j * height_ + i] != -1){
					qt.push(id);
					range_image_.at<cv::Vec3b>(j, i) = cv::Vec3b(255,0,0);
				}else{
					range_image_.at<cv::Vec3b>(j, i) = cv::Vec3b(0,0,255);
				}
			}
		}
	}
	cv::Mat show_image;
	cv::flip(range_image_,show_image, 0);
	cv::imshow("Dilate image", show_image);
	cv::waitKey(1);

	while(!qt.empty()){
		Index id = qt.front();
		qt.pop();
		int cloud_id = id.x * height_ + id.y;
		Eigen::VectorXf D(24);
		int mask[24];
		float sumD(0);
		for(int i=0; i<24; ++i){
			int nx = neighborx_[i] + id.x;
			int ny = neighbory_[i] + id.y;

			int ncloud_id = nx * height_ + ny;
			float range_diff(0);
			
			if(nx < 0 || nx >= width_ || ny < 0 ||ny >= height_ || cloud_index_[ncloud_id] == -1){
				D(i) = 0;
				sumD += D(i) ;
				mask[i] = -1;		
				continue;

			}

			calRangeDiff(cloud_->points[cloud_index_[cloud_id]], cloud_->points[cloud_index_[ncloud_id]], range_diff);
			if(range_diff > 3){
				D(i) = 0;
				sumD += D(i);
			}else{
				D(i) = (exp(-5 * range_diff));
				sumD += D(i);
			}
			if(range_image_.at<cv::Vec3b>(ny, nx) == cv::Vec3b(255,0,0)){
				mask[i] = 2;
			}else if(range_image_.at<cv::Vec3b>(ny, nx) == cv::Vec3b(0,255,0)){
				mask[i] = 1;
			}else if(range_image_.at<cv::Vec3b>(ny, nx) == cv::Vec3b(0,0,255)){
				mask[i] = 0;
			}
		}


		Eigen::VectorXf W(24);
		W = D / sumD;

		float score_r(0), score_g(0);
		for(int i=0; i<D.size(); ++i){
			if(mask[i] == 0){
				score_r += W(i);
			}else if(mask[i] == 1){
				score_g += W(i);
			}
		}
		
		if(score_r > score_g){
			range_image_.at<cv::Vec3b>(id.y, id.x) = cv::Vec3b(0,0,255);
		}else{
			range_image_.at<cv::Vec3b>(id.y, id.x) = cv::Vec3b(0,255,0);
		}

	}
					

	//cv::flip(range_image_,show_image, 0);
	//cv::imshow("Final image", show_image);
	//cv::waitKey(1);
	return show_image;
}



void JpcGroundRemove::GroundRemove(pcl::PointCloud<PointXYZIR>& cloud_IN,
					pcl::PointCloud<PointXYZIR>& cloud_gr, 
					pcl::PointCloud<PointXYZIR>& cloud_ob,
					cv::Mat& range_image){

	for(auto p:cloud_IN.points){
		cloud_->points.push_back(p);
	}
	printf("[INFO] Start segmentation!\n");

	RangeProjection();
	RECM();
	range_image = JCP();
	for(int i=0; i<width_; ++i){
		for(int j=0; j<height_; ++j){
			int index = cloud_index_[i*height_ + j];
			if(index != -1){
				if(range_image_.at<cv::Vec3b>(j, i) == cv::Vec3b(0,255,0)){	
					cloud_gr.push_back(cloud_->points[index]);
				}else if(range_image_.at<cv::Vec3b>(j, i) == cv::Vec3b(0,0,255)){	
					cloud_ob.push_back(cloud_->points[index]);
				}
			}
		}
	}
}	

