
#include "jpc_groundremove.h"

int main(int argc, char** argv){

	pcl::PointCloud<PointXYZIR>::Ptr cloud(new pcl::PointCloud<PointXYZIR>);
    	pcl::io::loadPCDFile<PointXYZIR>(argv[1], *cloud);
    	cloud->height = 1;
    	cloud->width = cloud->points.size();

	cout<< cloud->points.size()<<endl;
    	cloud->is_dense = false;

	pcl::PointCloud<PointXYZIR>::Ptr cloud_gr(new pcl::PointCloud<PointXYZIR>);
	pcl::PointCloud<PointXYZIR>::Ptr cloud_ob(new pcl::PointCloud<PointXYZIR>);

	JpcGroundRemove groundremove;
	groundremove.GroundRemove(*cloud, *cloud_gr, *cloud_ob);


	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
			new pcl::visualization::PCLVisualizer("pcd")); //PCLVisualizer 可视化类
	viewer->setBackgroundColor(0.1, 0.1, 0.1);

	pcl::visualization::PointCloudColorHandlerCustom <PointXYZIR> color_gr(cloud_gr, (0), (255), (0));
	string s_gr = "cloud_gr";
	viewer->addPointCloud(cloud_gr, color_gr, s_gr);

	pcl::visualization::PointCloudColorHandlerCustom <PointXYZIR> color_ob(cloud_ob, (255), (0), (0));
	string s_ob = "cloud_ob";
	viewer->addPointCloud(cloud_ob, color_ob, s_ob);

	while (!viewer->wasStopped()) {
		viewer->spin();
	}

	return 0;
}
