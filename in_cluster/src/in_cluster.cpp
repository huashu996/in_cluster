#include <ros/ros.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PolygonStamped.h"
#include <pcl/segmentation/extract_clusters.h>

using namespace std;

class InCluster
{
//**********************************************************************************************************
  //1、定义成员变量
  private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_ptr;
    typedef pcl::PointXYZI PointT;
		typedef pcl::PointCloud<PointT> PointCloudT;
		typedef std::vector<PointCloudT::Ptr> PointCloudVectorT;
    ros::NodeHandle nh;
    ros::Publisher cloud_showpub;
    ros::Subscriber sub_lidar_points;
    ros::Publisher boxpub;
    visualization_msgs::Marker box;
    
	
  public:   
//构造函数
    InCluster():
    nh("~"){
		std::string lidar_points_topic;
		ROS_ASSERT(private_nh.getParam("lidar_points_topic", lidar_points_topic));
		
		sub_lidar_points = nh.subscribe("/livox/lidar", 100, &InCluster::callback, this);
		cloud_showpub = nh.advertise<sensor_msgs::PointCloud2>("in_cloud", 1);
		boxpub = nh.advertise<visualization_msgs::MarkerArray>("box", 1);
        allocateMemory(); //初始化
    }

    void allocateMemory()
    {
        input_cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
        in_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        
    }
    void resetParameters(){ 
        in_cloud->clear();
    }
    
    void callback(const sensor_msgs::PointCloud2ConstPtr input_cloud_msg)
    {
        resetParameters();
        std_msgs::Header cloud_header = input_cloud_msg->header;
        //获取点云
        pcl::fromROSMsg(*input_cloud_msg, *input_cloud_ptr);//把input_cloud_msg放
        cout<<"receiving pointcloud"<<endl;
        for(int i=0;i<input_cloud_ptr->points.size();i++)
		{
		      
		    if (input_cloud_ptr->points[i].intensity==255)
		    {
		    	pcl::PointXYZI point;   
		    	point.x = input_cloud_ptr->points[i].x;                                                        
				point.y = input_cloud_ptr->points[i].y;                                                        
				point.z = input_cloud_ptr->points[i].z;
				point.intensity = input_cloud_ptr->points[i].intensity;
				in_cloud->points.push_back (point);
		    }  
		    else 
		    {
		    	continue;
		    }                                     

        }
        publishCloudtoShow(cloud_showpub, cloud_header, in_cloud);
        cluster(cloud_header,in_cloud);
        
        
    }
	void cluster(const std_msgs::Header& header, const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud)
    {
		visualization_msgs::MarkerArray box_array;
	  // 创建分割对象
	  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
	  ec.setClusterTolerance (0.05); // 设置聚类的距离阈值
	  ec.setMinClusterSize (10); // 设置聚类的最小点数
	  ec.setMaxClusterSize (1000); // 设置聚类的最大点数
	  
	  // 设置要分割的点云数据
	  ec.setInputCloud (cloud);
	  
	  // 执行聚类操作，并输出结果
	  std::vector<pcl::PointIndices> cluster_indices;
	  ec.extract (cluster_indices);
	  
	  int j = 0;
	  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	  {
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
		cloud_cluster->push_back ((*cloud)[*pit]); // 把每个聚类中的点存入对应的点云中
		cloud_cluster->width = cloud_cluster->size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		
		// 输出每个聚类的信息
		std::cout << "Cluster " << j << " has " << cloud_cluster->size () << " points:" << std::endl;
		std::cout << "    Centroid: (" << cloud_cluster->points[0].x << ", " << cloud_cluster->points[0].y << ", " << cloud_cluster->points[0].z << ")" << std::endl;
		std::cout << "    Min point: (" << cloud_cluster->points[0].x << ", " << cloud_cluster->points[0].y << ", " << cloud_cluster->points[0].z << ")" << std::endl;
		std::cout << "    Max point: (" << cloud_cluster->points[cloud_cluster->size()-1].x << ", " << cloud_cluster->points[cloud_cluster->size()-1].y << ", " << cloud_cluster->points[cloud_cluster->size()-1].z << ")" << std::endl;
		j++;
	  }
	  boxpub.publish(box_array);
	}

	//发布聚类点云


    
    void publishCloudtoShow(const ros::Publisher& cloud_showpub, const std_msgs::Header& header, const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud)
    {
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud, output_msg);
        output_msg.header = header;
        cloud_showpub.publish(output_msg);
        cout<<"sucess pub"<<endl;
        
    }
};


int main(int argc, char** argv)
{
  //1、节点初始化 及定义参数
    ros::init(argc, argv, "in_cluster");
    InCluster IC;
    ros::spin();
    return 0;
}



