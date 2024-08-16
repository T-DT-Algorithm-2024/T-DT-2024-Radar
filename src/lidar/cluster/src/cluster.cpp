#include "cluster.h"
#include <pcl/PCLPointCloud2.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <rclcpp/duration.hpp>
#include <pcl/segmentation/extract_clusters.h>
namespace tdt_radar{

    Cluster::Cluster(const rclcpp::NodeOptions& node_options): Node("cluster_node", node_options)
    {
        RCLCPP_INFO(this->get_logger(), "cluster_node start");
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar_dynamic", 10, std::bind(&Cluster::callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar_cluster", 10);
    }


void Cluster::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::fromROSMsg(*msg, *cloud);
    if (cloud->empty()) {return;}
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    auto time = std::chrono::system_clock::now();

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.25);
    ec.setMinClusterSize (5);
    ec.setMaxClusterSize (1000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract (cluster_indices);
    // std::cout<<(std::chrono::system_clock::now()-time).count()<<"ms"<<std::endl;
    
    pcl::PointCloud<pcl::PointXYZ> *out_cloud(new pcl::PointCloud<pcl::PointXYZ>); 
    for(auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for(auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            cloud_cluster->points.push_back(cloud->points[*pit]);
        }        
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        pcl::PointXYZ move_point;
        for(auto point:cloud_cluster->points)
        {
            move_point.x += point.x;
            move_point.y += point.y;
            move_point.z += point.z;
        }
        move_point.x /= cloud_cluster->points.size();
        move_point.y /= cloud_cluster->points.size();
        move_point.z /= cloud_cluster->points.size();
        out_cloud->points.push_back(move_point);        
    }
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*out_cloud, output);
    output.header.frame_id = "rm_frame";
    output.header.stamp = msg->header.stamp;
    pub_->publish(output);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(), "Cluster callback time: %f", std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()/1000.0);
}
}//namespace tdt_radar

RCLCPP_COMPONENTS_REGISTER_NODE(tdt_radar::Cluster)