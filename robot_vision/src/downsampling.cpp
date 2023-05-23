/*
* ROS node for downsampling point cloud data.
*
* Author: Anubhav Singh
*/

#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <sensor_msgs/PointCloud2.h>

class Preprocess
{
    public:
        explicit Preprocess(ros::NodeHandle nh)
            : nh_(nh)
        {
            pcl_sub = nh_.subscribe("/camera/depth_registered/points", 1, &Preprocess::pclCallback, this);
            pcl_downsampled_pub = nh_.advertise<sensor_msgs::PointCloud2>("/pcl_downsampled", 1);
        }

    private:
        ros::NodeHandle nh_;
        ros::Subscriber pcl_sub;
        ros::Publisher pcl_downsampled_pub;

        void pclCallback(const sensor_msgs::PointCloud2& cloud_msg)
        {
            ROS_INFO("Point Cloud Cluster Received.");

            pcl::PointCloud<pcl::PointXYZRGB> *p_cloud = new pcl::PointCloud<pcl::PointXYZRGB>();
            pcl::PointCloud<pcl::PointXYZRGB> *downsampled = new pcl::PointCloud<pcl::PointXYZRGB>();

            const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > sp_pcl_cloud(p_cloud);
            pcl::fromROSMsg(cloud_msg, *p_cloud);

            const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > sp_pcl_downsampled_cloud(downsampled);

            ROS_INFO("---------------------------------");
            ROS_INFO("Point Cloud Cluster Stats:");
            ROS_INFO("Height: %d", sp_pcl_cloud->height);
            ROS_INFO("Weight: %d", sp_pcl_cloud->width);
            ROS_INFO("Total data points : %d", sp_pcl_cloud->height*sp_pcl_cloud->width);
            // ROS_INFO("",*p_cloud);
            ROS_INFO("---------------------------------");
            
            ROS_INFO("Starting Preprocessing steps...");

            ROS_INFO("Starting Voxel Grid Downsampling...");
            pcl::VoxelGrid<pcl::PointXYZRGB> vox;
            vox.setInputCloud(sp_pcl_cloud);
            vox.setLeafSize(0.01f, 0.01f, 0.01f);
            vox.filter(*downsampled);
            // ROS_INFO("Total data points after downsampling: %d", sp_pcl_cloud->height*sp_pcl_cloud->width);


            sensor_msgs::PointCloud2 pcl_downsampled;
            pcl::toROSMsg(*downsampled, pcl_downsampled);
            pcl_downsampled_pub.publish(pcl_downsampled);
        }



};  // Preprocess

int main(int argc, char **argv)
{
    ros::init(argc, argv, "downsampling");
    ros::NodeHandle nh;

    Preprocess preprocess(nh);

    // Spin until ROS is shutdown
    while (ros::ok())
        ros::spin();

    return 0;
}
