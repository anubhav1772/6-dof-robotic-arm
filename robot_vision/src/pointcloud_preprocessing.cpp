/*
* ROS node for point cloud preprocessing and object segmentation.
*
* Author: Anubhav Singh
* How to install Point Cloud Library v1.8 (pcl-1.8.0) on Ubuntu 16.04.2 [LTS] for C++
* * https://askubuntu.com/questions/916260/how-to-install-point-cloud-library-v1-8-pcl-1-8-0-on-ubuntu-16-04-2-lts-for
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
            // pcl_sub = nh.subscribe("/transformed_point_cloud", 1, &Preprocess::pclCallback, this);
            pcl_downsampled_pub = nh_.advertise<sensor_msgs::PointCloud2>("/pcl_downsampled", 1);
            pcl_inliers_pub = nh_.advertise<sensor_msgs::PointCloud2>("/pcl_inliers", 1);
            pcl_outliers_pub = nh_.advertise<sensor_msgs::PointCloud2>("/pcl_outliers", 1);
            pcl_passthroughx_pub = nh_.advertise<sensor_msgs::PointCloud2>("/pcl_passthroughx", 1);
            pcl_passthroughy_pub = nh_.advertise<sensor_msgs::PointCloud2>("/pcl_passthroughy", 1);
            pcl_passthroughz_pub = nh_.advertise<sensor_msgs::PointCloud2>("/pcl_passthroughz", 1);

            pcl_cloud_table_pub = nh_.advertise<sensor_msgs::PointCloud2>("/pcl_table", 1);
            pcl_cloud_objects_pub = nh_.advertise<sensor_msgs::PointCloud2>("/pcl_objects", 1);

            // pcl_clusters_pub = nh_.advertise<sensor_stick::SegmentedClustersArray>("/pcl_clusters", 1);
        }

    private:
        ros::NodeHandle nh_;
        ros::Subscriber pcl_sub;
        ros::Publisher pcl_outlers_removed_pub;
        ros::Publisher pcl_downsampled_pub;
        ros::Publisher pcl_inliers_pub;
        ros::Publisher pcl_outliers_pub;
        ros::Publisher pcl_passthroughx_pub;
        ros::Publisher pcl_passthroughy_pub;
        ros::Publisher pcl_passthroughz_pub;

        ros::Publisher pcl_cloud_table_pub;
        ros::Publisher pcl_cloud_objects_pub;

        // ros::Publisher pcl_clusters_pub;

        void pclCallback(const sensor_msgs::PointCloud2& cloud_msg)
        {
            ROS_INFO("Point Cloud Cluster Received.");

            pcl::PointCloud<pcl::PointXYZRGB> *p_cloud = new pcl::PointCloud<pcl::PointXYZRGB>();
            pcl::PointCloud<pcl::PointXYZRGB> *downsampled = new pcl::PointCloud<pcl::PointXYZRGB>();
            pcl::PointCloud<pcl::PointXYZRGB> *inliers_cloud = new pcl::PointCloud<pcl::PointXYZRGB>();
            pcl::PointCloud<pcl::PointXYZRGB> *outliers_cloud = new pcl::PointCloud<pcl::PointXYZRGB>();
            pcl::PointCloud<pcl::PointXYZRGB> *passthrough_x = new pcl::PointCloud<pcl::PointXYZRGB>();
            pcl::PointCloud<pcl::PointXYZRGB> *passthrough_y = new pcl::PointCloud<pcl::PointXYZRGB>();
            pcl::PointCloud<pcl::PointXYZRGB> *passthrough_z = new pcl::PointCloud<pcl::PointXYZRGB>();

            pcl::PointCloud<pcl::PointXYZRGB> *cloud_objects = new pcl::PointCloud<pcl::PointXYZRGB>();
            pcl::PointCloud<pcl::PointXYZRGB> *cloud_table = new pcl::PointCloud<pcl::PointXYZRGB>();

            const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > sp_pcl_cloud(p_cloud);
            pcl::fromROSMsg(cloud_msg, *p_cloud);

            const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > sp_pcl_downsampled_cloud(downsampled);
            const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > sp_pcl_inliers_cloud(inliers_cloud);
            // const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > sp_pcl_outliers_cloud(outliers_cloud);
            const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > sp_pcl_passthroughx_cloud(passthrough_x);
            const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > sp_pcl_passthroughy_cloud(passthrough_y);
            const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > sp_pcl_passthroughz_cloud(passthrough_z);
            const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > sp_pcl_table_cloud(cloud_table);
            const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > sp_pcl_objects_cloud(cloud_objects);

            
            

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

            ROS_INFO("Filtering point cloud data using Statistical Outlier Removal method...");
            pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> outliers_filter;
            outliers_filter.setInputCloud(sp_pcl_downsampled_cloud);
            // The number of neighbors to analyze for each point is set to 5, and 
            // the standard deviation multiplier to 1. This means, all points which
            // have a distance larger than 1 standard deviation of the mean distance 
            // to the query point will be marked as outliers and removed. The output is 
            // computer and stored in inliers. 
            outliers_filter.setMeanK(5);
            outliers_filter.setStddevMulThresh(1.0);
            outliers_filter.filter(*inliers_cloud);

            // outliers_filter is called with the same parameters, but with the output 
            // negated, to obtain the outliers(e.g. the points that were filtered)
            outliers_filter.setNegative(true);
            outliers_filter.filter(*outliers_cloud);

            ROS_INFO("Applying passthrough filtering...");
            pcl::PassThrough<pcl::PointXYZRGB> pass_filter;
            
            // filter along x-axis: left/right 
            pass_filter.setInputCloud(sp_pcl_inliers_cloud);
            pass_filter.setFilterFieldName("x");
            pass_filter.setFilterLimits(-0.8, 5.0);
            pass_filter.setNegative(false);
            pass_filter.filter(*passthrough_x);

            // filter along y-axis: up/down 
            pass_filter.setInputCloud(sp_pcl_passthroughx_cloud);
            pass_filter.setFilterFieldName("y");
            pass_filter.setFilterLimits(-0.5, 0.5);
            pass_filter.setNegative(false);
            pass_filter.filter(*passthrough_y);

           /** ROS_INFO("Applying RANSAC Plane Segmentation...");
            
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            
            pcl::SACSegmentation<pcl::PointXYZRGB> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(100);
            seg.setDistanceThreshold(0.01);

            seg.setInputCloud(sp_pcl_passthroughy_cloud);
            seg.segment (*inliers, *coefficients);

            // Create the filtering object
            pcl::ExtractIndices<pcl::PointXYZRGB> extract;
            extract.setInputCloud(sp_pcl_passthroughy_cloud);
            extract.setIndices(inliers);
            extract.setNegative(false);
            // get the point associated with the planar surface
            // extract inliers - tabletop
            extract.filter(*cloud_table);

            // remove the planar inliers, extract the rest
            extract.setNegative(true);
            extract.filter(*cloud_objects);**/

            // RANSAC algorithm hasn't removed the table edge, as it's not planar with table surface.
            // filter along z-axis: into/outside of the screen
            // removing all points below the table height should get rid of the edge
            
            //pass_filter.setInputCloud(sp_pcl_objects_cloud);
            pass_filter.setInputCloud(sp_pcl_passthroughy_cloud);
            pass_filter.setFilterFieldName("z");
            pass_filter.setFilterLimits(0.0, 2.0);
            //pass_filter.setFilterLimits((sp_pcl_passthroughy_cloud->points[inliers->indices[0]].z + 0.01), (sp_pcl_passthroughy_cloud->points[inliers->indices[0]].z + 1));
            pass_filter.setNegative(false);
            pass_filter.filter(*passthrough_z);

            sensor_msgs::PointCloud2 pcl_downsampled;
            pcl::toROSMsg(*downsampled, pcl_downsampled);
            pcl_downsampled_pub.publish(pcl_downsampled);

            sensor_msgs::PointCloud2 pcl_inliers;
            pcl::toROSMsg(*inliers_cloud, pcl_inliers);
            pcl_inliers_pub.publish(pcl_inliers);

            sensor_msgs::PointCloud2 pcl_outliers;
            pcl::toROSMsg(*outliers_cloud, pcl_outliers);
            pcl_outliers_pub.publish(pcl_outliers);

            sensor_msgs::PointCloud2 pcl_passthroughx;
            pcl::toROSMsg(*passthrough_x, pcl_passthroughx);
            pcl_passthroughx_pub.publish(pcl_passthroughx);

            sensor_msgs::PointCloud2 pcl_passthroughy;
            pcl::toROSMsg(*passthrough_y, pcl_passthroughy);
            pcl_passthroughy_pub.publish(pcl_passthroughy);

            sensor_msgs::PointCloud2 pcl_cloud_objects;
            pcl::toROSMsg(*cloud_objects, pcl_cloud_objects);
            pcl_cloud_objects_pub.publish(pcl_cloud_objects);

            sensor_msgs::PointCloud2 pcl_cloud_table;
            pcl::toROSMsg(*cloud_table, pcl_cloud_table);
            pcl_cloud_table_pub.publish(pcl_cloud_table);

            sensor_msgs::PointCloud2 pcl_passthroughz;
            pcl::toROSMsg(*passthrough_z, pcl_passthroughz);
            pcl_passthroughz_pub.publish(pcl_passthroughz);

            // ROS_INFO("publishing custers...");
            // pcl_clusters_pub.publish(cloudClusters);
            // ROS_INFO("published custers...");

        }



};  // Preprocess

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_cloud_preprocess");
    ros::NodeHandle nh;

    Preprocess preprocess(nh);

    // Spin until ROS is shutdown
    while (ros::ok())
        ros::spin();

    return 0;
}
