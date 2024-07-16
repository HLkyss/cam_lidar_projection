#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

// Function to color the point cloud based on depth
void colorPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud)
{
    for (const auto& point : input_cloud->points)
    {
        pcl::PointXYZRGB point_rgb;
        point_rgb.x = point.x;
        point_rgb.y = point.y;
        point_rgb.z = point.z;

        if (point.y < 0)//经过测试发现,-y是前向,z是纵向
        {
            // Positive depth (in front of the camera): Red
            point_rgb.r = 255;
            point_rgb.g = 0;
            point_rgb.b = 0;
        }
        else
        {
            // Negative depth (behind the camera): Blue
            point_rgb.r = 0;
            point_rgb.g = 0;
            point_rgb.b = 255;
        }

        output_cloud->points.push_back(point_rgb);
    }

    output_cloud->width = output_cloud->points.size();
    output_cloud->height = 1;
    output_cloud->is_dense = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_publisher");
    ros::NodeHandle nh;

    // Define publisher for the colored point cloud
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("colored_cloud", 1);

    // Load point cloud from file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/media/hl/One_Touch/ubuntu_share/Dataset/MYNTEYE_D1000/cam_lidar/cam_lidar/pointclouds/1720014586960077006.pcd", *cloud) == -1)
    {
        ROS_ERROR("Couldn't read file %s", argv[1]);
        return -1;
    }

    // Create a new point cloud for the colored points
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Color the point cloud based on depth
    colorPointCloud(cloud, colored_cloud);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        // Convert to ROS message and publish
        sensor_msgs::PointCloud2 ros_cloud;
        pcl::toROSMsg(*colored_cloud, ros_cloud);
        ros_cloud.header.frame_id = "map";
        cloud_pub.publish(ros_cloud);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
