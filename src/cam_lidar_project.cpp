#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>

// 自定义颜色映射函数，将深度值映射到颜色
//cv::Vec3b depthToColor(float depth, float max_depth) {
//    float norm_depth = std::min(1.0f, std::max(0.0f, depth / max_depth));
//    int r = static_cast<int>(norm_depth * 255);
//    int g = 0;
//    int b = 255 - r;
//    return cv::Vec3b(b, g, r);
//}
cv::Vec3b depthToColor(float depth, float max_depth) {
    float norm_depth = std::min(1.0f, std::max(0.0f, depth / max_depth));
    int hue = static_cast<int>(norm_depth * 240); // 从红色到蓝色的色调范围
    cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue, 255, 255)); // HSV颜色
    cv::Mat bgr;
    cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR); // 转换为BGR颜色
    return bgr.at<cv::Vec3b>(0, 0);
}

// Function to project 3D point cloud onto a 2D image and color the points based on the image
void projection(const pcl::PointCloud<pcl::PointXYZ>::Ptr& ccloud, cv::Mat& img,
                const cv::Mat& rotate_mat, const cv::Mat& transform_vec,
                const cv::Mat& camera_mat, const cv::Mat& dist_coeff,
                ros::Publisher& cloud_pub, ros::Publisher& image_pub)
{
    cv::Mat img2=img.clone();//带雷达点云的2d图像

    std::vector<cv::Point3f> points3d;//3d雷达点
    points3d.reserve(ccloud->size());
    for (const auto& pt : ccloud->points)
    {
        points3d.emplace_back(pt.x, pt.y, pt.z);
    }

    std::vector<cv::Point2f> projectedPoints;//投影到2d的地图点
    cv::projectPoints(points3d, rotate_mat, transform_vec, camera_mat, dist_coeff, projectedPoints);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);//染色点云
    float max_depth = 4.0f; // todo 最大深度值，可根据需要调整
    for (size_t i = 0; i < projectedPoints.size(); ++i)
    {
        const auto& p = projectedPoints[i];
        pcl::PointXYZRGB point_rgb;
        point_rgb.x = ccloud->points[i].x;
        point_rgb.y = ccloud->points[i].y;
        point_rgb.z = ccloud->points[i].z;

        if (p.y >= 0 && p.y < img.rows && p.x >= 0 && p.x < img.cols && point_rgb.y < 0)//投影到2d的地图点在图像范围内，且是雷达前方的点
//        if (p.y >= 0 && p.y < img.rows && p.x >= 0 && p.x < img.cols )//投影到2d的地图点在图像范围内，且是雷达前方的点
        {
            // Retrieve the color from the image
            cv::Vec3b color = img.at<cv::Vec3b>(static_cast<int>(p.y), static_cast<int>(p.x));
            point_rgb.r = color[2];
            point_rgb.g = color[1];
            point_rgb.b = color[0];
            rgb_cloud->points.push_back(point_rgb);

            // Calculate the depth color based on the -y value (depth)
            float depth = -point_rgb.y; // Use -y as the depth value
            // Map normalized depth to a color
            cv::Vec3b color2 = depthToColor(depth, max_depth);
            cv::circle(img2, p, 4, cv::Scalar(color2[0], color2[1], color2[2]), 2, 8, 0);
        }
    }

    rgb_cloud->width = rgb_cloud->points.size();
    rgb_cloud->height = 1;
    rgb_cloud->is_dense = true;

    // Convert to ROS message and publish
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*rgb_cloud, ros_cloud);
    ros_cloud.header.frame_id = "map";
    cloud_pub.publish(ros_cloud);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img2).toImageMsg();
    image_pub.publish(msg);
}

// Function to publish the original point cloud
void publishOriginalCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& ccloud, ros::Publisher& original_cloud_pub)
{
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*ccloud, ros_cloud);
    ros_cloud.header.frame_id = "map";
    original_cloud_pub.publish(ros_cloud);
}

// Function to publish the origin point for reference
void publishOrigin(ros::Publisher& origin_pub)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr origin_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointXYZRGB origin;
    origin.x = 0.0;
    origin.y = 0.0;
    origin.z = 0.0;
    origin.r = 255;
    origin.g = 255;
    origin.b = 0;
    origin_cloud->points.push_back(origin);

    sensor_msgs::PointCloud2 ros_origin;
    pcl::toROSMsg(*origin_cloud, ros_origin);
    ros_origin.header.frame_id = "map";
    origin_pub.publish(ros_origin);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_projection");
    ros::NodeHandle nh;

    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("projected_cloud", 1);
    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("projected_image", 1);
    ros::Publisher origin_pub = nh.advertise<sensor_msgs::PointCloud2>("origin_point", 1);
    ros::Publisher original_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("original_cloud", 1);

    cv::Mat extrinsic_mat, camera_mat, dist_coeff;
//    cv::FileStorage fs_read("/media/hl/One_Touch/ubuntu_share/Dataset/MYNTEYE_D1000/cam_lidar/result1/result1.yaml", cv::FileStorage::READ);
    cv::FileStorage fs_read("/media/hl/One_Touch/ubuntu_share/Dataset/MYNTEYE_D1000/cam_lidar/result2/result2.yaml", cv::FileStorage::READ);
    fs_read["CameraExtrinsicMat"] >> extrinsic_mat;
    fs_read["CameraMat"] >> camera_mat;
    fs_read["DistCoeff"] >> dist_coeff;
    fs_read.release();

    cv::Mat rotate_mat(3, 3, cv::DataType<double>::type);
    cv::Mat transform_vec(3, 1, cv::DataType<double>::type);
//    rotate_mat = extrinsic_mat(cv::Rect(0, 0, 3, 3)).t(); // Transpose the rotation matrix
//    transform_vec.at<double>(0) = extrinsic_mat.at<double>(1, 3); // y
//    transform_vec.at<double>(1) = extrinsic_mat.at<double>(2, 3); // z
//    transform_vec.at<double>(2) = -extrinsic_mat.at<double>(0, 3); // -x
    for(int i = 0; i < 3; ++i)
    {
        for(int j = 0; j < 3; ++j)
        {
            rotate_mat.at<double>(i, j) = extrinsic_mat.at<double>(j, i);
        }
    }
    transform_vec.at<double>(0) = extrinsic_mat.at<double>(0, 3);
    transform_vec.at<double>(1) = extrinsic_mat.at<double>(1, 3);
    transform_vec.at<double>(2) = extrinsic_mat.at<double>(2, 3);

    // Load point cloud and image
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::io::loadPCDFile<pcl::PointXYZ>("/media/hl/One_Touch/ubuntu_share/Dataset/MYNTEYE_D1000/cam_lidar/cam_lidar/pointclouds/1720014586960077006.pcd", *cloud);
//    cv::Mat image = cv::imread("/media/hl/One_Touch/ubuntu_share/Dataset/MYNTEYE_D1000/cam_lidar/cam_lidar/images/1720014586977703670.png");
    pcl::io::loadPCDFile<pcl::PointXYZ>("/media/hl/One_Touch/ubuntu_share/Dataset/MYNTEYE_D1000/cam_lidar/cam_lidar/pointclouds/1720014585258922841.pcd", *cloud);
    cv::Mat image = cv::imread("/media/hl/One_Touch/ubuntu_share/Dataset/MYNTEYE_D1000/cam_lidar/cam_lidar/images/1720014585248763058.png");

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        projection(cloud, image, rotate_mat, transform_vec, camera_mat, dist_coeff, cloud_pub, image_pub);
        publishOriginalCloud(cloud, original_cloud_pub); // Publish original point cloud
        publishOrigin(origin_pub); // Publish origin point
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
//激光雷达通常以圆形或扇形模式进行扫描。例如，旋转式激光雷达会围绕中心进行360度旋转扫描，这会导致在平面图像上看到弧形的边缘。
//将三维点云数据投影到二维图像平面上时，投影方式也可能导致弧形边缘。常见的投影方法包括球面投影和柱面投影，这些投影方式会将原本直线的点在图像上显示为弧形。