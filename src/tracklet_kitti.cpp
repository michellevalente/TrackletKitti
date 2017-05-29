#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/crop_box.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Quaternion.h>
#include "tracklets.h"

ros::Publisher pub_pc;
ros::Publisher marker_pub;
image_transport::Publisher pub_camera;
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
Tracklets * tracklets(new Tracklets());
Eigen::MatrixXd tmat;
Eigen::MatrixXd pmat;
Eigen::MatrixXd rmat;
cv::Mat rect_view;
bool image_initialized = false;

static const std::string OPENCV_WINDOW = "Image window";


void camera_cb(const sensor_msgs::ImageConstPtr& msg)
{

  cv::Size imageSize = cv::Size(msg->width,msg->height);
  cv_bridge::CvImagePtr cv_cam = cv_bridge::toCvCopy(msg, "rgb8");

  rect_view = cv_cam->image;

  //cv::imshow(OPENCV_WINDOW,  rect_view);
  sensor_msgs::ImagePtr msg_send = cv_bridge::CvImage(std_msgs::Header(), "rgb8", rect_view).toImageMsg();
  //image_initialized = true;
  pub_camera.publish(msg_send);

}

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  //ROS_INFO("Got point cloud!");

  visualization_msgs::Marker marker_delete;
  marker_delete.action = 3;
  marker_pub.publish(marker_delete);
  pcl::fromROSMsg(*input, *laserCloudIn);

    for(int i = 0; i < tracklets->numberOfTracklets(); i++){
    visualization_msgs::Marker marker;
    if(tracklets->isActive(i,input->header.seq)){
      Tracklets::tTracklet* tracklet = tracklets->getTracklet(i);
      Tracklets::tPose *pose;
      if(tracklets->getPose(i, input->header.seq, pose)){
         marker.header.frame_id = "/velo_link";
         marker.header.stamp = ros::Time::now();
         marker.ns = "label";
         marker.id = i;
         marker.action = visualization_msgs::Marker::ADD;
         marker.type = visualization_msgs::Marker::CUBE;
        
        tf::Quaternion q;
        q.setEuler(pose->rx, pose->ry, pose->rz);
        marker.pose.position.x = pose->tx;
        marker.pose.position.y = pose->ty;
        marker.pose.position.z = pose->tz;
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = tracklet->l;
        marker.scale.y = tracklet->w;
        marker.scale.z = tracklet->h;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();

        marker_pub.publish(marker);

      }

    }
  }

  sensor_msgs::PointCloud2 scan_color = sensor_msgs::PointCloud2();
  pcl::toROSMsg(*laserCloudIn, scan_color);
  scan_color.header.frame_id = "velo_link";
  pub_pc.publish (scan_color);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");

  if (!tracklets->loadFromFile("/home/michelle/datasets/tracklet_labels.xml"))
    std::cout << "ERROR " << std::endl;

  tmat = Eigen::MatrixXd(4,4);
  tmat <<   7.533745e-03, -9.999714e-01, -6.166020e-04, -4.069766e-03,
            1.480249e-02,7.280733e-04, -9.998902e-01, -7.631618e-02,
             9.998621e-01, 7.523790e-03, 1.480755e-02, -2.717806e-01,
             0.0f, 0.0f, 0.0f, 1.0f;

  pmat = Eigen::MatrixXd(3,4);
  pmat << 7.215377e+02, 0.000000e+00, 6.095593e+02,
   0.000000e+00, 0.000000e+00, 7.215377e+02,
    1.728540e+02, 0.000000e+00, 0.000000e+00,
     0.000000e+00, 1.000000e+00, 0.000000e+00;

  rmat = Eigen::MatrixXd(4,4);
  rmat << 9.999239e-01, 9.837760e-03, -7.445048e-03,0,
   -9.869795e-03, 9.999421e-01, -4.278459e-03,0,
    7.402527e-03, 4.351614e-03, 9.999631e-01,0,
    0,0,0,1;

  ros::NodeHandle nh;

  cv::namedWindow(OPENCV_WINDOW);
  cv::startWindowThread();

  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_camera = it.subscribe("/kitti/camera_color_left/image_raw", 0, camera_cb);

    // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub_pc = nh.subscribe ("/kitti/velo/pointcloud", 0, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub_pc = nh.advertise<sensor_msgs::PointCloud2> ("output_pc", 10);

  pub_camera = it.advertise("camera/image", 10);

  ros::spin();
  cv::destroyWindow(OPENCV_WINDOW);
  delete tracklets;
}
