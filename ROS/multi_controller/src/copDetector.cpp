 /*
  * Detect Blue Copernicus and find the relative position of it
  */

 // Include the ROS library
 #include <ros/ros.h>

 // Include opencv2
 #include <opencv2/core/mat.hpp>
 #include <opencv2/highgui.hpp>
 #include <opencv2/imgproc.hpp>


 // Include CvBridge, Image Transport, Image msg
 #include <image_transport/image_transport.h>
 #include <cv_bridge/cv_bridge.h>
 #include <sensor_msgs/image_encodings.h>
 #include <sensor_msgs/PointCloud2.h>
 #include <geometry_msgs/Point.h>
 #include <geometry_msgs/PoseStamped.h>
 #include <geometry_msgs/Pose.h>

// Include tf2 for transformation
 #include <tf2_ros/buffer.h>
 #include <tf2_ros/transform_listener.h>
 #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

 #include "multi_controller/target_position.h"
#include "aol_stack/Target_deets.h"

#include <stdexcept>

 // Topics
std::string robot_prefix;
std::string IMAGE_TOPIC = "/color/image_raw";
std::string POINT_CLOUD2_TOPIC = "/depth/points";
std::string TARGET_TOPIC = "/target_position";

// Frames
std::string from_frame = "/camera_correct";
std::string to_frame = "/map";

float spawn_x = 0;
float spawn_y = 0;


 // Publisher
ros::Publisher target_pub;

tf2_ros::Buffer tf_buffer;


cv::Mat camera_image;

cv::Point2f target_centroid;

double detection_score = 0;

geometry_msgs::Point box_position_base_frame;
geometry_msgs::Point target_position_base_frame;


int iLowH = 67;
int iHighH = 118;

int iLowS = 31;
int iHighS = 173;

int iLowV = 0;
int iHighV = 129;

cv::Mat apply_cv_algorithms(cv::Mat camera_image) {
  // convert the image to grayscale format
  cv::Mat img_hsv;
  cv::cvtColor(camera_image, img_hsv, cv::COLOR_BGR2HSV);

  cv::Mat imgThreshold;
  cv::inRange(img_hsv, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThreshold);

  // Remove artifacts from bg
   cv::erode(imgThreshold, imgThreshold, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
   cv::dilate(imgThreshold, imgThreshold, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

  // Smooth the image
   cv::dilate(imgThreshold, imgThreshold, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
   cv::erode(imgThreshold, imgThreshold, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

  // cv::Mat canny_output;
  // cv::Canny(img_gray,canny_output,10,350);

  return imgThreshold;
}

cv::Point2f extract_centroids(cv::Mat canny_output) {

  // detect the contours on the binary image using cv2.CHAIN_APPROX_NONE
   std::vector<std::vector<cv::Point>> contours;
   std::vector<cv::Vec4i> hierarchy;
   cv::findContours(canny_output, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
   double cont_area = 0;


  // get the moment
  cv::Moments mu_big;

  for( int i = 0; i<contours.size(); i++ )
  {
    if (cv::contourArea(contours[i]) > cont_area)
    {
      mu_big = cv::moments(contours[i], false );
      cont_area = cv::contourArea(contours[i]);
    }

  }
  detection_score = cont_area/5300;

  // get the centroid of figure.
  cv::Point2f centroid_big;
  centroid_big = cv::Point2f(mu_big.m10/mu_big.m00, mu_big.m01/mu_big.m00);

  // draw contours
  cv::Mat drawing(canny_output.size(), CV_8UC3, cv::Scalar(255,255,255));

  for( int i = 0; i<contours.size(); i++ )
  {
  cv::Scalar color = cv::Scalar(167,151,0); // B G R values
  cv::drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
  }

  cv::circle( drawing, centroid_big, 4, cv::Scalar(160,15,0), -1, 8, 0 );
  if(!std::isnan(centroid_big.x) && !std::isnan(centroid_big.y))
    ROS_INFO_STREAM("Centre of the box is at"<< " --> "  << centroid_big.x << " " << centroid_big.y<<"\n"<<" Area is: "<<cont_area<<" Detection Score: "<<detection_score);


  // cv::namedWindow( "Control", cv::WINDOW_AUTOSIZE );
  // cv::namedWindow( "Extracted centroids", cv::WINDOW_AUTOSIZE );

  //Create trackbars in "Control" window
  //  cv::createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
  //  cv::createTrackbar("HighH", "Control", &iHighH, 179);
  //  cv::createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
  //  cv::createTrackbar("HighS", "Control", &iHighS, 255);
  //  cv::createTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
  //  cv::createTrackbar("HighV", "Control", &iHighV, 255);


   // show the resuling image
  // cv::imshow( "Control", canny_output);
  // cv::imshow( "Extracted centroids", drawing );
  // cv::waitKey(3);

  return centroid_big;
}

void image_cb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat canny_output = apply_cv_algorithms(cv_ptr->image);

  target_centroid = extract_centroids(canny_output);
}


geometry_msgs::Point pixel_to_3d_point(const sensor_msgs::PointCloud2 pCloud, const int u, const int v)
{
  // get width and height of 2D point cloud data
  int width = pCloud.width;
  int height = pCloud.height;

  // Convert from u (column / width), v (row/height) to position in array
  // where X,Y,Z data starts
  int arrayPosition = v*pCloud.row_step + u*pCloud.point_step;

  // compute position in array where x,y,z data start
  int arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
  int arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
  int arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8

  // ROS_INFO_STREAM("Index"<<arrayPosition<<"X"<<arrayPosX<<"Y"<<arrayPosY<<"Z"<<arrayPosZ);

  float X = 0.0;
  float Y = 0.0;
  float Z = 0.0;

  memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
  memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
  memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));

  geometry_msgs::Point p;
  p.x = X;
  p.y = Y;
  p.z = Z;

  return p;
}

geometry_msgs::Point transform_between_frames(geometry_msgs::Point p, const std::string from_frame, const std::string to_frame) {

  geometry_msgs::PoseStamped input_pose_stamped;
  input_pose_stamped.pose.position = p;
  input_pose_stamped.header.frame_id = from_frame;
  input_pose_stamped.header.stamp = ros::Time(0); //ros::Time::now();

  geometry_msgs::PoseStamped output_pose_stamped = tf_buffer.transform(input_pose_stamped, to_frame, ros::Duration(1));
  output_pose_stamped.pose.position.x = output_pose_stamped.pose.position.x +spawn_x;
  output_pose_stamped.pose.position.y = output_pose_stamped.pose.position.y +spawn_y;
  return output_pose_stamped.pose.position;
}

void point_cloud_cb(const sensor_msgs::PointCloud2 pCloud) {

  geometry_msgs::Point target_position_camera_frame;
  target_position_camera_frame = pixel_to_3d_point(pCloud, target_centroid.x, target_centroid.y);
  if(!std::isnan(target_position_camera_frame.x) && !std::isnan(target_position_camera_frame.y))
    ROS_INFO_STREAM("[PRE] 3d target position base frame: x " << target_position_camera_frame.x << " y " << target_position_camera_frame.y << " z " << target_position_camera_frame.z);

  target_position_base_frame = transform_between_frames(target_position_camera_frame, from_frame, to_frame);
  if(!std::isnan(target_position_base_frame.x) && !std::isnan(target_position_base_frame.y))
  {
    ROS_INFO_STREAM("3d target position base frame: x " << target_position_base_frame.x  << " y " << target_position_base_frame.y << " z " << target_position_base_frame.z << " robot: "<< robot_prefix);
  }
  aol_stack::Target_deets msg_to_send;
  msg_to_send.x = target_position_base_frame.x;
  msg_to_send.y = target_position_base_frame.y;
  msg_to_send.conf_score = detection_score;
  target_pub.publish(msg_to_send);
}

// service call response
bool get_target_position(multi_controller::target_position::Request  &req,
    multi_controller::target_position::Response &res) {
      res.box_position = box_position_base_frame;
      res.target_position = target_position_base_frame;
      return true;
    }

 // Main function
int main(int argc, char **argv)
{
  // The name of the node
  ros::init(argc, argv, "multi_controller");
  if (ros::param::has("robot_prefix")) {
    ros::param::get("robot_prefix", robot_prefix);
    ros::param::get("robot_prefix", spawn_x);
    IMAGE_TOPIC = "/" + robot_prefix + IMAGE_TOPIC;
    POINT_CLOUD2_TOPIC = "/" + robot_prefix + POINT_CLOUD2_TOPIC;
    TARGET_TOPIC = "/" + robot_prefix + TARGET_TOPIC;
    from_frame = robot_prefix + from_frame;
    to_frame = robot_prefix + to_frame;
  }
  else{
    throw std::invalid_argument("ERROR: robot_prefix not mentioned");
  }
  if (ros::param::has("x_") && ros::param::has("y_"))
  {
    ros::param::get("x_", spawn_x);
    ros::param::get("y_", spawn_y);
  }
  else{
    throw std::invalid_argument("ERROR: x_ and y_ not mentioned");
  }

  // Default handler for nodes in ROS
  ros::NodeHandle nh("");

    // Used to publish and subscribe to images
  image_transport::ImageTransport it(nh);

    // Subscribe to the /camera raw image topic
  image_transport::Subscriber image_sub = it.subscribe(IMAGE_TOPIC, 1, image_cb);

    // Subscribe to the /camera PointCloud2 topic
  ros::Subscriber point_cloud_sub = nh.subscribe(POINT_CLOUD2_TOPIC, 1, point_cloud_cb);
  target_pub = nh.advertise<aol_stack::Target_deets>(TARGET_TOPIC, 1000);

  tf2_ros::TransformListener listener(tf_buffer);

  // ros::ServiceServer service = nh.advertiseService("target_position",  get_target_position);

  // Make sure we keep reading new video frames by calling the imageCallback function
  ros::spin();

  // Close down OpenCV
  // cv::destroyWindow("view");
}



