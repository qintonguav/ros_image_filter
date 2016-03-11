#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace sensor_msgs;
using namespace message_filters;
using namespace std;
using namespace cv;

image_transport::Publisher pub_img;


void callback(const ImageConstPtr& image1, const ImageConstPtr& image2)
{
  // Solve all of perception here...
  //cout<<"callback"<<endl;
  //cout<<"iamge1 time"<<image1->header.stamp<<endl;
  //cout<<"iamge2 time"<<image2->header.stamp<<endl;
  cv_bridge::CvImagePtr bridge_ptr1 = cv_bridge::toCvCopy(image1, sensor_msgs::image_encodings::MONO8);
  cv_bridge::CvImagePtr bridge_ptr2 = cv_bridge::toCvCopy(image2, sensor_msgs::image_encodings::MONO8);
  //cout<<"a cols"<<image1->height<<endl;
  //cout<<"a raws"<<image2->width<<endl;
  int cols = bridge_ptr1->image.cols;
  int rows = bridge_ptr1->image.rows;

  Size size( cols + cols, rows);  
  
  Mat img_merge;  
  Mat outImg_left, outImg_right;  
  img_merge.create(size, CV_8UC1);  
  outImg_left = img_merge(Rect(0, 0, cols, rows));  
  outImg_right = img_merge(Rect(cols, 0, cols, rows));  

  bridge_ptr1->image.copyTo(outImg_left); 
  bridge_ptr2->image.copyTo(outImg_right);  
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(image1->header, sensor_msgs::image_encodings::MONO8, img_merge).toImageMsg();

  pub_img.publish(msg);
  //cout<<"cols"<<msg->height<<endl;
  //cout<<"raws"<<msg->width<<endl;
  //imshow("iamge",img_merge);
  //waitKey(1);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh("~");
  message_filters::Subscriber<Image> image1_sub(nh, "image1", 1);
  message_filters::Subscriber<Image> image2_sub(nh, "image2", 1);

  image_transport::ImageTransport it(nh);
  pub_img = it.advertise("image", 1);


  //typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)

  typedef sync_policies::ExactTime<Image, Image> MySyncPolicy;

  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image1_sub, image2_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ROS_INFO("image_filter begin!");

  ros::spin();

  return 0;
}