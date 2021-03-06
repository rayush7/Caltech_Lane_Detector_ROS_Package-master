// Lane Detection ROS Publisher
// argv[1] : ../../../../data
// ../../../../shared_folder/unity/imgs

//Topic1 : /camera/depth/image
//Topic2 : /stereo/right/image


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <std_msgs/String.h>
#include <string>

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{// main function

  // Counter Variable
  int count=1;

  // Path to Image Folder  
// char path[1000]=argv[1];
  // char path[1000]="../../../../caltech-lanes/Unity_Images";

  // Image
  char imD[1000];
  char imR[1000];
  
  sprintf(imD,"%s/depth_right%d.png",argv[1],count);
  sprintf(imR,"%s/camera_right%d.png",argv[1],count);

  ros::init(argc, argv,"Stereo_Image_Publisher");
  ros::NodeHandle nh; 
  image_transport::ImageTransport it(nh); 

  // Declaring Left and Right Images
  cv::Mat imageD;
  cv::Mat imageR;

  // Image Publishers
  image_transport::Publisher pubD = it.advertise("/camera/depth/image",10);
  image_transport::Publisher pubR = it.advertise("/stereo/right/image",10);


 //Image Message Declaration 
  sensor_msgs::ImagePtr msgD;
  sensor_msgs::ImagePtr msgR;

 // ros::Rate loop_rate(5000);
  
  while (nh.ok()) 
      { // while begins
                
          
          //Loading Opencv Images
          imageD = cv::imread(imD,CV_LOAD_IMAGE_COLOR);
          imageR = cv::imread(imR,CV_LOAD_IMAGE_COLOR);

          // Waiting for the Next Image
          if(!imageD.data || !imageR.data)
           {
         //       cout << "Waiting for Unity to Generate the Images\n\n\n\n";
                continue;
           }


         //Converting Opencv Image to ROS Messages                
          msgD = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageD).toImageMsg();
          msgR = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageR).toImageMsg();

         //Adding Information to Messages
          ros::Time stamp = ros::Time::now();
          
          msgR->header.stamp = stamp;
          msgR->header.seq=(uint32_t)count;
          msgR->width=imageR.cols;
          msgR->width=imageR.cols;
          
          msgD->header.seq=(uint32_t)count;
          msgD->header.stamp = stamp;
          msgD->height= imageD.rows;
          msgD->width=imageD.cols;
          
          
         //Publishing Information
          pubD.publish(msgD);
//          ROS_INFO("%s PUBLISHED",imD);

          pubR.publish(msgR);
//          ROS_INFO("%s PUBLISHED",imR);
//          printf("\n\n");

          count++;
          sprintf(imD,"%s/depth_right%d.png",argv[1],count);   
          sprintf(imR,"%s/camera_right%d.png",argv[1],count);   

          ros::Rate loop_rate(1000);
          ros::spinOnce();
          loop_rate.sleep();

       } // while ends
  
 return 0;

} // main function
