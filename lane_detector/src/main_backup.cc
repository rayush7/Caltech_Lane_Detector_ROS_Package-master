// Lane Detection

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "main.hh"
#include "cmdline.h"
#include "LaneDetector.hh"
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <opencv/cv.h>
#include <cv.h>
#include <highgui.h>
#include <stdlib.h>
#include "lane_detector/LaneDataSim.h"
#include <ros/console.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/bind.hpp>
#include <cmath>
#include <limits.h>
#include <climits>

#include "CKalmanFilter.hh"

// Horizontal Focal Length in pixel units
#define fx 74

// Vertical Focal Length in pixel units
#define fy 74

// Principle Points
#define px 128
#define py 128

// Scale Factor for Depth
#define sd 108


int i=0;
using namespace std;
using namespace cv;

cv::Mat raw_mat1;
cv::Mat depth_mat1;

// Declaring the global 2D Vector
vector<vector<vector<float> > >P;

typedef struct Det_Points
{
  
  CvPoint3D32f points[4];
  
} Det_Points;

// Message Filters
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> policy_t;


// Prototyping the functions
CvPoint3D32f InvPerspTransf(cv::Mat Depth, CvPoint3D32f im_point);
CvPoint3D32f downscale_point(CvPoint3D32f im_point);
 

#define MSG(fmt, ...) \
  (fprintf(stdout, "%s:%d msg   " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__) ? 0 : 0)

// Useful error macro
#define ERROR(fmt, ...) \
  (fprintf(stderr, "%s:%d error " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__) ? -1 : -1)


namespace LaneDetector
{

    /**
     * This function reads lines from the input file into a vector of strings
     *
     * \param filename the input file name
     * \param lines the output vector of lines
     */
  bool ReadLines(const char* filename, vector<string> *lines)
  {
    // make sure it's not NULL
    if (!lines)
      return false;
    // resize
    lines->clear();

    ifstream  file;
    file.open(filename, ifstream::in);
    char buf[5000];
    // read lines and process
    while (file.getline(buf, 5000))
    {
      string str(buf);
      lines->push_back(str);
    }

    // close

    file.close();
    return true;
  }

/** This function converts Spline points to vec2f points
*
*
*/
vector<Vec2f> mcvConvertSplineTOVec2f(CvPoint2D32f* points)
{
  vector<Vec2f> lines(4);
  lines[0][1] = points[0].x;
  lines[0][1] = points[0].y;

  lines[1][0] = points[1].x;
  lines[1][1] = points[1].y;

  lines[2][0] = points[2].x;
  lines[2][1] = points[2].y;

  lines[3][0] = points[3].x;
  lines[3][1] = points[3].y;

  return lines;
}

/** This function converts vec2f points to Spline points
*
*
*/
Spline mcvConvertVec2fTOSpline(Vector<Vec2f> lines)
{

  Spline spline;
  spline.points[0].x = lines[0][0];
  spline.points[0].y = lines[0][1];

  spline.points[1].x = lines[1][0];
  spline.points[1].y = lines[1][1];

  spline.points[2].x = lines[2][0];
  spline.points[2].y = lines[2][1];

  spline.points[3].x = lines[3][0];
  spline.points[3].y = lines[3][1];

  return spline;
}

/*
 *
 */
void compute_slope_from_spline(vector<Vec2f> points)
{

}

    /**
     * This function processes an input image and detects lanes/stoplines
     * based on the passed in command line arguments
     *
     * \param filename the input file name
     * \param cameraInfo the camera calibration info
     * \param lanesConf the lane detection settings
     * \param stoplinesConf the stop line detection settings
     * \param options the command line arguments
     * \param outputFile the output file stream to write output lanes to
     * \param index the image index (used for saving output files)
     * \param elapsedTime if NOT NULL, it is accumulated with clock ticks for
     *        the detection operation
     */
  /*
     void ProcessImage( CameraInfo& cameraInfo,
                  LaneDetectorConf& lanesConf, LaneDetectorConf& stoplinesConf,
                  gengetopt_args_info& options, ofstream* outputFile,
                  int index, clock_t *elapsedTime,cv::Mat raw_mat2)  //defined in IPM.hh */
  void ProcessImage( CameraInfo& cameraInfo,
               LaneDetectorConf& lanesConf, LaneDetectorConf& stoplinesConf,
               gengetopt_args_info& options, ofstream* outputFile,
               int index, clock_t *elapsedTime,cv::Mat raw_mat2, int count, CKalmanFilter KF2)  //defined in IPM.hh1

    {//begin


     // Start Debugging it right here. raw_mat2 is correctly view

      // IplImage
     
      IplImage* im;
      im = cvCreateImage(cvSize(raw_mat2.cols,raw_mat2.rows),8,3);

     // Conversion from cv::Mat to IplImage
      IplImage ipltemp = raw_mat2;
      //cvCopy(&ipltemp,im);
      im = &ipltemp;

     //cvNamedWindow("New", CV_WINDOW_AUTOSIZE);
     //cvShowImage("New", im);
     //cvWaitKey(30);

      CvMat temp;
      
      if (!im)
      {        
          // print error and exit
          cout << "ERROR: Capture is null!\n";
          return ; 
      }    
      cvGetMat(im, &temp);
      CvMat *schannel_mat;
      CvMat* tchannelImage = cvCreateMat(im->height, im->width, INT_MAT_TYPE);
      cvSplit(&temp, tchannelImage, NULL, NULL, NULL);

      // convert to float
      CvMat *channelImage= cvCreateMat(im->height, im->width, FLOAT_MAT_TYPE);
      cvConvertScale(tchannelImage, channelImage, 1./255);
      vector<FLOAT> lineScores, splineScores;
      vector<Line> lanes, lanes_pre;
      vector<Spline> splines;
      clock_t startTime = clock();

     // MSG ("procesing images");
      mcvGetLanes(channelImage, &temp, &lanes, &lineScores, &splines, &splineScores,   //1st image=5
                  &cameraInfo, &lanesConf, NULL);   ///defined in lanedetector.cc


    //  ROS_INFO("processsed");

      clock_t endTime = clock();

    //  MSG("Found %d lanes in %f msec", splines.size(),static_cast<double>(endTime - startTime) / CLOCKS_PER_SEC * 1000.);

      // update elapsed time
      if (elapsedTime)
        (*elapsedTime) += endTime - startTime;


      // show or save
      if (options.show_flag || options.save_images_flag)
      {

         CvMat *imDisplay = cvCloneMat(&temp);

        if (lanesConf.ransacLine && !lanesConf.ransacSpline)
          for(int i=0; i<lanes.size(); i++)
            mcvDrawLine(imDisplay, lanes[i], CV_RGB(0,125,0), 3);
        // print lanes
        if (lanesConf.ransacSpline)
        {
            //

          for(int i=0; i<splines.size(); i++)
          {

              /**************************************************************************/
                 // convert spline type to vec2f type
                 vector<Vec2f> lines = mcvConvertSplineTOVec2f(splines.points);

                //// Kalman Filter to predict the next state
                CKalmanFilter KF2(lines_pre); // Initialization
                vector<Vec2f> pp = KF2.predict(); // Prediction

                vector<Vec2f> lines2Final = KF2.update(lines); // Correction
                lines = lines2Final; // updating the model
               // imgFinal = detect.drawLines(img2,lines2, imname); // draw final Lane Markings on the original image

                lines_pre = lines;
                spline = mcvConvertVec2fTOSpline(lines);

              /***************************************************************************/


            // Putting the points in the Spline Vector
              // detected_points[i].points[0].x=splines[i].points[0].x;
               
            if (splines[i].color == LINE_COLOR_YELLOW)
              mcvDrawSpline(imDisplay, splines[i], CV_RGB(0,255,0), 3);
            else
              mcvDrawSpline(imDisplay, splines[i], CV_RGB(0,255,0), 3);
          }
        }
        // show?
        if (options.show_flag)
        {
          // set the wait value
          int wait = options.step_flag ? 0 : options.wait_arg;

          // show image with detected lanes
          SHOW_IMAGE(imDisplay, "Detected Lanes", wait);
        }

      // Putting the values in detected Points
      //  P.resize(splines.size());
      /*    P.resize(2);
	
          for(int i=0;i<2;i++)
           {
              P[i].resize(4);

              for(int j=0;j<4;j++)
              {
                 P[i][j].resize(2);
                 P[i][j][0]=splines[i].points[j].x;
                 P[i][j][1]=splines[i].points[j].y;                 
              }
           }*/

          cvReleaseMat(&imDisplay);
      }
     
    } // end


//--------PROCESS FUNCTION---------------------------------------------------------------------------------------------------------------------


    //int Process(int argc, char** argv ,cv::Mat raw_mat2)
    int Process(int argc, char** argv ,cv::Mat raw_mat2, int count, CKalmanFilter KF2)
    {
      //parse the command line paramters
      gengetopt_args_info options;

      if (cmdline_parser (argc, argv,  &options) < 0)
        return -1;

      //read the camera configurations
      CameraInfo cameraInfo;  //defined in IPM.hh
      mcvInitCameraInfo(options.camera_conf_arg, &cameraInfo); //defined in inverseperspectivemaping.cc
      //MSG("\nLoaded camera file\n");

      //read the configurations
      LaneDetectorConf lanesConf, stoplinesConf; //struct definition found in Landedetector.hh(which has params stored in Lanes.conf)

      if (!options.no_lanes_flag)
      {
        mcvInitLaneDetectorConf(options.lanes_conf_arg, &lanesConf); //defined in lanedetector.cc
       //MSG("\nLoaded lanes config file\n");
      }

      if (!options.no_stoplines_flag)
      {
        mcvInitLaneDetectorConf(options.stoplines_conf_arg, &stoplinesConf);
        MSG("Loaded stop lines config file");
      }

      //elapsed time
      clock_t elapsed = 0;

      //Calling Process Image function
      //ProcessImage( cameraInfo, lanesConf, stoplinesConf,options, NULL, elapsed, 0 , raw_mat2);
      ProcessImage( cameraInfo, lanesConf, stoplinesConf,options, NULL, elapsed, 0 , raw_mat2, count, KF2);
      double elapsedTime = static_cast<double>(elapsed) / CLOCKS_PER_SEC;
     //MSG("Total time %f secs for 1 image = %f Hz", elapsedTime,1. / elapsedTime);
      return 0;

    }
    
} // namespace LaneDetector

//-------------------------Image Callback Function----------------------------------------------------------------------

/// callback function to subscribe images.
void imageCallback(const sensor_msgs::ImageConstPtr& msgD,const sensor_msgs::ImageConstPtr& msgRgb)
{
    

    ++i;
    cv::Mat im_new;

    try  
     { 
        im_new = cv_bridge::toCvShare(msgRgb, "bgr8")->image;        
     } 

    catch (cv_bridge::Exception& im_new)
     {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msgRgb->encoding.c_str());
     }


    try  
     { 
        depth_mat1 = cv_bridge::toCvShare(msgD, "bgr8")->image;
     } 

    catch (cv_bridge::Exception& depth_mat1)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msgD->encoding.c_str());
    }


     //Size size(512,512);
     Size size(640,480);
     resize(im_new,raw_mat1,size);



   return;
    
}

//--------------------------------------------MAIN FUNCTION---------------------------------------------------------------------

// main entry point
int main(int argc, char** argv)
{ // main begins
   
     int count = 0;
      ros::init(argc, argv, "my_subsc");
      ros::NodeHandle nh;
      cv::startWindowThread();
      image_transport::ImageTransport it(nh);
      cv::Mat depth_mat2;
      
      //Declaring the Publisher to send info to the Planner
      ros::Publisher Lane_pub = nh.advertise<lane_detector::LaneDataSim>("LaneData", 100);
  
     //Declaring the Message to be published
      lane_detector::LaneDataSim lanemsg;
  
     // Subscriber

     message_filters::Subscriber<sensor_msgs::Image> subD(nh,"/camera/depth/image", 1);
     message_filters::Subscriber<sensor_msgs::Image> subRgb(nh,"/stereo/right/image", 1);
     message_filters::Synchronizer<policy_t> sync(policy_t(10),subD,subRgb);

     sync.registerCallback(boost::bind(imageCallback,_1, _2));

 
  
      while(ros::ok())
       { // while loop begins
          // count variable to check image being processed
          count++;

           ros::spinOnce();

           if(i>0)
            {

               if (!raw_mat1.data)
                {       
                  // print error and exit
                  cout << "EMPTY IMAGE\n";
               
                }  
  
                 // Cloning the Depth Image
                 depth_mat2=depth_mat1.clone();

                 //ROS_INFO("Calling Process");
                 //LaneDetector::Process(argc,argv, raw_mat1);
                 LaneDetector::Process(argc,argv, raw_mat1, count);


                // Getting the World Coordinates of the detected points
                /* Det_Points Point[2];

                for(int i=0;i<2;i++)
                 {
                    for(int j=0;j<4;j++)
                     {
                        // Point[i].points[j].x=P[i][j][0];
                        // Point[i].points[j].y=P[i][j][1];
                         Point[i].points[j].x=0;
                         Point[i].points[j].y=1;
                     }
                }*/


              // Downscaling the Points Here

                /*for(int i=0;i<2;i++)
                 {
                    for(int j=0;j<4;j++)
                     {
                         Point[i].points[j]=downscale_point(Point[i].points[j]);
                         
                     }
                }*/

              
             // Perform Inverse Perspective Transformation (From 2D to 3D)

             /*for(int i=0;i<2;i++)
                 {
                    for(int j=0;j<4;j++)
                     {
                         Point[i].points[j]=InvPerspTransf(depth_mat2,Point[i].points[j]);
                     }
                }*/


            //Putting the Data in the Message         

            /*lanemsg.P1.x=Point[0].points[0].x;
            lanemsg.P1.y=Point[0].points[0].y;
            lanemsg.P1.z=Point[0].points[0].z;

            lanemsg.P2.x=Point[0].points[1].x;
            lanemsg.P2.y=Point[0].points[1].y;
            lanemsg.P2.z=Point[0].points[1].z;

            lanemsg.P3.x=Point[0].points[2].x;
            lanemsg.P3.y=Point[0].points[2].y;
            lanemsg.P3.z=Point[0].points[2].z;

            lanemsg.P4.x=Point[0].points[3].x;
            lanemsg.P4.y=Point[0].points[3].y;
            lanemsg.P4.z=Point[0].points[3].z;

            lanemsg.P5.x=Point[1].points[0].x;
            lanemsg.P5.y=Point[1].points[0].y;
            lanemsg.P5.z=Point[1].points[0].z;

            lanemsg.P6.x=Point[1].points[1].x;
            lanemsg.P6.y=Point[1].points[1].y;
            lanemsg.P6.z=Point[1].points[1].z;

            lanemsg.P7.x=Point[1].points[2].x;
            lanemsg.P7.y=Point[1].points[2].y;
            lanemsg.P7.z=Point[1].points[2].z;

            lanemsg.P8.x=Point[1].points[3].x;
            lanemsg.P8.y=Point[1].points[3].y;
            lanemsg.P8.z=Point[1].points[3].z;

            // Publishing the Lane Message to Planner
            Lane_pub.publish(lanemsg);*/

        }// if ends
  
      }  // while loop ends
    
}// main ends


//-------------------------------------Inverse Perspective Transformation------------------------------------------------------

CvPoint3D32f InvPerspTransf(cv::Mat Depth,CvPoint3D32f im_point)
{
    // Detected Points after being Centered
       im_point.x=im_point.x-px;
       im_point.y=im_point.y-py;

    // Point w.r.t World Coordinate System
    
       CvPoint3D32f temp;
       CvPoint3D32f world_point;

    // Calculating the Encoded Depth Values. Currently using the blue channel only.

       temp.x = Depth.at<cv::Vec3b>(im_point.x,im_point.y)[0];
        

   // Z coordinate of world Coordinate System. sd is scale factor

       world_point.z=(temp.x*30)/sd;
           
    // X Coordinate of World Coordinate System
       world_point.x=((im_point.x)*world_point.z)/fx;

    // Y Coordinate of World Coordinate System  
       world_point.y=((im_point.y)*world_point.z)/fy;

      return world_point;
}

//---------------------------Detected Points in Downscaled Image--------------------------------------------------------------

CvPoint3D32f downscale_point(CvPoint3D32f im_point)
{
       CvPoint3D32f down_im_point;       

       down_im_point.x=im_point.x/2;
       down_im_point.y=im_point.y/2;

       return  down_im_point;
}
