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
#include <math.h>

#define PI 3.14159265

// Horizontal Focal Length in pixel units
#define fx 73.9

// Vertical Focal Length in pixel units
#define fy 73.9

// Principle Points
#define px 128
#define py 128

// Scale Factor for Depth
#define sd 782.96

int i=0;
using namespace std;
using namespace cv;

int count_frames = 0;

cv::Mat raw_mat1;
cv::Mat depth_mat1;

// Declaring the global 2D Vector
vector<vector<float> > P;
//vector<Vec2f> P;

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

// Lines from previous frame to initialize kalman filter
vector<Spline> splines_pre;



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
  lines[0][0] = points[0].x;
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
    ///cout << lines.size() << endl;

  Spline spline;
  spline.points[0].x = lines[0][0];
  spline.points[0].y = lines[0][1];

  spline.points[1].x = lines[1][0];
  spline.points[1].y = lines[1][1];

  spline.points[2].x = lines[2][0];
  spline.points[2].y = lines[2][1];

  spline.points[3].x = lines[3][0];
  spline.points[3].y = lines[3][1];

  spline.degree = 3;
  spline.score = 0;
  spline.color = LINE_COLOR_YELLOW;

  return spline;
}

/* This function computes slope from spline points (1st and 4th)
 *
 */
double compute_slope_from_spline(CvPoint2D32f* points)
{
    double x1 = points[0].x;
    double y1 = points[0].y;

    double x4 = points[3].x;
    double y4 = points[3].y;

    double slope = atan2( (y4-y1),(x4-x1) ) * 180/PI;
    return slope;
}

/* This function generates a default left spline with four points
 *
 *
 */
Spline default_leftSpline(Spline rightSpline)
{
    Spline spline;

    spline.degree = 3;

    spline.points[0] = {rightSpline.points[0].x - 120, rightSpline.points[0].y};
    spline.points[1] = {rightSpline.points[1].x - 200, rightSpline.points[1].y};
    spline.points[2] = {rightSpline.points[2].x - 280, rightSpline.points[2].y};
    spline.points[3] = {rightSpline.points[3].x - 400, rightSpline.points[3].y};

    spline.score = rightSpline.score;
    spline.color = rightSpline.color;

    return spline;
}

/* This function generates a default right spline with four points
 *
 */
Spline default_rightSpline(Spline leftSpline)
{
    Spline spline;

    spline.degree = 3;

    spline.points[0] = {leftSpline.points[0].x + 120, leftSpline.points[0].y};
    spline.points[1] = {leftSpline.points[1].x + 200, leftSpline.points[1].y};
    spline.points[2] = {leftSpline.points[2].x + 280, leftSpline.points[2].y};
    spline.points[3] = {leftSpline.points[3].x + 400, leftSpline.points[3].y};

    spline.score = leftSpline.score;
    spline.color = leftSpline.color;

    return spline;
}

/* This function applies kalman filter on the previous and current splines
 *
 *
 */
vector<Vec2f> apply_kalman_filter(vector<Vec2f> lines_pre, vector<Vec2f> currlines)
{
   // for(int i = 0; i < currSpline.size(); i++)  // repeat for left and right lanes
  //  {

        CKalmanFilter KF2(lines_pre); // Initialization
        vector<Vec2f> pp = KF2.predict(); // Prediction

        vector<Vec2f> lines2Final = KF2.update(currlines); // Correction
        return lines2Final;
        //lines = lines2Final; // updating the model
       // imgFinal = detect.drawLines(img2,lines2, imname);
   // }

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

     void ProcessImage( CameraInfo& cameraInfo,
                  LaneDetectorConf& lanesConf, LaneDetectorConf& stoplinesConf,
                  gengetopt_args_info& options, ofstream* outputFile,
                  int index, clock_t *elapsedTime,cv::Mat raw_mat2)  //defined in IPM.hh
 /* Spline ProcessImage( CameraInfo& cameraInfo,
               LaneDetectorConf& lanesConf, LaneDetectorConf& stoplinesConf,
               gengetopt_args_info& options, ofstream* outputFile,
               int index, clock_t *elapsedTime,cv::Mat raw_mat2, int count, Spline splines_pre)  //defined in IPM.hh1
*/
    {//begin


     //   cout << "Inside Process Image function" << endl;



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
      vector<Spline> splines_all, splines(2);
      clock_t startTime = clock();

     // MSG ("procesing images");
      //mcvGetLanes(channelImage, &temp, &lanes, &lineScores, &splines, &splineScores,   //1st image=5
      mcvGetLanes(channelImage, &temp, &lanes, &lineScores, &splines_all, &splineScores,   //1st image=5
                  &cameraInfo, &lanesConf, NULL);   ///defined in lanedetector.cc


      /************************************************************************************/

    // cout << "Splines_all size = " << splines_all.size() << endl;

      if(splines_all.size() == 0)
         return;

      // Find left or right splines
      vector<bool> which_lane(splines_all.size());  // flag to identify left(0) or right(1) lane
      int x_leftmost_forRightLane = 10000, x_rightmost_forLeftLane = -1;
      int left_lane_idx = -1, right_lane_idx = -1;
      bool left_avail = false, right_avail = false;

      for(int i=0; i<splines_all.size(); i++)
      {
            Spline current_spline = splines_all[i];
            double slope = compute_slope_from_spline(current_spline.points);
            //cout << "slope: " << slope << endl;
            if(slope > 90)
            {
                left_avail = true;
                which_lane[i] = false;  // it is a left lane
                if(x_rightmost_forLeftLane < current_spline.points[0].x)
                {
                    x_rightmost_forLeftLane = current_spline.points[0].x;
                    left_lane_idx = i;
                }
            }
            else
            {
                right_avail = true;
                which_lane[i] = true;    // it is a right lane
                if(x_leftmost_forRightLane > current_spline.points[3].x)
                {
                    x_leftmost_forRightLane = current_spline.points[3].x;
                    right_lane_idx = i;
                }
            }
      }

     // cout << "numSplines >= 2 " << splines_all.size() << endl;
     // cout << x_leftmost_forRightLane << " " << x_rightmost_forLeftLane << endl;

    // cout << "I am here" << endl;
    // cout << "Spline Size = " << splines.size() << endl;
    // cout << "Spline_all Size = " << splines_all.size() << endl;


      if(left_avail == true && right_avail == true)
      {
       //   cout << "Both lanes with indices: " << left_lane_idx << " " << right_lane_idx << endl;
      //    cout << splines_all.size() << " " << splines.size() << endl;
          //splines.resize(2);
     //     cout << "First" << endl;
     //     cout << "Left_Index = " << left_lane_idx << endl;
     //     cout << "Right_Index = " << right_lane_idx << endl;
          splines[0] = splines_all[left_lane_idx];
          
          splines[1] = splines_all[right_lane_idx];
     //     cout << "Just after " << endl;
      }
      else if(left_avail == true) {

     //     cout << "Only left lane with index: " << left_lane_idx << " " << splines.size() << endl;

          //splines.resize(2);
     //     cout << "Second" << endl;
          splines[0] = splines_all[left_lane_idx];
          splines[1] = default_rightSpline(splines[0]); //generates a default right spline
      }
      else if(right_avail == true)
      {
       //   cout << "Only right lane with index: " << right_lane_idx << endl;
          //splines.resize(2);
     //     cout << "Three" << endl;
          splines[1] = splines_all[right_lane_idx];
          splines[0] = default_leftSpline(splines[1]); //generates a default left spline
      }
      else
      {
       //   cout << "no lane available, returning.." << endl;


            return;

          //cout << "numSplines < 2 "  << splines_all.size() << endl;
          //splines.resize(splines_all.size());
          //splines = splines_all;
      }

      // Initialize previous splines, if empty
      if(count_frames == 1 || splines_pre.size() == 0)
      {
      //    cout << "Initialiing previous splines.. " << endl;
        //  cout << "Inside prev frames " << endl;
          splines_pre.resize(splines.size());
          splines_pre = splines;

          return;
      }

      //************************************************************************************/
      //  cout << "showing splines "  << endl;

        //ROS_INFO("processsed");


      //cout << "Spline Size = " << endl;                         
      //  cout << "I am here 2" << endl;

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

          for(int i=0; i<splines.size(); i++)  //resize
          {
            //  cout << "drawing spline: " << i << endl;


              /**************************************************************************/
              // convert spline type to vec2f type
              vector<Vec2f> lines = mcvConvertSplineTOVec2f(splines[i].points);
              vector<Vec2f> lines_pre = mcvConvertSplineTOVec2f(splines_pre[i].points);

              // save current splines
              splines_pre[i] = splines[i];

              // apply kalman filter using previous and current lines
             //  lines = apply_kalman_filter(lines_pre, lines);
              /*
              Vec2f tmp = lines[1];
              lines[1] = lines[0];
              lines[0] = tmp;*/

              // convert back to spline representation
              splines[i] = mcvConvertVec2fTOSpline(lines);
//              // save current splines
//              splines_pre[i] = splines[i];

              /**************************************************************************/

              /************************************************************************** /
              cout << splines.size() << " " << splines_pre.size() << endl;
                 // convert spline type to vec2f type
                 vector<Vec2f> lines = mcvConvertSplineTOVec2f(splines[0].points);
                 vector<Vec2f> lines_pre = mcvConvertSplineTOVec2f(splines_pre[0].points);

                //// Kalman Filter to predict the next state
                CKalmanFilter KF2(lines_pre); // Initialization
                vector<Vec2f> pp = KF2.predict(); // Prediction

                vector<Vec2f> lines2Final = KF2.update(lines); // Correction
                lines = lines2Final; // updating the model
               // imgFinal = detect.drawLines(img2,lines2, imname); // draw final Lane Markings on the original image
/*
                Spline newSpline = mcvConvertVec2fTOSpline(lines);    // to be used in current frame
                splines[0] = newSpline;
                splines_pre.resize(splines.size());
                splines_pre = splines; //mcvCscoreonvertVec2fTOSpline(lines); //splines_pre are splines of current frame

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



   //     cout << "Size of Splines" << splines.size() << endl;


     

      //  Putting the detected Points in P vector

      

       P[0][0]=splines[0].points[0].x;
       P[0][1]=splines[0].points[0].y;

       P[1][0]=splines[0].points[1].x;
       P[1][1]=splines[0].points[1].y;

       P[2][0]=splines[0].points[2].x;
       P[2][1]=splines[0].points[2].y;

       P[3][0]=splines[0].points[3].x;
       P[3][1]=splines[0].points[3].y;

       P[4][0]=splines[1].points[0].x;
       P[4][1]=splines[1].points[0].y;

       P[5][0]=splines[1].points[1].x;
       P[5][1]=splines[1].points[1].y;

       P[6][0]=splines[1].points[2].x;
       P[6][1]=splines[1].points[2].y;

       P[7][0]=splines[1].points[3].x;
       P[7][1]=splines[1].points[3].y;



       cvReleaseMat(&imDisplay);
      }

//      return splines_pre;
     
    } // end


//--------PROCESS FUNCTION---------------------------------------------------------------------------------------------------------------------


    int Process(int argc, char** argv ,cv::Mat raw_mat2)
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

    // ROS_INFO("Inside the process function");

      //elapsed time
      clock_t elapsed = 0;

      //Calling Process Image function
      ProcessImage( cameraInfo, lanesConf, stoplinesConf,options, NULL, elapsed, 0 , raw_mat2);

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


     Size size(512,512);
     //Size size(640,480);
     resize(im_new,raw_mat1,size);



   return;
    
}

//--------------------------------------------MAIN FUNCTION---------------------------------------------------------------------

// main entry point
int main(int argc, char** argv)
{ // main begins
   
      ros::init(argc, argv, "my_subsc");
      ros::NodeHandle nh;
      cv::startWindowThread();
      image_transport::ImageTransport it(nh);
      cv::Mat depth_mat2;

       // Initialising the vector
         P.resize(8);
      
         for(int i1=0;i1<8;i1++)
           {
              P[i1].resize(2);

             for(int j=0;j<2;j++)
              {
      
                 P[i1][j]=0;                 
              }
           }

  //     cout << "Inside the main function" << endl;

      
      //Declaring the Publisher to send info to the Planner
      ros::Publisher Lane_pub = nh.advertise<lane_detector::LaneDataSim>("LaneData", 100);
  
     //Declaring the Message to be published
      lane_detector::LaneDataSim lanemsg;
  
     // Subscriber

     message_filters::Subscriber<sensor_msgs::Image> subD(nh,"/camera/depth/image", 100);
     message_filters::Subscriber<sensor_msgs::Image> subRgb(nh,"/stereo/right/image", 100);
     message_filters::Synchronizer<policy_t> sync(policy_t(10),subD,subRgb);

     sync.registerCallback(boost::bind(imageCallback,_1, _2));

//     cout << "Inside the main function 2" << endl;
  
      while(ros::ok())
       { // while loop begins
          // count variable to check image being processed
          count_frames++;

           ros::spinOnce();

           if(i>0)
            {

               if (!raw_mat1.data)
                {       
                  // print error and exit
                  cout << "EMPTY IMAGE\n";
                 // continue;
               
                }  
  
                 // Cloning the Depth Image
           //      cout << "Just above cloning the image" << endl;

                 


               //  depth_mat2=depth_mat1.clone();


         //        cout << "Just after cloning the image" << endl;

         //        ROS_INFO("Calling Process");
                 LaneDetector::Process(argc,argv, raw_mat1);


                // Getting the World Coordinates of the detected points
                CvPoint3D32f P1,P2,P3,P4,P5,P6,P7,P8;


                P1.x=P[0][0];
                P1.y=P[0][1];

                P2.x=P[1][0];
                P2.y=P[1][1];

                P3.x=P[2][0];
                P3.y=P[2][1];

                P4.x=P[3][0];
                P4.y=P[3][1];
                
                P5.x=P[4][0];
                P5.y=P[4][1];

                P6.x=P[5][0];
                P6.y=P[5][1];

                P7.x=P[6][0];
                P7.y=P[6][1];

                P8.x=P[7][0];
                P8.y=P[7][1]; 


                if(P1.x==0 && P1.y==0 && P2.x==0 && P2.y==0 && P3.x==0 && P3.y==0 && P4.x==0 && P4.y==0 && P5.x==0 && P5.y==0 && P6.x==0 && P6.y==0 && P7.x==0 && P7.y==0 && P8.x==0 && P8.y==0)
                continue;



              // Downscaling the Points Here

                P1=downscale_point(P1);
                P2=downscale_point(P2);
                P3=downscale_point(P3);
                P4=downscale_point(P4);
                P5=downscale_point(P5);
                P6=downscale_point(P6);
                P7=downscale_point(P7);
                P8=downscale_point(P8);
              
             // Perform Inverse Perspective Transformation (From 2D to 3D)

               P1=InvPerspTransf(depth_mat1,P1);
               P2=InvPerspTransf(depth_mat1,P2);
               P3=InvPerspTransf(depth_mat1,P3);
               P4=InvPerspTransf(depth_mat1,P4);
               P5=InvPerspTransf(depth_mat1,P5);
               P6=InvPerspTransf(depth_mat1,P6);
               P7=InvPerspTransf(depth_mat1,P7);
               P8=InvPerspTransf(depth_mat1,P8);


            //Putting the Data in the Message         

              lanemsg.P1.x=P1.x;
              lanemsg.P1.y=P1.y;
              lanemsg.P1.z=P1.z;

              lanemsg.P2.x=P2.x;
              lanemsg.P2.y=P2.y;
              lanemsg.P2.z=P2.z;

              lanemsg.P3.x=P3.x;
              lanemsg.P3.y=P3.y;
              lanemsg.P3.z=P3.z;

              lanemsg.P4.x=P4.x;
              lanemsg.P4.y=P4.y;
              lanemsg.P4.z=P4.z;

              lanemsg.P5.x=P5.x;
              lanemsg.P5.y=P5.y;
              lanemsg.P5.z=P5.z;

              lanemsg.P6.x=P6.x;
              lanemsg.P6.y=P6.y;
              lanemsg.P6.z=P6.z;

              lanemsg.P7.x=P7.x;
              lanemsg.P7.y=P7.y;
              lanemsg.P7.z=P7.z;

              lanemsg.P8.x=P8.x;
              lanemsg.P8.y=P8.y;
              lanemsg.P8.z=P8.z;

             
            // Publishing the Lane Message to Planner
              Lane_pub.publish(lanemsg);

              ROS_INFO("\n\nLane Message Published");
              ROS_INFO("Left Lane");
              ROS_INFO("P1= X=%f  Y=%f Z=%f",lanemsg.P1.x,lanemsg.P1.y,lanemsg.P1.z);
              ROS_INFO("P2= X=%f  Y=%f Z=%f",lanemsg.P2.x,lanemsg.P2.y,lanemsg.P2.z);
              ROS_INFO("P3= X=%f  Y=%f Z=%f",lanemsg.P3.x,lanemsg.P3.y,lanemsg.P3.z);
              ROS_INFO("P4= X=%f  Y=%f Z=%f",lanemsg.P4.x,lanemsg.P4.y,lanemsg.P4.z);

              ROS_INFO("Right Lane");
              ROS_INFO("P5= X=%f  Y=%f Z=%f",lanemsg.P5.x,lanemsg.P5.y,lanemsg.P5.z);
              ROS_INFO("P6= X=%f  Y=%f Z=%f",lanemsg.P6.x,lanemsg.P6.y,lanemsg.P6.z);
              ROS_INFO("P7= X=%f  Y=%f Z=%f",lanemsg.P7.x,lanemsg.P7.y,lanemsg.P7.z);
              ROS_INFO("P8= X=%f  Y=%f Z=%f",lanemsg.P8.x,lanemsg.P8.y,lanemsg.P8.z);


        }// if ends
  
      }  // while loop ends
    
}// main ends


//-------------------------------------Inverse Perspective Transformation------------------------------------------------------

CvPoint3D32f InvPerspTransf(cv::Mat Depth,CvPoint3D32f im_point)
{
   
    // Point w.r.t World Coordinate System
    
       CvPoint3D32f temp;
       CvPoint3D32f world_point;

    // Calculating the Encoded Depth Values. Currently using the blue channel only.

       temp.x = Depth.at<cv::Vec3b>((int)im_point.y,(int)im_point.x)[0];
        

   // Z coordinate of world Coordinate System. sd is scale factor

  // Detected Points after being Centered
       im_point.x=im_point.x-px;
       im_point.y=im_point.y-py;


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
