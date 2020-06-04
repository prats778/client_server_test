#include "ros/ros.h"
//#include "sim_img_proc/server.h"

#include "opencv2/imgproc.hpp"
#include "sim_img_proc/get_coordinates.h"
//#include "opencv2/videoio.hpp"
#include <iostream>
#include <limits.h>
#include <opencv2/opencv.hpp>
#include <string>
#include<cmath>

#define PI 3.14159265
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

//all variables which were made in image processor.h
Mat initial_frame, hsv_frame, gauss_frame, erosion_frame, dilation_frame,
      gray_frame, gray_frame_1, canny_frame,morph_frame, out_frame;

  Mat element;

int thresh_l_B = 0, thresh_l_G = 112, thresh_l_R = 43;
int thresh_h_B = 51, thresh_h_G = 255, thresh_h_R = 255;

int canny_low_thresh = 0, canny_ratio = 3, canny_kernel_size = 3;
int hl_thresh_detect = 50, hl_min_line_length = 80, hl_max_line_gap = 30;//change the values
//int threshold_th=24;
//int threshold_th2=95;
int morph_operator = 0, morph_elem = 0, morph_size = 1;
int x_order=2, d_depth=CV_16S;
double scale=1,delta=0;
int sobel_kernel_size=1;

int dilation_size=2;
  int x_parameter=50 , y_parameter=20;

  int speed_parameter=200;

  float angle;

  std::vector<Vec4i> lines;

  Vec4i line1,line2;

  Mat morph_op(Mat src);//this function is defined in the end

  float angle_finder(Vec4i line);//this function is defined in the end


bool get(
    sim_img_proc::get_coordinates::Request &req,
    sim_img_proc::get_coordinates::Response &res)

{
    float p=-1,q=-1,r=-1,s=-1;

    //std::cout << "Processing yellow flare " << std::endl;
    initial_frame= imread("/home/pratyush/Downloads/frame_15.png", IMREAD_COLOR);//load ur image(of single flare)
    cv::cvtColor(initial_frame,hsv_frame, CV_BGR2HSV);
    cv::blur( hsv_frame,gauss_frame, Size(3,3) );
    cv::inRange(gauss_frame,Scalar(thresh_l_B,thresh_l_G,thresh_l_R),Scalar(thresh_h_B,thresh_h_G,thresh_h_R),gray_frame);
    morph_frame=morph_op(gray_frame);
    // Canny detector
    cv::Canny( morph_frame,canny_frame,canny_low_thresh,canny_ratio,canny_kernel_size );

    cv::HoughLinesP(canny_frame,lines, 1, CV_PI/180,hl_thresh_detect,hl_min_line_length,hl_max_line_gap );
    out_frame=initial_frame.clone();
    for( size_t i = 0; i <lines.size(); i++ )
      {
            /*LOGIC-We first select a line from the total numbers of lines in the frame, then we select another line, we check if the angles of both the lines is between
                    80 and 100, then we find the differences between their y and x coordinates, if the diff of y coordinates is nearly the same and x coordinates diff is
                    some parameter, then we say that the flare is detected*/


            line1=lines[i];
            for(size_t j=i+1;j<lines.size();j++)
            {
              line2=lines[j];
              float angle1=angle_finder(line1);
              float angle2=angle_finder(line2);
              if(abs(abs(line1[3]-line1[1]) - abs(line2[3]-line2[1])) < y_parameter && (angle1 >=80 && angle1 <= 100) && (angle2 >=80 && angle2 <= 100) && abs(line1[0] - line2[0]) < x_parameter)
              {
                cv::line(out_frame,cv::Point(line1[0],line1[1]),cv::Point(line1[2],line1[3]),Scalar(0,0,255),3,8);
                cv::line(out_frame,cv::Point(line2[0],line2[1]),cv::Point(line2[2],line2[3]),Scalar(0,0,255),3,8);
                p=min(min(line1[0],line1[2]),min(line2[0],line2[2]));
                q=min(min(line1[1],line1[3]),min(line2[1],line2[3]));
                r=max(max(line1[0],line1[2]),max(line2[0],line2[2]));
                s=max(max(line1[1],line1[3]),max(line2[1],line2[3]));
                cv::rectangle(out_frame,cv::Point(p,q),cv::Point(r,s),Scalar(255,0,0),3,8,0);
              //}
            }
          }
            //line( out_frame, Point(lines[i][0], lines[i][1]),Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 );

      }
    if(req.dummy==1)
    {
      res.x[0]=p;
      res.x[1]=q;
      res.y[0]=r;
      res.y[1]=s;
      std::cout<<"Coordinates stored in the response"<<std::endl;
    }

  return true;
}
//main function
int main(int argc, char **argv)
{
  ros::init(argc, argv, "server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("get_coordinates", get(request,result));
  ROS_INFO("Ready detect yellow flare");
  ros::spin();

  return 0;
}
//functions

Mat morph_op(Mat src)
{

   Mat dest;
   int operation = morph_operator + 2;

   Mat element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );

     /// Apply the specified morphology operation
   morphologyEx( src, dest, operation, element );
   return dest;

}
float angle_finder(Vec4i line)
{
  float angle;
  if(line[2] - line[0]==0)
        angle=90;
      else
      {
        angle=atan((line[3]-line[1])/(line[2]-line[0]));
        angle=angle*180/PI;
      }
      return angle;
}

