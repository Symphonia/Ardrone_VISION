#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>     //make sure to include the relevant headerfiles
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <cvaux.h>
#include <math.h>
#include <cxcore.h>
#include "turtlesim/Velocity.h"
#include "turtlesim/Pose.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>

using namespace std;
using namespace cv;

int lowerH = 0;
int lowerS = 0;
int lowerV = 0;
static const char WINDOW[] = "Image window";

int upperH = 180;
int upperS = 256;
int upperV = 256;

int minRadius = 0;
int maxRadius = 200;

bool circleTrackOn = false;

//create variables for reading in pid configurations from text file
float PID[9];
float a;
//char array that contains the location of the pid.txt, have to use full path, cannot use ~/ on linux
char pidFileLoc[] = "/home/icarus/Desktop/PID.txt";



namespace enc = sensor_msgs::image_encodings;

//left click on the video window to enable circle tracking
//right click on the video window to disable circle tracking
void mouseHandler(int event, int x, int y, int flag, void* param)
  {
  	if(event == CV_EVENT_LBUTTONDOWN)
  	{
  		circleTrackOn = true;
  	}
  	if(event == CV_EVENT_RBUTTONDOWN)
  	{
  		circleTrackOn = false;
  	}
  }
 
class ImageConverter
{
  ros::NodeHandle nh_;
  ros::NodeHandle n;
  ros::Publisher pub ;
  ros::Publisher pub1;
  ros::Publisher tog;
  image_transport::ImageTransport it_;    
  image_transport::Subscriber image_sub_; //image subscriber 
  image_transport::Publisher image_pub_; //image publisher(we subscribe to ardrone image_raw)
  std_msgs::String msg;
public:
  ImageConverter()
    : it_(nh_)
  {

      pub= n.advertise<turtlesim::Velocity>("/drocanny/vanishing_points", 500);//
      pub1= n.advertise<turtlesim::Pose>("/drone/walldis", 500);
      image_sub_ = it_.subscribe("/ardrone/image_raw", 1, &ImageConverter::imageCb, this);
      image_pub_= it_.advertise("/arcv/Image",1);    
  }
 
  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
  }
 
  //function for getting pid values from pid.txt
  void getPidValue()
  {
    ifstream file;
    file.open(pidFileLoc);
    int i = 0;
    if(!file)
    {
      printf("no file found!");
    }
    if(file.is_open())
    {
      while((file>>a)&&(i<9))
      {
        PID[i] = a;
        i++;
      }
    }
	file.close();
  }


  //creates the trackbars for configuration of the HSV and detected circle radius
  void setwindowSettings()
  {
  	cvNamedWindow("Video");
  	cvNamedWindow("Configuration");

  	cvCreateTrackbar("LowerH","Configuration",&lowerH, 180, NULL);
  	cvCreateTrackbar("UpperH", "Configuration",&upperH, 180, NULL);
  	cvCreateTrackbar("LowerS", "Configuration", &lowerS, 256, NULL);
  	cvCreateTrackbar("UpperS", "Configuration", &upperS, 256, NULL);
  	cvCreateTrackbar("lowerV", "Configuration", &lowerV, 256, NULL);
  	cvCreateTrackbar("upperV", "Configuration", &upperV, 256, NULL);
    cvCreateTrackbar("minRadius","Configuration",&minRadius,200,NULL);
    cvCreateTrackbar("maxRadius","Configuration",&maxRadius,200,NULL);
  }

  //function for the filteration of colors not within the defected hsv range
  IplImage* GetThresholdedImage(IplImage* imgHSV)
  {
  	IplImage* imgThresh = cvCreateImage(cvGetSize(imgHSV),IPL_DEPTH_8U,1);
  	cvInRangeS(imgHSV, cvScalar(lowerH,lowerS,lowerV), cvScalar(upperH,upperS,upperV), imgThresh);
  	return imgThresh;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
	getPidValue();
    //setting up the bridge to convert ros image to opencv image
  	sensor_msgs::CvBridge bridge;
  	IplImage* frame = bridge.imgMsgToCv(msg,"bgr8");

  	turtlesim::Velocity velMsg;
	turtlesim::Pose dist;

    //trackbar
    setwindowSettings();
    //mouse listener
    cvSetMouseCallback("Video", mouseHandler,0);

    //creating memory storage of the circle sequence elements
  	CvMemStorage* storage = cvCreateMemStorage(0);

    //create an iplimage variable that will store the HSV converted images
  	IplImage* imgHSV = cvCreateImage(cvGetSize(frame),IPL_DEPTH_8U,3);
    //changes converts the bgr image to the HSV scale
  	cvCvtColor(frame, imgHSV,CV_BGR2HSV);
    //call the filtering fuction to get a black and white image that only contains the detected object
  	IplImage* imgThreshed = GetThresholdedImage(imgHSV);
    //uses the gaussian blur to smooth out the pixels so that it is easier to pick up the circle
  	cvSmooth(imgThreshed,imgThreshed,CV_GAUSSIAN, 15,15);
  	CvSize size = cvGetSize(imgThreshed);


	float x_pos = 0;
	float z_pos = 0;
	float r_size = 0;


    //if circleTrackOn is true, then start the circle tracking routine
  	if (circleTrackOn)
  	{
     	CvSeq* circles = cvHoughCircles(imgThreshed,storage,CV_HOUGH_GRADIENT,2, size.height/4,100,40,minRadius,maxRadius);
     	for (int i = 0; i < circles->total; i++)
     	{
     	float* circle = (float*)cvGetSeqElem(circles,i);
        cv::Point center(cvRound(circle[0]),cvRound(circle[1]));
        int radius = cvRound(circle[2]);
        cvCircle(frame,center, 3, CV_RGB(0,255,0), -1, 8,0);
        cvCircle(frame,center, radius, CV_RGB(0,0,255), 3, 8, 0);
 	//printf("x %f, y %f, r %f",circle[0],circle[1],circle[2]);
	//printf("\n");
	x_pos = circle[0];
	z_pos = circle[1];
	r_size = circle[2];
	//printf("x %f, z %f, r %f",x_pos,z_pos,r_size);
	//printf("\n");
       	}
	//0 = x, 1 = z, 2 = r
	float thresh_x = 320.0; //-320 to 320
	float thresh_z = 240.0; //-240 to 240
	float thresh_r = 70.0;  //0 to 360
	float lastError[3];
	float error[3];
	float integral[3];
	float derivative[3];
	float output[3];  //must be between -1 to 1
	float kpx = PID[0]; //.5
	float kix = PID[1]; //0
	float kdx = PID[2];
	float kpzrot = PID[3]; //.6
	float kizrot = PID[4]; //.01
	float kdzrot = PID[5];
	float kpzlin = PID[6]; //.65
	float kizlin =PID[7]; //.1
	float kdzlin = PID[8];
	error[0] = x_pos - thresh_x;
	error[1] = z_pos - thresh_z;
	error[2] = r_size - thresh_r;
	integral[0] = .9*(error[0] + integral[0]);
	integral[1] = .9*(error[1] + integral[1]);
	integral[2] = .9*(error[2] + integral[2]);
	derivative[0] = error[0] - lastError[0];
	derivative[1] = error[1] - lastError[1];
	derivative[2] = error[2] - lastError[2];
	output[0] = (kpzrot*error[0] + kizrot*integral[0] + kdzrot*derivative[0])/320.0; //normalizes ranges from -1 to 1
	output[1] = ((kpzlin*error[1] + kizlin*integral[1] + kdzlin*derivative[1])/240.0);
	output[2] = ((kpx*error[2] + kix*integral[2] + kdx*derivative[2])/200.0);
	lastError[0] = error[0];
	lastError[1] = error[1];
	lastError[2] = error[2];
	if(circles->total !=1) //fail safe, if the copter detects either 0 or more than 1 circle then the copter will just hover, so it doesnt go ballistic
	{
		output[0] = 0.0;
		output[1] = 0.0;
		output[2] = 0.0;		
		velMsg.angular = output[0]; //rotation   angularz  theta
		velMsg.linear = output[1]; //altitude adjustment   linearz  y
		dist.x = output[2]; //forward/backward    linearx  x
		printf("\nX::Z::R::%f:%f:%f",output[0],output[1],output[2]);
		//printf("\nerrorX::errorZ::errrorR::%f:%f:%f",error[0],error[1],error[2]);
		pub.publish(velMsg);
        	pub1.publish(dist);
	}
	else
	{
		velMsg.angular = output[0]; //rotation   angularz  theta
		velMsg.linear = output[1]; //altitude adjustment   linearz  y
		dist.x = output[2]; //forward/backward    linearx  x
		printf("\nX::Z::R::%f:%f:%f",output[0],output[1],output[2]);
		//printf("\nerrorX::errorZ::errrorR::%f:%f:%f",error[0],error[1],error[2]);
		pub.publish(velMsg);
        	pub1.publish(dist);	
	}
  	}

    cvShowImage("Object", imgThreshed);
    cvShowImage("Video", frame);
    //cvShowImage("Configuration",imgHSV); //this attachs a video window within trackbar window and displays the hsv video
    cv::waitKey(3);
    sensor_msgs::ImagePtr out = sensor_msgs::CvBridge::cvToImgMsg(frame,"rgb8");
    image_pub_.publish(out);
    cvClearMemStorage(storage);
    cvReleaseMemStorage(&storage);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc,argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

