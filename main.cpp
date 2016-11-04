/*
name: main.cpp
date: 17.02.2016
author: Kevin Klein
description:
			This program will enable realtime video streaming of the local webcamera
			of a laptop to the iCub Simulator through yarp network ports. It should 
			process a number of various linear filters and detection methods in order
			to simulate visual attention with realtime image processing.
setup:
terminal 1: yarpserver (can be run globally)
terminal 2: iCub_SIM (can be run globally, but preferable to run in location of file)
terminal 3: cmake . (run in location of file)
			make
			./yarpy

IMPORTANT: assure that CMakeLists.txt file is in same location as this file, enabling 
		   compilation of yarp with openCV
*/

#include <stdio.h>
#include "movingrobot.h"
/* Get all OS and signal processing YARP classes */
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

// update rate at which eye-view will be processed in seconds
static const double updateRate = 0.02;

using namespace yarp::os;
using namespace yarp::sig;
using namespace cv;
// convert yarp image to opencv data type
cv::Mat ToMat(const ImageOf<PixelBgr>& imageIn)
{
	return cv::Mat((IplImage*)imageIn.getIplImage());
}

// convert opencv to yarp data type
ImageOf<PixelBgr> ToPixelBgr(const cv::Mat& imageIn)
{
	IplImage image(imageIn);
	ImageOf<PixelBgr> imageOut;
	imageOut.wrapIplImage(&image);
	return imageOut;
}

int main() {
    //stuff
  char* window_name = "Sobel Derative";
  int scale = 1;
  int delta = 0;
  int ddepth = CV_16S;
    
    Network yarp; // set up yarp
 	BufferedPort<ImageOf<PixelBgr> > imagePort; 
	BufferedPort<yarp::sig::Vector> targetPort;
    imagePort.open("/tutorial/image/in"); // give the port a name
	targetPort.open("/tutorial/target/out");
	Network::connect("/icubSim/cam/left","/tutorial/image/in"); 
	movingrobot *look = new movingrobot();

    // output video stream to screen
    BufferedPort<ImageOf<PixelBgr> > videoOut;
    videoOut.open("/icubsim-cw/out/video");
    Network::connect("/icubsim-cw/out/video", "/icubSim/texture/screen");
 
    // input eye-view image
    BufferedPort<ImageOf<PixelBgr> > camIn;
    camIn.open("/icubsim-cw/in/cam");
    Network::connect("/icubSim/cam/left", "/icubsim-cw/in/cam");
 
    // output processed eye-view image
    BufferedPort<ImageOf<PixelBgr> > camOut;
    camOut.open("/icubsim-cw/out/cam");
 
    // prepare camera and fallback image
    cv::Mat fallbackImage = cv::imread("fallback.jpg", CV_LOAD_IMAGE_COLOR);
    cv::VideoCapture camera;
    camera.open(0);
 
    int64 lastUpdateTick = 0;
    while (1)
    {
        // update
        int64 currentTick = cv::getTickCount();
        if ((double)(currentTick - lastUpdateTick) / cv::getTickFrequency() > updateRate)
        {
            // read eye-view image
            ImageOf<PixelBgr> *imageYarp = camIn.read();
            ImageOf<PixelBgr> *image = imagePort.read();
            if (imageYarp)
            {
                // process image
                cv::Mat image = ToMat(*imageYarp);
                
                //sobel Derative Straight from OpenCV Tutorial
                cv::Mat grad;
                cv::Mat gray;
				GaussianBlur( image, gray, Size(3,3), 0, 0, BORDER_DEFAULT );

				  /// Convert it to gray
				  cvtColor( image, gray, CV_BGR2GRAY );

				  /// Create window
				  namedWindow( window_name, CV_WINDOW_AUTOSIZE );

				  /// Generate grad_x and grad_y
				  Mat grad_x, grad_y;
				  Mat abs_grad_x, abs_grad_y;

				  /// Gradient X
				  //Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
				  Sobel( gray, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
				  convertScaleAbs( grad_x, abs_grad_x );

				  /// Gradient Y
				  //Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
				  Sobel( gray, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
				  convertScaleAbs( grad_y, abs_grad_y );

				  /// Total Gradient (approximate)
				  addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );

				  imshow( window_name, grad );

				  waitKey(1);
				  
				//Circle Detection
				vector<Vec3f> circles;

			  /// Apply the Hough Transform to find the circles
			  HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 1, gray.rows/8, 150, 50, 0, 0 );

			  /// Draw the circles detected
			  for( size_t i = 0; i < circles.size(); i++ )
			  {
				  Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
				  int radius = cvRound(circles[i][2]);
				  // circle center
				  circle( image, center, 3, Scalar(0,255,0), -1, 8, 0 );
				  // circle outline
				  circle( image, center, radius, Scalar(0,0,255), 3, 8, 0 );
			   }

			/// Show your results
			namedWindow( "Hough Circle Transform ", CV_WINDOW_AUTOSIZE );
			imshow( "Hough Circle Transform ", image );

			waitKey(1);
           //TIME FOR Robot to detect circles     
            int minRadius = 50;    
        	int maxRadius = 0;
			int maxAcceptedRadius = 120;
			int maxRadiusCircleIndex = 0;
		
			for (int i = 0; i < circles.size(); i++)
			{
				int radius = cvRound(circles[i][2]);
				if (radius > maxRadius && radius <= maxAcceptedRadius)
				{
					maxRadius = radius;
					maxRadiusCircleIndex = i;
				}
			}

			if (maxRadius > 0) {
				printf("The Best guess of finding the circle %g %g\n", circles[maxRadiusCircleIndex][0], circles[maxRadiusCircleIndex][1]);
				yarp::sig::Vector &target = targetPort.prepare();
				target.resize(3);
				target[0] = circles[maxRadiusCircleIndex][0];
				target[1] = circles[maxRadiusCircleIndex][1];
				target[2] = 1;
				targetPort.write();
			}else{
				yarp::sig::Vector& target = targetPort.prepare();
				target.resize(3);
				target[0] = 0;
				target[1] = 0;
				target[2] = 0;
				targetPort.write();
			}
                // write processed eye-view
                ImageOf<PixelBgr> &camOutObj = camOut.prepare();
                camOutObj.copy(ToPixelBgr(gray));
                camOut.write();
            }
 
            lastUpdateTick = currentTick;
        }
 
        // write new camera frame if present
        if (camera.isOpened())
        {
            ImageOf<PixelBgr> &videoOutObj = videoOut.prepare();
            cv::Mat frame;
            camera.read(frame);
            videoOutObj.copy(ToPixelBgr(frame));
            videoOut.write();
        }
        else if(fallbackImage.data)
        {
            // otherwise fall back to loaded image if present
            ImageOf<PixelBgr> &videoOutObj = videoOut.prepare();
            videoOutObj.copy(ToPixelBgr(fallbackImage));
            videoOut.write();
        }
			look->doLook();
    }
 
    camera.release();
 
    return 0;
}
