#include <iostream>

#include<opencv/cvaux.h>
#include<opencv/highgui.h>
#include<opencv/cxcore.h>

#include<stdio.h>
#include<stdlib.h>

#define WEBCAM
//#define KINECT
//#define ARDUINO


#ifdef KINECT
#include "libfreenect/libfreenect_cv.h"
#endif

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <math.h>

using namespace cv;

void maskRGBImage(Mat& in_image, unsigned char lut[256][256][256], Mat& out_image){
	Vec3b *in_p;
	Vec3b *out_p;
	for(int ii=0; ii < in_image.rows; ii++){
		in_p = in_image.ptr<Vec3b>(ii);
		out_p = out_image.ptr<Vec3b>(ii);
		for(int jj=0; jj < in_image.cols; jj++){
			out_p[jj] = lut[in_p[jj][0]][in_p[jj][1]][in_p[jj][2]];
		}
	}
}

void initializeRGCLUT(double o_min, double o_max, double s_min, double s_max, unsigned char lut[256][256][256]){
	double one_third = (double)(1)/3;

	//Following formula described at: www.ikaros-project.org/articles/2007/colortracking/
	for(int R=0; R < 256; R++){
		for(int G=0; G < 256; G++){
			for(int B=0; B < 256; B++){
				//Compute chromaticity value for this color
				double r = (double)(R)/(R+G+B)/255;
				double g = (double)(G)/(R+G+B)/255;
				
				//Compute magnitude and phase in RG chromaticity plane
				double o = atan((r-one_third)/(g-one_third));
				double s = hypot(r-one_third, g-one_third);

				//Make sure the color falls within the specified bounds
				if((o >= o_min) && (o <= o_max) && (s >= s_min) && (s <= s_max)){
					lut[R][G][B] = 255;
				} else {
					lut[R][G][B] = 0;
				}
			}
		}
	}
}

int
set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // ignore break signal
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void
set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf ("error %d setting term attributes", errno);
}


///////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
	double o_min = -1.2;
	double o_max = -0.9;
	double s_min = 0.2;
	double s_max = 0.5;
	unsigned char mask[256][256][256];
	initializeRGCLUT(o_min, o_max, s_min, s_max, mask);
#ifdef ARDUINO
	char *portname = "/dev/ttyACM0";
	char outputChars[2];
	outputChars[1] = 0;

	int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0)
	{
	        printf ("error %d opening %s: %s", errno, portname, strerror (errno));
	        return 1;
	}
	
	set_interface_attribs (fd, B9600, 0);  // set speed to 115,200 bps, 8n1 (no parity)
	set_blocking (fd, 0);                // set no blocking
#endif

	// Setup OpenCV variables and structures
	Mat p_imgOriginal;			// pointer to an image structure, this will be the input image from webcam
	Mat p_imgResized;			// pointer to an image structure, this will be the processed image
	Mat p_imgProcessed;			// pointer to an image structure, this will be the processed image
	Mat p_imgHSV;                 // pointer to an image structure, this will hold the image after the color has been changed from RGB to HSV
										// IPL is short for Intel Image Processing Library, this is the structure used in OpenCV 1.x to work with images
	vector<Vec3f> circles;
	
	Vec3i p_fltXYRadius;				// pointer to a 3 element array of floats
										// [0] => x position of detected object
										// [1] => y position of detected object
									// [2] => radius of detected object

	int i;								// loop counter
	char charCheckForEscKey;			// char for checking key press (Esc exits program)
#ifdef WEBCAM
	VideoCapture p_capWebcam(1);	// 0 => use 1st webcam, may have to change to a different number if you have multiple cameras

	if(!p_capWebcam.isOpened()) {			// if capture was not successful . . .
		printf("error: webcam not found \n");	// error message to standard out . . .
		getchar();								// getchar() to pause for user see message . . .
		return(-1);								// exit program
	}
        double cam_w = p_capWebcam.get(CV_CAP_PROP_FRAME_WIDTH);
        double cam_h = p_capWebcam.get(CV_CAP_PROP_FRAME_HEIGHT);
        double fps = 30;//cvGetCaptureProperty(p_capWebcam, CV_CAP_PROP_FPS);    
        printf("* Capture properties: %f x %f, %f fps\n", cam_w, cam_h, fps); 
    
        namedWindow("Grayscale video", CV_WINDOW_AUTOSIZE);
    
        VideoWriter writer("out.avi", CV_FOURCC('x','v','i','d'), 25, cvSize((int)cam_w,(int)cam_h) );
        if (!writer.isOpened())
        {
            printf("!!! ERROR: unable to open VideoWriter\n");
            return -1;
        }

#endif
											            // declare 2 windows
	namedWindow("Original", CV_WINDOW_AUTOSIZE);		// original image from webcam
	namedWindow("Processed", CV_WINDOW_AUTOSIZE);		// the processed image we will use for detecting circles

	// Variables for Arduino Control
	int servoPosition = 90;
	int servoPositionV = 90;
	int servoOrientation = 0;

	// Main program loop
	while(1) {								// for each frame . . .
#ifdef KINECT
		p_imgOriginal = freenect_sync_get_rgb_cv(0);
		cvtColor(p_imgOriginal, p_imgOriginal, CV_RGB2BGR);
#endif
#ifdef WEBCAM
		bool success = p_capWebcam.read(p_imgOriginal);		// get frame from webcam
		if(!success){
			printf("!!! ERROR: Cannot read image from webcam\n");
			break;
		}

                writer.write(p_imgOriginal);
#endif
		resize(p_imgOriginal, p_imgResized, Size(640.480));

		maskRGBImage(p_imgResized, mask, p_imgProcessed);
										// smooth the processed image, this will make it easier for the next function to pick out the circles
		GaussianBlur(p_imgProcessed,		// function input
				 p_imgProcessed,		// function output
				 Size(0,0),			// use Gaussian filter (average nearby pixels, with closest pixels weighted more)
				 9,						// smoothing filter window width
				 9);					// smoothing filter window height

													// fill sequential structure with all circles in processed image
		HoughCircles(p_imgProcessed,		// input image, nothe that this has to be grayscale (no color)
		             circles,			// provide function with memory storage, makes function return a pointer to a CvSeq
		             CV_HOUGH_GRADIENT,	// two-pass algorithm for detecting circles, this is the only choice available
		             2,					// size of image / 2 = "accumulator resolution", i.e. accum = res = size of image / 2
		             p_imgProcessed->height / 4,	// min distance in pixels between the centers of the detected circles
		             100,						// high threshold of Canny edge detector, called by cvHoughCircles
		             50,						// low threshold of Canny edge detector, called by cvHoughCircles
		             10,	 //10					// min circle radius, in pixels
		             400);						// max circle radius, in pixels

		// Run this if the camera can see at least one circle
		for(i=0; i < circles.size(); i++) {		// for each element in sequential circles structure (i.e. for each object detected)

			p_fltXYRadius = circles[i];	// from the sequential structure, read the ith value into a pointer to a float

			printf("ball position x = %f, y = %f, r = %f \n", p_fltXYRadius[0],		// x position of center point of circle
															  p_fltXYRadius[1],		// y position of center point of circle
															  p_fltXYRadius[2]);	// radius of circle
#ifdef ARDUINO
			// Reset servo orientation as the camera now has focus of a circle
			// Servo orientation is important only when the camera doesn't see a circle
			servoOrientation = 0;

			// Check whether camera should turn to its left if the circle gets near the right end of the screen
			if (p_fltXYRadius[0] > 390)
			{
				outputChars[0] = 'l';
				write(fd, outputChars, strlen(outputChars));

				servoPosition+=1;

				if (servoPosition > 180)
					servoPosition = 180;
			}

			// Check whether camera should turn to its right if the circle gets near the left end of the screen
			if (p_fltXYRadius[0] < 250)
			{
				outputChars[0] = 'r';
				write(fd, outputChars, strlen(outputChars));

				servoPosition-=1;

				if (servoPosition < 0)
					servoPosition = 0;
			}

			if (p_fltXYRadius[1] > 280){
				outputChars[0] = 'd';
				write(fd, outputChars, strlen(outputChars));

				servoPositionV+=1;

				if (servoPositionV > 180)
					servoPositionV = 180;
			}

			if (p_fltXYRadius[1] < 200)
			{
				outputChars[0] = 'u';
				write(fd, outputChars, strlen(outputChars));

				servoPositionV-=1;

				if (servoPositionV < 0)
					servoPositionV = 0;
			}
#endif 
										// draw a small green circle at center of detected object
			circle(p_imgOriginal,										// draw on the original image
			       Point(cvRound(p_fltXYRadius[0]), cvRound(p_fltXYRadius[1])),		// center point of circle
			       3,													// 3 pixel radius of circle
			       Scalar(0,255,0),									// draw pure green
			       CV_FILLED);										// thickness, fill in the circle
			
										// draw a red circle around the detected object
			circle(p_imgOriginal,										// draw on the original image
			       Point(cvRound(p_fltXYRadius[0]), cvRound(p_fltXYRadius[1])),		// center point of circle
			       cvRound(p_fltXYRadius[2]),							// radius of circle in pixels
			       Scalar(255,0,0),									// draw pure red
			       3);												// thickness of circle in pixels
		}	// end for

		imshow("Original", p_imgOriginal);			// original image with detectec ball overlay
		imshow("Processed", p_imgProcessed);		// image after processing

		charCheckForEscKey = waitKey(10);				// delay (in ms), and get key press, if any
		if(charCheckForEscKey == 27) break;				// if Esc key (ASCII 27) was pressed, jump out of while loop
	}	// end while

#ifdef ARDUINO
	close(fd);
#endif

	// This closes the Serial Port
//DEBUG    CloseHandle(hSerial);

	return(0);
}
/*	while (cvWaitKey(10) < 0) {
                IplImage *image = freenect_sync_get_rgb_cv(0);
                if (!image) {
                    printf("Error: Kinect not connected?\n");
                    return -1;
                }
                cvCvtColor(image, image, CV_RGB2BGR);
                cvShowImage("RGB", image);
        }
        return 0;
}*/