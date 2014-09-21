#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "SDL/SDL.h"

/*
 * A simple 'getting started' interface to the ARDrone, v0.2 
 * author: Tom Krajnik
 * The code is straightforward,
 * check out the CHeli class and main() to see 
 */
#include <stdlib.h>
#include "CHeli.h"
#include <unistd.h>
#include <stdio.h>
#include <iostream>

using namespace std;
using namespace cv;

bool stop = false;
CRawImage *image;
CHeli *heli;
float pitch, roll, yaw, height;
int hover;
// Joystick related
SDL_Joystick* m_joystick;
bool useJoystick;
int joypadRoll, joypadPitch, joypadVerticalSpeed, joypadYaw;
bool navigatedWithJoystick, joypadTakeOff, joypadLand, joypadHover;
bool is_binary = false;
bool enters = false;
int cambia = 0;
bool firstclick=true;

//Images
Mat freezeImage = Mat(240, 320, CV_8UC3);
Mat HSVImage = Mat(240, 320, CV_8UC3);
Mat YIQImage = Mat(240, 320, CV_8UC3);
Mat coloredImage = Mat(240, 320, CV_8UC3);
Mat binImage = Mat(240, 320, CV_8UC3);
Mat shownImage = Mat(240, 320, CV_8UC3);
// Destination OpenCV Mat	
Mat currentImage = Mat(240, 320, CV_8UC3);

//Scalar value
Vec3b intensity_min, intensity_max, intensity_maxHSV, intensity_minHSV;

// Here we will store points
vector<Point> points, point_max, point_min;
Point inicio, termina;

float hue, sat, val, b, g, r, y, i, q;
float YIQ[3];

//Matricez Histrogramas
int Blue[255], Green[255], Red[255];
int bli=0;
int blm=0;
int blf=0;
int gli=0;
int glm=0;
int glf=0;
int rli=0;
int rlm=0;
int rlf=0;
int hli=0;
int hlm=0;
int hlf=0;
int sli=0;
int slm=0;
int slf=0;
int vli=0;
int vlm=0;
int vlf=0;
int yli=0;
int ylm=0;
int ylf=0;
int ili=0;
int ilm=0;
int ilf=0;
int qli=0;
int qlm=0;
int qlf=0;


//Histograma R de RGB
void histograma(Mat& src)
{	
	int rojo, verde, azul;
	for (int y = 0; y < src.rows; ++y)
		for (int x = 0; x < src.cols; ++x){
			azul=src.at<Vec3b>(y,x)[0];
			verde=src.at<Vec3b>(y,x)[1];
			rojo=src.at<Vec3b>(y,x)[2];
			Blue[azul]+=1;
			Green[verde]+=1;
			Red[rojo]+=1;
		}

	// Draw the histograms for B, G and R
	int hist_w = 255; int hist_h = 150;
	
	Mat hist_B( hist_h, hist_w, CV_8UC3, Scalar(255,255,255) );
	Mat hist_G( hist_h, hist_w, CV_8UC3, Scalar(255,255,255) );	
	Mat hist_R( hist_h, hist_w, CV_8UC3, Scalar(255,255,255) );
	
	int maxb=0;
	int maxg=0;
	int maxr=0;

	for(int i=0; i<255; i++){
		if(Blue[i]>maxb)	
			maxb=Blue[i];
		if(Green[i]>maxg)		
			maxg=Green[i];
		if(Red[i]>maxr)		
			maxr=Red[i];	
	}
	if(cambia==0){
		for(int i=0; i<255; i++)
		{	
			Blue[i]=Blue[i]/(maxb/100);
			if(i>0)
      				line(hist_B, Point(i-1,135-Blue[i-1]), Point(i,135-Blue[i]), Scalar(255, 0, 0), 2, 8, 0  );
			else
				line(hist_B, Point(i,135), Point(i,135-Blue[i]), Scalar(255, 0, 0), 2, 8, 0  );
			line(hist_B, Point(i,150), Point(i,140), Scalar(i, 0, 0), 2, 8, 0  );
			line(hist_B, Point(bli,135), Point(bli,50), Scalar(0, 0, 0), 2, 8, 0  );
			line(hist_B, Point(blf,135), Point(blf,50), Scalar(100, 100, 100), 2, 8, 0  );
			line(hist_B, Point(blm,135), Point(blm,50), Scalar(255, 100, 255), 2, 8, 0  );
		}

		for(int i=0; i<255; i++)
		{	
			Green[i]=Green[i]/(maxg/100);
			if(i>0)
      				line(hist_G, Point(i-1,135-Green[i-1]), Point(i,135-Green[i]), Scalar(0, 255, 0), 2, 8, 0  );
			else
				line(hist_G, Point(i,135), Point(i,135-Green[i]), Scalar(0, 255, 0), 2, 8, 0  );
			line(hist_G, Point(i,150), Point(i,140), Scalar(0, i, 0), 2, 8, 0  );
			line(hist_G, Point(gli,135), Point(gli,50), Scalar(0, 0, 0), 2, 8, 0  );
			line(hist_G, Point(glf,135), Point(glf,50), Scalar(100, 100, 100), 2, 8, 0  );
			line(hist_G, Point(glm,135), Point(glm,50), Scalar(255, 100, 255), 2, 8, 0  );
		}

		for(int i=0; i<255; i++)
		{	
			Red[i]=Red[i]/(maxr/100);
			if(i>0)
      				line(hist_R, Point(i-1,135-Red[i-1]), Point(i,135-Red[i]), Scalar(0, 0, 255), 2, 8, 0  );
			else
				line(hist_R, Point(i,135), Point(i,135-Red[i]), Scalar(0, 0, 255), 2, 8, 0  );
			line(hist_R, Point(i,150), Point(i,140), Scalar(0, 0, i), 2, 8, 0  );
			line(hist_R, Point(rli,135), Point(rli,50), Scalar(0, 0, 0), 2, 8, 0  );
			line(hist_R, Point(rlf,135), Point(rlf,50), Scalar(100, 100, 100), 2, 8, 0  );
			line(hist_R, Point(rlm,135), Point(rlm,50), Scalar(255, 100, 255), 2, 8, 0  );
		}
	}
	else if(cambia==1){
		for(int i=0; i<255; i++)
		{	
			Blue[i]=Blue[i]/(maxb/100);
			if(i>0)
      				line(hist_B, Point(i-1,145-Blue[i-1]), Point(i,145-Blue[i]), Scalar(255, 0, 0), 2, 8, 0  );
			else
				line(hist_B, Point(i,145), Point(i,145-Blue[i]), Scalar(255, 0, 0), 2, 8, 0  );
			line(hist_B, Point(hli,135), Point(hli,50), Scalar(0, 0, 0), 2, 8, 0  );
			line(hist_B, Point(hlf,135), Point(hlf,50), Scalar(100, 100, 100), 2, 8, 0  );
			line(hist_B, Point(hlm,135), Point(hlm,50), Scalar(255, 100, 255), 2, 8, 0  );
		}

		for(int i=0; i<255; i++)
		{	
			Green[i]=Green[i]/(maxg/100);
			if(i>0)
      				line(hist_G, Point(i-1,145-Green[i-1]), Point(i,145-Green[i]), Scalar(0, 255, 0), 2, 8, 0  );
			else
				line(hist_G, Point(i,145), Point(i,145-Green[i]), Scalar(0, 255, 0), 2, 8, 0  );
			line(hist_G, Point(sli,135), Point(sli,50), Scalar(0, 0, 0), 2, 8, 0  );
			line(hist_G, Point(slf,135), Point(slf,50), Scalar(100, 100, 100), 2, 8, 0  );
			line(hist_G, Point(slm,135), Point(slm,50), Scalar(255, 100, 255), 2, 8, 0  );
		}

		for(int i=0; i<255; i++)
		{	
			Red[i]=Red[i]/(maxr/100);
			if(i>0)
      				line(hist_R, Point(i-1,145-Red[i-1]), Point(i,145-Red[i]), Scalar(0, 0, 255), 2, 8, 0  );
			else
				line(hist_R, Point(i,145), Point(i,145-Red[i]), Scalar(0, 0, 255), 2, 8, 0  );
			line(hist_R, Point(vli,135), Point(vli,50), Scalar(0, 0, 0), 2, 8, 0  );
			line(hist_R, Point(vlf,135), Point(vlf,50), Scalar(100, 100, 100), 2, 8, 0  );
			line(hist_R, Point(vlm,135), Point(vlm,50), Scalar(255, 100, 255), 2, 8, 0  );
		}
	}
	else if(cambia==2){
		for(int i=0; i<255; i++)
		{	
			Blue[i]=Blue[i]/(maxb/100);
			if(i>0)
      				line(hist_B, Point(i-1,145-Blue[i-1]), Point(i,145-Blue[i]), Scalar(255, 0, 0), 2, 8, 0  );
			else
				line(hist_B, Point(i,145), Point(i,145-Blue[i]), Scalar(255, 0, 0), 2, 8, 0  );
			line(hist_B, Point(yli,135), Point(yli,50), Scalar(0, 0, 0), 2, 8, 0  );
			line(hist_B, Point(ylf,135), Point(ylf,50), Scalar(100, 100, 100), 2, 8, 0  );
			line(hist_B, Point(ylm,135), Point(ylm,50), Scalar(255, 100, 255), 2, 8, 0  );
		}

		for(int i=0; i<255; i++)
		{	
			Green[i]=Green[i]/(maxg/100);
			if(i>0)
      				line(hist_G, Point(i-1,145-Green[i-1]), Point(i,145-Green[i]), Scalar(0, 255, 0), 2, 8, 0  );
			else
				line(hist_G, Point(i,145), Point(i,145-Green[i]), Scalar(0, 255, 0), 2, 8, 0  );
			line(hist_G, Point(ili,135), Point(ili,50), Scalar(0, 0, 0), 2, 8, 0  );
			line(hist_G, Point(ilf,135), Point(ilf,50), Scalar(100, 100, 100), 2, 8, 0  );
			line(hist_G, Point(ilm,135), Point(ilm,50), Scalar(255, 100, 255), 2, 8, 0  );
		}

		for(int i=0; i<255; i++)
		{	
			Red[i]=Red[i]/(maxr/100);
			if(i>0)
      				line(hist_R, Point(i-1,145-Red[i-1]), Point(i,145-Red[i]), Scalar(0, 0, 255), 2, 8, 0  );
			else
				line(hist_R, Point(i,145), Point(i,145-Red[i]), Scalar(0, 0, 255), 2, 8, 0  );
			line(hist_R, Point(qli,135), Point(qli,50), Scalar(0, 0, 0), 2, 8, 0  );
			line(hist_R, Point(qlf,135), Point(qlf,50), Scalar(100, 100, 100), 2, 8, 0  );
			line(hist_R, Point(qlm,135), Point(qlm,50), Scalar(255, 100, 255), 2, 8, 0  );
		}
	}
	if(cambia==0){
		namedWindow("RGB Histogram Blue", CV_WINDOW_AUTOSIZE );
  		imshow("RGB Histogram Blue", hist_B );	
		namedWindow("RGB Histogram Green", CV_WINDOW_AUTOSIZE );
	  	imshow("RGB Histogram Green", hist_G );
  		namedWindow("RGB Histogram Red", CV_WINDOW_AUTOSIZE );
  		imshow("RGB Histogram Red", hist_R );		
	}
	else if(cambia==1){
		namedWindow("HSV Histogram Hue", CV_WINDOW_AUTOSIZE );
  		imshow("HSV Histogram Hue", hist_B );	
		namedWindow("HSV Histogram Saturation", CV_WINDOW_AUTOSIZE );
	  	imshow("HSV Histogram Saturation", hist_G );
  		namedWindow("HSV Histogram Value", CV_WINDOW_AUTOSIZE );
  		imshow("HSV Histogram Value", hist_R );		
	}
	else if(cambia==2){
		namedWindow("YIQ Histogram Y", CV_WINDOW_AUTOSIZE );
  		imshow("YIQ Histogram Y", hist_B );	
		namedWindow("YIQ Histogram I", CV_WINDOW_AUTOSIZE );
	  	imshow("YIQ Histogram I", hist_G );
  		namedWindow("YIQ Histogram Q", CV_WINDOW_AUTOSIZE );
  		imshow("YIQ Histogram Q", hist_R );		
	}
	
}

void diblin(Point dibujo)
{
	int g, r;
	if(cambia==0)
	{	
		bli=freezeImage.at<Vec3b>(dibujo)[0];	
		g=freezeImage.at<Vec3b>(dibujo)[1];	
		r=freezeImage.at<Vec3b>(dibujo)[2];	
	}	
}
void diblinini(Point dibujo)
{
	int g, r;
	if(cambia==0)
	{
		bli=freezeImage.at<Vec3b>(dibujo)[0];	
		g=freezeImage.at<Vec3b>(dibujo)[1];	
		r=freezeImage.at<Vec3b>(dibujo)[2];	
	}	
}
void diblinter(Point dibujo)
{
	int g, r;
	if(cambia==0)
	{
		blf=freezeImage.at<Vec3b>(dibujo)[0];	
		g=freezeImage.at<Vec3b>(dibujo)[1];	
		r=freezeImage.at<Vec3b>(dibujo)[2];	
	}
}

void RGBtoHSV (Mat& src)
{
	
	cvtColor(src,HSVImage,CV_BGR2HSV);
	imshow("HSV Image", HSVImage);
}

void PixelRGBtoYIQ (Vec3b values)
{
        YIQ[0] = 0.299900 * values[2] + 0.587000 * values[1] + 0.114000 * values[0];
        YIQ[1] = 0.595716 * values[2] - 0.274453 * values[1] - 0.321264 * values[0];
        YIQ[2] = 0.211456 * values[2] - 0.522591 * values[1] + 0.311350 * values[0];
}

void RGBtoYIQ(const Mat& src)
{
	for (int y = 0; y < src.rows; ++y)
		for (int x = 0; x < src.cols; ++x){
			PixelRGBtoYIQ(src.at<Vec3b>(y, x));
			int by = (int)(0.5f + YIQ[0] * 255.0f);
			int bi = (int)(0.5f + (YIQ[1] + 0.5957f) * (255.0f/(0.5957f*2.0f)));
			int bq = (int)(0.5f + (YIQ[2] + 0.5226f) * (255.0f/(0.5226f*2.0f)));
			if (by > 255)
				by = 255;
			if (by < 0)
				by = 0;
			if (bi > 255)
				bi = 255;
			if (bi < 0)
				bi = 0;
			if (bq > 255)
				bq = 255;
			if (bq < 0)
				bq = 0;         
			YIQImage.at<Vec3b>(y,x)[0]=by;
			YIQImage.at<Vec3b>(y,x)[1]=bi;
			YIQImage.at<Vec3b>(y,x)[2]=bq;
		}
	imshow("YIQ Image", YIQImage);
}

//Show Histogram
/*void showHistogram(Mat& src, Mat& msk)
{
  Mat dst;

  /// Separate the image in 3 places ( B, G and R )
  vector<Mat> bgr_planes;
  split( src, bgr_planes );

  /// Establish the number of bins
  int histSize = 256;

  /// Set the ranges ( for B,G,R) )
  float range[] = { 0, 256 } ;
  const float* histRange = { range };

  bool uniform = true; bool accumulate = false;

  Mat b_hist, g_hist, r_hist;

	bitwise_not(msk, msk);

  /// Compute the histograms:
  calcHist( &bgr_planes[0], 1, 0, msk, b_hist, 1, &histSize, &histRange, uniform, accumulate );
  calcHist( &bgr_planes[1], 1, 0, msk, g_hist, 1, &histSize, &histRange, uniform, accumulate );
  calcHist( &bgr_planes[2], 1, 0, msk, r_hist, 1, &histSize, &histRange, uniform, accumulate );

  // Draw the histograms for B, G and R
  int hist_w = 512; int hist_h = 400;
  int bin_w = cvRound( (double) hist_w/histSize );

  Mat RGB_hist_R( hist_h, hist_w, CV_8UC3, Scalar(255,255,255) );
  Mat RGB_hist_G( hist_h, hist_w, CV_8UC3, Scalar(255,255,255) );
  Mat RGB_hist_B( hist_h, hist_w, CV_8UC3, Scalar(255,255,255) );

  /// Normalize the result to [ 0, histImage.rows ]
  normalize(b_hist, b_hist, 0, RGB_hist_B.rows, NORM_MINMAX, -1, Mat() );
  normalize(g_hist, g_hist, 0, RGB_hist_G.rows, NORM_MINMAX, -1, Mat() );
  normalize(r_hist, r_hist, 0, RGB_hist_R.rows, NORM_MINMAX, -1, Mat() );

  /// Draw for each channel
  for( int i = 1; i < histSize; i++ )
  {
      line( RGB_hist_B, Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,
                       Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
                       Scalar( 255, 0, 0), 2, 8, 0  );
      line( RGB_hist_G, Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ) ,
                       Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
                       Scalar( 0, 255, 0), 2, 8, 0  );
      line( RGB_hist_R, Point( bin_w*(i-1), hist_h - cvRound(r_hist.at<float>(i-1)) ) ,
                       Point( bin_w*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
                       Scalar( 0, 0, 255), 2, 8, 0  );
  }

  /// Display
  namedWindow("RGB Histogram Red", CV_WINDOW_AUTOSIZE );
  imshow("RGB Histogram Red", RGB_hist_R );
  
  namedWindow("RGB Histogram Green", CV_WINDOW_AUTOSIZE );
  imshow("RGB Histogram Green", RGB_hist_G );
  
  namedWindow("RGB Histogram Blue", CV_WINDOW_AUTOSIZE );
  imshow("RGB Histogram Blue", RGB_hist_B );
}*/

//Show Histogram HSV
void showHistogramHSV(Mat& src, Mat& msk)
{
  Mat dst;

  /// Separate the image in 3 places ( H, S and V )
  vector<Mat> hsv_planes;
  split( src, hsv_planes );

  /// Establish the number of bins
  int histSizeH = 179;
  int histSizeS = 255;
  int histSizeV = 255;

  /// Set the ranges ( for H,S,V) )
  float rangeH[] = { 0, 179 } ;
  const float* histRangeH = { rangeH };
  float rangeS[] = { 0, 255 } ;
  const float* histRangeS = { rangeS };
  float rangeV[] = { 0, 255 } ;
  const float* histRangeV = { rangeV };

  bool uniform = true; bool accumulate = false;

  Mat h_hist, s_hist, v_hist;
  
  bitwise_not(msk, msk);

  /// Compute the histograms:
  calcHist( &hsv_planes[0], 1, 0, msk, h_hist, 1, &histSizeH, &histRangeH, uniform, accumulate );
  calcHist( &hsv_planes[1], 1, 0, msk, s_hist, 1, &histSizeS, &histRangeS, uniform, accumulate );
  calcHist( &hsv_planes[2], 1, 0, msk, v_hist, 1, &histSizeV, &histRangeV, uniform, accumulate );

  // Draw the histograms for H, S and V
  int hist_w = 512; int hist_h = 400;
  int bin_wH = cvRound( (double) hist_w/histSizeH );
  int bin_wS = cvRound( (double) hist_w/histSizeS );
  int bin_wV = cvRound( (double) hist_w/histSizeV );

  Mat HSV_hist_H( hist_h, hist_w, CV_8UC3, Scalar( 255,255,255) );
  Mat HSV_hist_S( hist_h, hist_w, CV_8UC3, Scalar( 255,255,255) );
  Mat HSV_hist_V( hist_h, hist_w, CV_8UC3, Scalar( 255,255,255) );

  /// Normalize the result to [ 0, histImage.rows ]
  normalize(h_hist, h_hist, 0, HSV_hist_H.rows, NORM_MINMAX, -1, Mat() );
  normalize(s_hist, s_hist, 0, HSV_hist_S.rows, NORM_MINMAX, -1, Mat() );
  normalize(v_hist, v_hist, 0, HSV_hist_V.rows, NORM_MINMAX, -1, Mat() );

  /// Draw for each channel
  for( int i = 1; i < histSizeH; i++ )
  {
      line( HSV_hist_H, Point( bin_wH*(i-1), hist_h - cvRound(h_hist.at<float>(i-1)) ) ,
                       Point( bin_wH*(i), hist_h - cvRound(h_hist.at<float>(i)) ),
                       Scalar( 255, 0, 0), 2, 8, 0  );
  }
  /// Draw for each channel
  for( int i = 1; i < histSizeS; i++ )
  {
      line( HSV_hist_S, Point( bin_wS*(i-1), hist_h - cvRound(s_hist.at<float>(i-1)) ) ,
                       Point( bin_wS*(i), hist_h - cvRound(s_hist.at<float>(i)) ),
                       Scalar( 0, 255, 0), 2, 8, 0  );
      line( HSV_hist_V, Point( bin_wV*(i-1), hist_h - cvRound(v_hist.at<float>(i-1)) ) ,
                       Point( bin_wV*(i), hist_h - cvRound(v_hist.at<float>(i)) ),
                       Scalar( 0, 0, 255), 2, 8, 0  );
  }

  /// Display
  namedWindow("HSV Histogram Value", CV_WINDOW_AUTOSIZE );
  imshow("HSV Histogram Value", HSV_hist_V );
  
  namedWindow("HSV Histogram Saturation", CV_WINDOW_AUTOSIZE );
  imshow("HSV Histogram Saturation ", HSV_hist_S );
  
  namedWindow("HSV Histogram Hue", CV_WINDOW_AUTOSIZE );
  imshow("HSV Histogram Hue", HSV_hist_H );
}

// Convert CRawImage to Mat
void rawToMat( Mat &destImage, CRawImage* sourceImage)
{	
	uchar *pointerImage = destImage.ptr(0);
	
	for (int i = 0; i < 240*320; i++)
	{
		pointerImage[3*i] = sourceImage->data[3*i+2];
		pointerImage[3*i+1] = sourceImage->data[3*i+1];
		pointerImage[3*i+2] = sourceImage->data[3*i];
	} 
}


/*
 * This method flips horizontally the sourceImage into destinationImage. Because it uses 
 * "Mat::at" method, its performance is low (redundant memory access searching for pixels).
 */
void flipImageBasic(const Mat &sourceImage, Mat &destinationImage)
{
	if (destinationImage.empty())
		destinationImage = Mat(sourceImage.rows, sourceImage.cols, sourceImage.type());

	for (int y = 0; y < sourceImage.rows; ++y)
		for (int x = 0; x < sourceImage.cols / 2; ++x)
			for (int i = 0; i < sourceImage.channels(); ++i)
			{
				destinationImage.at<Vec3b>(y, x)[i] = sourceImage.at<Vec3b>(y, sourceImage.cols - 1 - x)[i];
				destinationImage.at<Vec3b>(y, sourceImage.cols - 1 - x)[i] = sourceImage.at<Vec3b>(y, x)[i];
			}
}

void deleteColor(const Mat &sourceImage)
{
	coloredImage = sourceImage.clone();
	Mat mask;
	if (cambia == 0){
	inRange(sourceImage, Scalar(b-20,g-20,r-20), Scalar(b+20, g+20, r+20), mask);
	bitwise_not(mask, mask);
	imshow("RGB Mask", mask);
	}
	else if (cambia == 1){
	inRange(sourceImage, Scalar(hue-10, sat-20, val-50), Scalar(hue+10, sat+20, val+50) , mask);
	bitwise_not(mask, mask);
	imshow("HSV Mask", mask);
	}
	else if (cambia == 2){
	inRange(sourceImage, Scalar(y-10, i-20, q-50), Scalar(y+10, i+20, q+50) , mask);
	bitwise_not(mask, mask);
	imshow("YIQ Mask", mask);
	}
	// Create a structuring element (SE)
    int morph_size = 2;
    Mat element = getStructuringElement( MORPH_RECT, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
    // Apply the specified morphology operation
    for (int i=1;i<2;i++)
    {   
    morphologyEx( mask, mask, MORPH_OPEN, element, Point(-1,-1), i );
	}
	//bitwise_and(coloredImage, Scalar(0,0,0), coloredImage, mask);
	coloredImage.setTo(Scalar(255,255,255), mask);
	if(cambia==0)
	{
		imshow("RGB Selection", coloredImage);
	}
	else if(cambia==1)
	imshow("HSV selection", coloredImage);
	else if(cambia==2)
	imshow("YIQ selection", coloredImage);
	//if (cambia == 0)
	//showHistogram(coloredImage);
	//else if (cambia == 1)
	//showHistogramHSV(coloredImage);
}

/* This is the callback that will only display mouse coordinates */
void mouseCoordinates(int event, int x, int y, int flags, void* param);

void choose() {
	if (cambia == 0){
		namedWindow("Frozen Image");
		setMouseCallback("Frozen Image", mouseCoordinates);
		cout << "RGB" << endl;
	}
	else if (cambia == 1) {
		namedWindow("HSV Image");
		setMouseCallback("HSV Image", mouseCoordinates);
		cout << "HSV" << endl;
	}
	else if (cambia == 2) {
		namedWindow("YIQ Image");
		setMouseCallback("YIQ Image", mouseCoordinates);
		cout << "YIQ" << endl;
	}
}

void recuadro(Point ini, Point fin)
{
	int minh=255;
	int mins=255;
	int minv=255;
	int minb=255;
	int ming=255;
	int minr=255;
	int miny=255;
	int mini=255;
	int minq=255;
	int maxh=0;
	int maxs=0;
	int maxv=0;
	int maxb=0;
	int maxg=0;
	int maxr=0;
	int maxy=0;
	int maxi=0;
	int maxq=0;
	if(cambia==0)
	{
		for (int y=ini.y; y<fin.y; y++)
			for (int x=ini.x; x<fin.x; x++)
			{
				if(freezeImage.at<Vec3b>(y,x)[0]>=maxb)
					maxb=freezeImage.at<Vec3b>(y,x)[0];
				if(freezeImage.at<Vec3b>(y,x)[1]>=maxg)
					maxg=freezeImage.at<Vec3b>(y,x)[1];
				if(freezeImage.at<Vec3b>(y,x)[2]>=maxr)
					maxr=freezeImage.at<Vec3b>(y,x)[2];
				if(freezeImage.at<Vec3b>(y,x)[0]<=minb)
					minb=freezeImage.at<Vec3b>(y,x)[0];
				if(freezeImage.at<Vec3b>(y,x)[1]<=ming)
					ming=freezeImage.at<Vec3b>(y,x)[1];
				if(freezeImage.at<Vec3b>(y,x)[2]<=minr)
					minr=freezeImage.at<Vec3b>(y,x)[2];
			}
	b=(maxb+minb)/2;
	g=(maxg+ming)/2;
	r=(maxr+minr)/2;
	deleteColor(freezeImage);
	}
	else if(cambia==1)
	{
		for (int y=ini.y; y<fin.y; y++)
			for (int x=ini.x; x<fin.x; x++)
			{
				if(HSVImage.at<Vec3b>(y,x)[0]>=maxh)
					maxh=HSVImage.at<Vec3b>(y,x)[0];
				if(HSVImage.at<Vec3b>(y,x)[1]>=maxs)
					maxs=HSVImage.at<Vec3b>(y,x)[1];
				if(HSVImage.at<Vec3b>(y,x)[2]>=maxv)
					maxv=HSVImage.at<Vec3b>(y,x)[2];
				if(HSVImage.at<Vec3b>(y,x)[0]<=minh)
					minh=HSVImage.at<Vec3b>(y,x)[0];
				if(HSVImage.at<Vec3b>(y,x)[1]<=mins)
					mins=HSVImage.at<Vec3b>(y,x)[1];
				if(HSVImage.at<Vec3b>(y,x)[2]<=minv)
					minv=HSVImage.at<Vec3b>(y,x)[2];
			}
	hue=(maxh+minh)/2;
	sat=(maxs+mins)/2;
	val=(maxv+minv)/2;
	deleteColor(HSVImage);
	}
	else if(cambia==2)
	{
		for (int y=ini.y; y<fin.y; y++)
			for (int x=ini.x; x<fin.x; x++)
			{
				if(YIQImage.at<Vec3b>(y,x)[0]>=maxy)
					maxy=YIQImage.at<Vec3b>(y,x)[0];
				if(YIQImage.at<Vec3b>(y,x)[1]>=maxi)
					maxi=YIQImage.at<Vec3b>(y,x)[1];
				if(YIQImage.at<Vec3b>(y,x)[2]>=maxq)
					maxq=YIQImage.at<Vec3b>(y,x)[2];
				if(YIQImage.at<Vec3b>(y,x)[0]<=miny)
					miny=YIQImage.at<Vec3b>(y,x)[0];
				if(YIQImage.at<Vec3b>(y,x)[1]<=mini)
					mini=YIQImage.at<Vec3b>(y,x)[1];
				if(YIQImage.at<Vec3b>(y,x)[2]<=minq)
					minq=YIQImage.at<Vec3b>(y,x)[2];
			}
	y=(maxy+miny)/2;
	i=(maxi+mini)/2;
	q=(maxq+minq)/2;
	deleteColor(YIQImage);
	}

}
void mouseCoordinates(int event, int x, int y, int flags, void* param)
{
	if(cambia==0)
	{
		 switch (event)
    		{   
			case CV_EVENT_LBUTTONDOWN:
		  		cout << "  Mouse X, Y: " << x << ", " << y << endl;
				inicio=Point(y,x);	
			    	bli=freezeImage.at<Vec3b>(y,x)[0];	
				gli=freezeImage.at<Vec3b>(y,x)[1];	
				rli=freezeImage.at<Vec3b>(y,x)[2];	
				histograma(freezeImage);	
				points.push_back(Point(x, y));
				break;
			case CV_EVENT_MOUSEMOVE:
				blm=freezeImage.at<Vec3b>(y,x)[0];
				glm=freezeImage.at<Vec3b>(y,x)[1];	
				rlm=freezeImage.at<Vec3b>(y,x)[2];
				histograma(freezeImage);		
				break;
			case CV_EVENT_LBUTTONUP:
				termina=Point(y,x);
				blf=freezeImage.at<Vec3b>(y,x)[0];
				glf=freezeImage.at<Vec3b>(y,x)[1];
				rlf=freezeImage.at<Vec3b>(y,x)[2];
				histograma(freezeImage);	
				recuadro(inicio,termina);
				break;
		}
	}
	else if(cambia==1)
	{
		 switch (event)
    		{   
			case CV_EVENT_LBUTTONDOWN:
		  		cout << "  Mouse X, Y: " << x << ", " << y << endl;
				inicio=Point(y,x);	
			    	hli=HSVImage.at<Vec3b>(y,x)[0];	
				sli=HSVImage.at<Vec3b>(y,x)[1];	
				vli=HSVImage.at<Vec3b>(y,x)[2];	
				histograma(HSVImage);	
				points.push_back(Point(x, y));
				break;
			case CV_EVENT_MOUSEMOVE:
				//hlm=HSVImage.at<Vec3b>(y,x)[0];
				//slm=HSVImage.at<Vec3b>(y,x)[1];	
				//vlm=HSVImage.at<Vec3b>(y,x)[2];
				//histograma(HSVImage);	
				break;
			case CV_EVENT_LBUTTONUP:
				termina=Point(y,x);
				hlf=HSVImage.at<Vec3b>(y,x)[0];
				slf=HSVImage.at<Vec3b>(y,x)[1];
				vlf=HSVImage.at<Vec3b>(y,x)[2];
				histograma(HSVImage);	
				recuadro(inicio,termina);
				break;
		}
	}
	else if(cambia==2)
	{
		 switch (event)
    		{   
			case CV_EVENT_LBUTTONDOWN:
		  		cout << "  Mouse X, Y: " << x << ", " << y << endl;
				inicio=Point(y,x);	
			    	yli=YIQImage.at<Vec3b>(y,x)[0];	
				ili=YIQImage.at<Vec3b>(y,x)[1];	
				qli=YIQImage.at<Vec3b>(y,x)[2];	
				histograma(YIQImage);	
				points.push_back(Point(x, y));
				break;
			case CV_EVENT_MOUSEMOVE:
				//ylm=YIQImage.at<Vec3b>(y,x)[0];
				//ilm=YIQImage.at<Vec3b>(y,x)[1];	
				//qlm=YIQImage.at<Vec3b>(y,x)[2];
				//histograma(YIQImage);		
				break;
			case CV_EVENT_LBUTTONUP:
				termina=Point(y,x);
				ylf=YIQImage.at<Vec3b>(y,x)[0];
				ilf=YIQImage.at<Vec3b>(y,x)[1];
				qlf=YIQImage.at<Vec3b>(y,x)[2];
				histograma(YIQImage);	
				recuadro(inicio,termina);
				break;
		}
	}
}

int main(int argc,char* argv[])
{
	//establishing connection with the quadcopter
	heli = new CHeli();
	
	//this class holds the image from the drone	
	image = new CRawImage(320,240);
	
	// Initial values for control	
    pitch = roll = yaw = height = 0.0;
    joypadPitch = joypadRoll = joypadYaw = joypadVerticalSpeed = 0.0;

	// Show it	
	//imshow("ParrotCam", currentImage);
	// Image to use for the mirror
	Mat mirrorImage = Mat(240, 320, CV_8UC3);
	
	/* First, open camera device */
	VideoCapture camera;
    camera.open(0);
	
	/* Create images where captured and transformed frames are going to be stored */
	//Mat currentImage;

    /* Create main OpenCV window to attach callbacks */
    choose();
    

    // Initialize joystick
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK);
    useJoystick = SDL_NumJoysticks() > 0;
    if (useJoystick)
    {
        SDL_JoystickClose(m_joystick);
        m_joystick = SDL_JoystickOpen(0);
    }

    while (stop == false)
    {

        // Clear the console
        printf("\033[2J\033[1;1H");

        if (useJoystick)
        {
            SDL_Event event;
            SDL_PollEvent(&event);

            joypadRoll = SDL_JoystickGetAxis(m_joystick, 2);
            joypadPitch = SDL_JoystickGetAxis(m_joystick, 3);
            joypadVerticalSpeed = SDL_JoystickGetAxis(m_joystick, 1);
            joypadYaw = SDL_JoystickGetAxis(m_joystick, 0);
            joypadTakeOff = SDL_JoystickGetButton(m_joystick, 1);
            joypadLand = SDL_JoystickGetButton(m_joystick, 2);
            joypadHover = SDL_JoystickGetButton(m_joystick, 0);
        }

        // prints the drone telemetric data, helidata struct contains drone angles, speeds and battery status
        printf("===================== Parrot Basic Example =====================\n\n");
        fprintf(stdout, "Angles  : %.2lf %.2lf %.2lf \n", helidata.phi, helidata.psi, helidata.theta);
        fprintf(stdout, "Speeds  : %.2lf %.2lf %.2lf \n", helidata.vx, helidata.vy, helidata.vz);
        fprintf(stdout, "Battery : %.0lf \n", helidata.battery);
        fprintf(stdout, "Hover   : %d \n", hover);
        fprintf(stdout, "Joypad  : %d \n", useJoystick ? 1 : 0);
        fprintf(stdout, "  Roll    : %d \n", joypadRoll);
        fprintf(stdout, "  Pitch   : %d \n", joypadPitch);
        fprintf(stdout, "  Yaw     : %d \n", joypadYaw);
        fprintf(stdout, "  V.S.    : %d \n", joypadVerticalSpeed);
        fprintf(stdout, "  TakeOff : %d \n", joypadTakeOff);
        fprintf(stdout, "  Land    : %d \n", joypadLand);
        fprintf(stdout, "Navigating with Joystick: %d \n", navigatedWithJoystick ? 1 : 0);

		//image is captured
		//heli->renewImage(image);

		// Copy to OpenCV Mat
		//rawToMat(currentImage, image);
		//imshow("Image", currentImage);

while (true)
	{
		/* Obtain a new frame from camera */
		//camera >> currentImage;
		//image is captured
		heli->renewImage(image);

		// Copy to OpenCV Mat
		rawToMat(currentImage, image);
		//imshow("Image", currentImage);

		if (shownImage.data) 
		{           

           		 /* Show image */
            		imshow("Image", currentImage);		
            
            		/*Show frozen image */
			imshow("Frozen Image", shownImage);
			
            

			/* If 'x' is pressed, exit program */
			if (waitKey(3) == 'x')
				break;
		}
		else
		{
			cout << "No image data.. " << endl;
		}
		
		/*RGB Image*/
		if (waitKey(4) == 's') {
            		freezeImage = currentImage.clone();
            		shownImage = freezeImage.clone();
            		cambia = 0;
			histograma(freezeImage);
            		choose();
		}
        	if (waitKey(2) == 'h') {
			RGBtoHSV(freezeImage);
			cambia = 1;
			histograma(HSVImage);
			choose();
		}
		
		if (waitKey(2)=='y'){
			RGBtoYIQ(freezeImage);
			cambia=2;
			histograma(YIQImage);
			choose();
		}
		if (waitKey(1) == 'c'){
			if (cambia==0){
				cambia=1;
				cout<<cambia;
				choose();
			}
			else if (cambia == 1){
				cambia = 2; 
				cout << cambia; 
				choose();
			}
			else if(cambia==2){
				cambia = 0; 
				cout << cambia; 
				choose();
			}
		}
        
        if (waitKey(3) == 'b') {
			if (is_binary == false){
				cvtColor(freezeImage, binImage, CV_RGB2GRAY);
				//threshold(binImage, binImage, 127, 255, THRESH_BINARY);
				is_binary = true;
				shownImage = binImage.clone();
			}
			else {
				is_binary = false;
				shownImage = freezeImage.clone();
			}
		}
	}

		
		// Mirror the image
		//flipImageBasic(currentImage, mirrorImage);
		//imshow("MirrorCam", mirrorImage);

        char key = waitKey(5);
		switch (key) {
			case 'a': yaw = -20000.0; break;
			case 'd': yaw = 20000.0; break;
			case 'w': height = -20000.0; break;
			case 'p': height = 20000.0; break;
			case 'q': heli->takeoff(); break;
			case 'e': heli->land(); break;
			case 'z': heli->switchCamera(0); break;
			case 'r': heli->switchCamera(1); break;
			case 'n': heli->switchCamera(2); break;
			case 'v': heli->switchCamera(3); break;
			case 'j': roll = -20000.0; break;
			case 'l': roll = 20000.0; break;
			case 'i': pitch = -20000.0; break;
			case 'k': pitch = 20000.0; break;
            case 'u': hover = (hover + 1) % 2; break;
            case 27: stop = true; break;
            default: pitch = roll = yaw = height = 0.0;
		}

        if (joypadTakeOff) {
            heli->takeoff();
        }	
        if (joypadLand) {
            heli->land();
        }
        hover = joypadHover ? 1 : 0;

        //setting the drone angles
        if (joypadRoll != 0 || joypadPitch != 0 || joypadVerticalSpeed != 0 || joypadYaw != 0)
        {
            heli->setAngles(joypadPitch, joypadRoll, joypadYaw, joypadVerticalSpeed, hover);
            navigatedWithJoystick = true;
        }
        else
        {
            heli->setAngles(pitch, roll, yaw, height, hover);
            navigatedWithJoystick = false;
        }

        usleep(15000);
	}
	
	
	heli->land();
    SDL_JoystickClose(m_joystick);
    delete heli;
	delete image;
	return 0;
}
