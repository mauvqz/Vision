	#include <opencv2/imgproc/imgproc.hpp>
	#include <opencv2/core/core.hpp>
	#include <opencv2/highgui/highgui.hpp>
	#include "SDL/SDL.h"

	// I am making a second test
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
	int cambia = 0; //variable que selecciona con que tipo de imagen estamos trabajando, 0-RGB, 1-HSV, 2-YIQ
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

	// Here we will store points for the selection
	Point inicio, termina;

	float hue, sat, val, b, g, r, y, i, q;
	float YIQ[3], YIQprom[3];

	//Matricez Histrogramas
	int Blue[255], Green[255], Red[255];

	//Variables para los valores de intensidad con el ratón
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
		//Se obtiene el valor de cada pixel
		for (int y = 0; y < src.rows; ++y)
			for (int x = 0; x < src.cols; ++x){
				azul=src.at<Vec3b>(y,x)[0];
				verde=src.at<Vec3b>(y,x)[1];
				rojo=src.at<Vec3b>(y,x)[2];
				//El valor es la posición en el arreglo(0-255) y se incrementa
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
		
		//Se obtiene el valor maximo de R, G y B para más adelante hacerlos más chicos debido a que los valores eran muy grandes para mostrarlos
		for(int i=0; i<255; i++){
			if(Blue[i]>maxb)
				maxb=Blue[i];
			if(Green[i]>maxg)
				maxg=Green[i];
			if(Red[i]>maxr)
				maxr=Red[i];
		}
		
		//Se dibuja el histrograma en RGB
		if(cambia==0){
			for(int i=0; i<255; i++)
			{
				Blue[i]=Blue[i]/(maxb/100); //Reducir el tamaño de los valores del histograma
				if(i>0)
					line(hist_B, Point(i-1,135-Blue[i-1]), Point(i,135-Blue[i]), Scalar(255, 0, 0), 2, 8, 0  ); //Dibujar linea desde la coordenada del valor anterior al nuevo
				else
					line(hist_B, Point(i,135), Point(i,135-Blue[i]), Scalar(255, 0, 0), 2, 8, 0  ); //Dibuja la primer línea al primer punto
				
				line(hist_B, Point(i,150), Point(i, 140), Scalar(i, 0, 0), 2, 8, 0  ); //Muestra una barra con las diferentes intensidades debajo del histrograma
				line(hist_B, Point(bli,135), Point(bli,50), Scalar(0, 0, 0), 2, 8, 0  ); //Muestra una línea vertical en el valor del pixel en donde se dió clik
				line(hist_B, Point(blf,135), Point(blf,50), Scalar(100, 100, 100), 2, 8, 0  ); //Muestra una línea vertical en el valor del pixel en donde se soltó el botón del ratón
				line(hist_B, Point(blm,135), Point(blm,50), Scalar(255, 100, 255), 2, 8, 0  ); //Muestra una línea vertical en el valor del pixel por donde pasa el ratón
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
		
		//Ya no se dibuja la barra de intensidades
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
		
		//Crear las ventanas para cada histograma y los muestra
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

	//Transformar la imagen RGB a HSV y mostrarla
	void RGBtoHSV (Mat& src)
	{
		
		cvtColor(src,HSVImage,CV_BGR2HSV);
		imshow("HSV Image", HSVImage);
	}

	//Funciones que comvierten la imagen RGB a YIQ
	void PixelRGBtoYIQ (Vec3b values)
	{
		YIQ[0] = 0.299900 * values[2] + 0.587000 * values[1] + 0.114000 * values[0];
		YIQ[1] = 0.595716 * values[2] - 0.274453 * values[1] - 0.321264 * values[0];
		YIQ[2] = 0.211456 * values[2] - 0.522591 * values[1] + 0.311350 * values[0];
	}

	void RGBtoYIQprom ()
	{
		YIQprom[0] = 0.299900 * r + 0.587000 * g + 0.114000 * b;
		YIQprom[1] = 0.595716 * r - 0.274453 * g - 0.321264 * b;
		YIQprom[2] = 0.211456 * r - 0.522591 * g + 0.311350 * b;
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

	//Función que elimina los colores fuera del rango
	void deleteColor(const Mat &sourceImage)
	{
		coloredImage = sourceImage.clone();
		Mat mask;
		
		if (cambia == 0){
			inRange(sourceImage, Scalar(b-20,g-20,r-20), Scalar(b+20, g+20, r+20), mask); //Elimina los colores fuera del rango establecido
			bitwise_not(mask, mask); //Los negros los hace blancos y los blancos negros
			imshow("RGB Mask", mask);
		}
		else if (cambia == 1){
			inRange(sourceImage, Scalar(hue-10, sat-20, val-50), Scalar(hue+10, sat+20, val+50) , mask);
			bitwise_not(mask, mask);
			imshow("HSV Mask", mask);
		}
		//else if (cambia == 2){
			//inRange(sourceImage, Scalar(YIQprom[0]-0.1, YIQprom[1]-0.1, YIQprom[2]-0.1), Scalar(YIQprom[0]+0.1, YIQprom[1]+0.1, YIQprom[2]+0.1) , mask);
			//bitwise_not(mask, mask);
			//imshow("YIQ Mask", mask);
		//}
		
		//Realiza el procesamiento para cerrar espacios en la imagen blanco y negro
		// Create a structuring element (SE)
		int morph_size = 2;
		Mat element = getStructuringElement( MORPH_RECT, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
		// Apply the specified morphology operation
		for (int i=1;i<2;i++)
		{
			morphologyEx( mask, mask, MORPH_OPEN, element, Point(-1,-1), i );
		}
		
		//Aplica la máscara a la imagen congelada y vuelve blanco lo que no se encuentra en ella
		coloredImage.setTo(Scalar(255,255,255), mask);
		
		//Muestra la imagen filtrada dependiendo de la imagen a trabajar (cambia)
		if(cambia==0)
			imshow("RGB Selection", coloredImage);
		else if(cambia==1)
			imshow("HSV selection", coloredImage);
		//else if(cambia==2)
		 //   imshow("YIQ selection", coloredImage);
	}

	/* This is the callback that will only display mouse coordinates */
	void mouseCoordinates(int event, int x, int y, int flags, void* param);

	//Fucnión que hace saber al evento del ratón sobre que ventana trabajar
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

	//Función que toma el máximo y mínimo del recuadro seleccionado y regresa su promedio
	void recuadro(Point ini, Point fin)
	{
		int minh=255;
		int mins=255;
		int minv=255;
		int minb=255;
		int ming=255;
		int minr=255;
		int maxh=0;
		int maxs=0;
		int maxv=0;
		int maxb=0;
		int maxg=0;
		int maxr=0;
		
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
			
			RGBtoYIQprom();
			
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
		else if (cambia==2)
		{
			for (int y=ini.y; y<fin.y; y++)
				for (int x=ini.x; x<fin.x; x++)
				{
					PixelRGBtoYIQ(freezeImage.at<Vec3b>(y,x));
					if (!(YIQ[0] > (YIQprom[0]-0.1) && YIQ[0] < (YIQprom[0]+0.1) && 
						YIQ[1] > (YIQprom[1]-0.1) && YIQ[1] < (YIQprom[1]+0.1) &&
						YIQ[2] > (YIQprom[2]-0.1) && YIQ[2] < (YIQprom[2]+0.1)))
						freezeImage.at<Vec3b>(y,x) = 255;
				}
				
				imshow("YIQ selection", freezeImage);
		}
		
	}

	//Función que regresa posiciones de la posición del ratón
	void mouseCoordinates(int event, int x, int y, int flags, void* param)
	{
		if(cambia==0)
		{
			switch (event)
			{
				case CV_EVENT_LBUTTONDOWN: //Evento al dar click
					cout << "  Mouse X, Y: " << x << ", " << y << endl;
					inicio=Point(y,x);
					//Asigna los valores de intensidad a sus respectivos canales
					bli=freezeImage.at<Vec3b>(y,x)[0];
					gli=freezeImage.at<Vec3b>(y,x)[1];
					rli=freezeImage.at<Vec3b>(y,x)[2];
					histograma(freezeImage); //Dibuja histograma con los nuevos valores
					break;
				case CV_EVENT_MOUSEMOVE: //Evento al pasar el ratón sobre la image
					blm=freezeImage.at<Vec3b>(y,x)[0];
					glm=freezeImage.at<Vec3b>(y,x)[1];
					rlm=freezeImage.at<Vec3b>(y,x)[2];
					histograma(freezeImage);
					break;
				case CV_EVENT_LBUTTONUP: //Evento al soltar el botón
					termina=Point(y,x);
					blf=freezeImage.at<Vec3b>(y,x)[0];
					glf=freezeImage.at<Vec3b>(y,x)[1];
					rlf=freezeImage.at<Vec3b>(y,x)[2];
					histograma(freezeImage);
					recuadro(inicio,termina);
					break;
			}
		}
		
		//Aquí están comentados las instrucciones al pasar el ratón sobre la imagen ya que la computadora no podía con tantos procesos
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
		
		/* First, open camera device */
		VideoCapture camera;
		camera.open(0);
		
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
			
			
			while (true)
			{
				/* Obtain a new frame from camera */
				camera >> currentImage;
				//image is captured from Parrot
				//heli->renewImage(image);
				
				// Copy to OpenCV Mat para poder trabajar con la imagen del Parrot
				//rawToMat(currentImage, image);
				
				if (shownImage.data)
				{
					
					/* Show image */
					imshow("Image", currentImage);
					
					/*Show frozen image */
					imshow("Frozen Image", shownImage);
					
					
					
					/* If 'x' is pressed, exit program No funciona con el Parrot*/
					if (waitKey(3) == 'x')
						break;
				}
				else
				{
					cout << "No image data.. " << endl;
				}
				
				//Congela la imagen de video
				if (waitKey(4) == 's') {
					freezeImage = currentImage.clone();
					shownImage = freezeImage.clone();
					cambia = 0;
					histograma(freezeImage);
					choose();
				}
				
				//Cambia la imagen a HSV
				if (waitKey(2) == 'h') {
					RGBtoHSV(freezeImage);
					cambia = 1;
					histograma(HSVImage);
					choose();
				}
				
				//Cambia la imagen a YIQ
				if (waitKey(2)=='y'){
					//RGBtoYIQ(freezeImage);
					cambia=2;
					histograma(YIQImage);
					choose();
				}
				
				//Cambia la imagen en la que se desea trabajar
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
				
				//Cambia la imagen a binario e intensidad de grises o la regresa si es que ya estaba en grises
				if (waitKey(3) == 'b') {
					if (is_binary == false){
						cvtColor(freezeImage, binImage, CV_RGB2GRAY);
						threshold(binImage, binImage, 127, 255, THRESH_BINARY);
						is_binary = true;
						shownImage = binImage.clone();
					}
					else {
						is_binary = false;
						shownImage = freezeImage.clone();
					}
				}
			}
			
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
