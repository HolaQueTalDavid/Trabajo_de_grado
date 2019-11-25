#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


  #include <iostream>
  #include <fstream>
  #include "string"
  #include <stdio.h>
  #include <stdlib.h>
  #include <vector>


using namespace std;
using namespace cv;

vector<int> v;

int k=0;

int confirmacion=0;
int area;
int veces=0;
//definition of values matrix camera and values distortion matrix
float fx=505.0590676001461;
float fy=505.4132151471707;
float cx=298.00492309887466;
float cy=251.26893943667042;
float k1=0.18441465558010364;
float k2=-0.6962997620031933;
float k3=-0.00539918455619508;
float k4=-0.005484312347412547;
float k5=0.7507887477679294;
//Variables to do the Pinhole Model
float xu=0;
float yu=0;
float xprim=0;
float yprim=0;
float xpix=0;
float ypix=0;
float xc=0;
float yc=0;
float z=0;
float zdepth=0;
float zmatriz[8];
float zprom=0;
float zref=0;
float altura=0;
float prompos[101][3];

int argc;
char **argv;
RNG rng(12345);

Mat ColorKinect;
Mat DepthKinect;
void rgbCallback(const sensor_msgs::ImageConstPtr& msg)
{
cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
  	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  	ColorKinect=cv_ptr->image;
  }
  catch (cv_bridge::Exception& ex)
  {
	ROS_ERROR("cv_bridge exception: %s", ex.what());
  	exit(-1);
  }
}


void depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
	 cv_ptr = cv_bridge::toCvCopy(msg,"");
   	DepthKinect = cv_ptr-> image;
  }
  catch (cv_bridge::Exception& ex)
  {
  	ROS_ERROR("cv_bridge exception: %s", ex.what());
   	 exit(-1);
  }
}


int main(int, char**, const sensor_msgs::ImageConstPtr & msg_ptr)
{
      Mat mask, hsv, mask1, mask2, mask3, mask4, mask5;
      ROS_INFO("Esperando.");
      ros::init(argc, argv, "kinectgrabber");
      ros::NodeHandle n;
      ros::Subscriber sub = n.subscribe("/camera/rgb/image_color", 1, rgbCallback);
      ros::Subscriber depth =	n.subscribe("/camera/depth_registered/image", 1, depthCallback);
    //  ros::Duration(1.0).sleep();
      namedWindow("RGB",CV_WINDOW_NORMAL);
      namedWindow("DEPTH",CV_WINDOW_NORMAL);
      namedWindow("Contours",CV_WINDOW_NORMAL);
      namedWindow("MASK",CV_WINDOW_NORMAL);
      //printf("%s\n","hi" );
      while(ros::ok()){
    //   printf("%s\n","ROSOK" );
      //printf("%i\n",ColorKinect.rows);
      //printf("%s\n","filas otra" );
    // printf("%i\n", DepthKinect.rows);


       //printf("%f\n",frame11.rows);
      if (!ColorKinect.empty() && !DepthKinect.empty())
      {
        //imwrite("/home/david/catkin_ws/Imagenes/finales/Prueba_42_C.jpg", ColorKinect);
        //imwrite("/home/david/catkin_ws/Imagenes/finales/Prueba_42_D.png",DepthKinect);
        //imwrite("/home/david/catkin_ws/Imagenes/finales/Prueba_41_M.jpg", mask);
       cvtColor(ColorKinect, hsv, CV_BGR2HSV);
       // Creating masks to detect the upper and lower red color.
       inRange(hsv, Scalar(0, 120, 70), Scalar(10, 255, 255), mask1);
       inRange(hsv, Scalar(170, 120, 70), Scalar(180, 255, 255), mask2);
        //creating masks to detect blue Color
        inRange(hsv, Scalar(100,65,75), Scalar(130, 255, 255), mask4);
        // Creatin mask to detect yellow color
        inRange(hsv, Scalar(20, 100, 100), Scalar(30, 255, 255), mask3);

        //creating masks to detect blue green
        inRange(hsv, Scalar(49,50,50), Scalar(107, 255, 255), mask5);

        circle(ColorKinect,Point(320,240),1, (Scalar(100, 50, 255)),4,8);


      //  circle(ColorKinect,Point(100,200),8, (Scalar(100, 65, 255)),4,8);
        //line(ColorKinect,Point(320,0),Point(320,480),Scalar(255, 123, 50),2,1);
        //line(ColorKinect,Point(0,240),Point(640,240),Scalar(255, 123, 50),2,1);
        mask = mask1+mask2+mask3+mask4+mask5;
        //Damos forma a la mascara para obtener una figura mas clara (Elipse)
        Mat element = getStructuringElement(MORPH_ELLIPSE, Size(15, 15));
        //Hacemos proceso de dilate y erode para mejorar el contorno
        dilate(mask, mask, element);
        erode(mask, mask, element);
        morphologyEx(mask,mask,MORPH_OPEN,element);

        //Encontramos los contronos
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        findContours(mask, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
        Mat drawing = Mat::zeros( mask.size(), CV_8UC3 );
        //encontramos los momentos
        vector<Moments> mu(contours.size());
        for( size_t i = 0; i < contours.size(); i++ )
        {
          Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
          drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );

         mu[i] = moments( contours[i] );
         area= contourArea(contours[i]);
          //printf("moments:%f\n",contours[i]);
        }
        //encontramos posición
        vector<Point2f> mc( contours.size() );
        //waitKey(30);
        if(veces<=100){
        if(area>100){ //Filtamos por area
         veces++;
          for( size_t i = 0; i < contours.size();  i++ ) //Recorremos el vector de contornos
           {
             mc[i] = Point2f( static_cast<float>(mu[i].m10/mu[i].m00) , static_cast<float>(mu[i].m01/mu[i].m00) );
             xpix=(mu[i].m10/mu[i].m00);
             ypix=(mu[i].m01/mu[i].m00);
             Point pt =  Point(xpix, ypix);
             //circle(DepthKinect,pt,40, (Scalar(130, 50, 255)),2,1);
             //circle(ColorKinect,Point(100,200),10, (Scalar(130, 50, 255)),2,8);
             if (!isnan(xpix)) {
                zdepth=DepthKinect.at<float>(pt);
                zmatriz[0]=DepthKinect.at<float>(Point(xpix-1,ypix-1));
                zmatriz[1]=DepthKinect.at<float>(Point(xpix,ypix-1));
                zmatriz[2]=DepthKinect.at<float>(Point(xpix+1,ypix-1));
                zmatriz[3]=DepthKinect.at<float>(Point(xpix-1,ypix));
                zmatriz[4]=DepthKinect.at<float>(Point(xpix+1,ypix));
                zmatriz[5]=DepthKinect.at<float>(Point(xpix-1,ypix+1));
                zmatriz[6]=DepthKinect.at<float>(Point(xpix,ypix+1));
                zmatriz[7]=DepthKinect.at<float>(Point(xpix+1,ypix+1));
                zmatriz[8]=DepthKinect.at<float>(Point(xpix,ypix));

               if (!isnan(zdepth)) {
                 for (size_t n = 0; n <=8; n++)
                 {
                   zprom=zprom+zmatriz[n];
                 }
                 zprom=zprom/9;
                xu=xpix-cx;
                 yu=ypix-cy;

                 xprim=xu/fx;
                 yprim=yu/fy;
                 //printf("%s\n","Posicion" );
                 xc=(xprim*zdepth)-0.050259;//-0.051885;
                 yc=(yprim*zdepth)+0.02418;//+0.023728;
                 z=zdepth;

                 for (size_t h = 1; h <= 100; h++) {
                   prompos[i][1]=xc;
                   prompos[i][2]=yc;
                   prompos[i][3]=z;
                   prompos[101][1]=(prompos[101][1])+prompos[i][1];
                   prompos[101][2]=(prompos[101][2])+prompos[i][2];
                   prompos[101][3]=((prompos[101][3])+prompos[i][3]);
                 }
                 prompos[101][1]=(prompos[101][1])/100;
                 prompos[101][2]=(prompos[101][2])/100;
                 prompos[101][3]=((prompos[101][3]))/100;
                 altura=((DepthKinect.at<float>(Point(100,200)))-z);

                 confirmacion=1;
               }
               else
               {
                 printf("%s\n","Error profundidad" );
                 xc=0;
                 yc=0;
                 z=0;
                 altura=0;
                 confirmacion=0;
               }
             }
             else
             {
               printf("%s\n","Error pixel" );
               xc=0;
               yc=0;
               z=0;
              // altura=0;
               confirmacion=0;
             }

            // printf("Confirmacion: %i\n",confirmacion );
             //printf("Zprom:%f\n",zprom );
             //printf("Xpix,Ypix:%f,%f\n",xpix,ypix);
             //printf("Altura botella:%f\n", altura );
             printf("pose,%f,%f,%f\n",xc,yc,z);
             //printf("Piso:%f\n",(DepthKinect.at<float>(Point(100,200))));
             //printf("%i\n", veces );
             //printf("Prom:%f,%f,%f\n",prompos[101][1],prompos[101][2],z );
           }
         }
       }
         circle(ColorKinect,Point(xpix,ypix),1, (Scalar(243, 176, 5)),4,8);
       //circle(ColorKinect,Point(153.641464,396.138458),5, (Scalar(100, 65, 255)),2,8);
         imshow( "Contours", drawing );
         imshow("RGB",ColorKinect);
         imshow("DEPTH",DepthKinect);
         imshow("MASK",mask);
         waitKey(30);

      }


       ros::Duration(0.001).sleep();
       ros::spinOnce();
     }

    return 0;
}
