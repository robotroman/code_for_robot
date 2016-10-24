/**********************************************************
 Software developed by AVA ( Ava Group of the University of Cordoba, ava  at uco dot es)
 Main author Rafael Munoz Salinas (rmsalinas at uco dot es)
 This software is released under BSD license as expressed below
-------------------------------------------------------------------
Copyright (c) 2013, AVA ( Ava Group University of Cordoba, ava  at uco dot es)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:
1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software
   must display the following acknowledgement:

   This product includes software developed by the Ava group of the University of Cordoba.

4. Neither the name of the University nor the names of its contributors
   may be used to endorse or promote products derived from this software
   without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AVA ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL AVA BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
****************************************************************/



#include "camera.h"
#include <iostream>
#include <fstream>


#define rad2deg 57.29577951308232087679815481410517033240547246656432154916
#define deg2rad 0.017453292519943295769236907684886127134428718885417254560
//! Auxiliary variables in order to detect red color and control of its hue,saturation.
//!
int minHuered=170;
int maxHuered=10;
int minSatured=70;
int maxSatured=255;
int minValuered=100;
int maxValuered=255;


int minHuered2=0;
int maxHuered2=20;
int minSatured2=0;
int maxSatured2=255;
int minValuered2=0;
int maxValuered2=255;


//!class camera constructor. It opens camera connection.
//!\fn processCommandLine Sets camera parameters according to user preferences  /sa processCommandLine
camera::camera()
{       processCommandLine ( Camera );
        cout<<"Connecting to camera"<<endl;
        if ( !Camera.open() ) {
        cerr<<"Error opening camera"<<endl;
         }
        cout<<"Connected to camera ="<<Camera.getId() <<endl;
        cout<<"Capturing"<<endl;

}
//!\fn convert_pict_to_color_hsv This function converts taken image from RGB to BGR color and stores image in vector pictures1 and pictures2
//! then in converts BGR image to HSV and then in stores in vector pictures2
//!\param img The type image vector, which holds images taken by the camera
//!\param b The type int parameter, which is the beginning of the vector img
//!\param e The type int parameter, which is the ending of the vector img
//!\var img2 The image variable,which helds the converted image to BGR, and which is stored in vector pictures
//!\var img3 The image variable, which helds the converted image to HSV and which is stored in vector pictures2
//!\var pictures1 The type image vector, which helds the converted to BGR images
//!\var pictures2 The type image vector, which helds the converted to HSV images
//!
  int camera::convert_pict_to_color_hsv(vector<Mat> img, int b, int e)
{
Mat img2,img3;
    for(int i=b;i<=e;i++)
        {
        cvtColor(img[i],img2,COLOR_RGB2BGR);
        pictures1.push_back(img2);
        cvtColor(img2,img3,COLOR_BGR2HSV);
        medianBlur(img3,img3,9);
        pictures2.push_back(img3);
        }
        return 0;
        }


//!\fn add_track This function selects appropriate values of saturation and value of colour red in order to find the best ball shape
//!\param img_origin The image type parameter, which holds the original colour image
//!\param img The image type parameter, which holds the converted and blurred image to hsv space
//!\var precision The float type variable, which determines the desired ratio of radii of the ellipse
//!\var rgb The image type variable, which is the clone of original image
//!\var step The int type variable, which is used to increase or decrease the iteration of value or saturation
//!
  int camera::add_track(Mat img_origin, Mat img)
{ float precision=0.92;
Mat rgb;
rgb=img_origin.clone();
red_ball(img,rgb);
int step=1;
indicator_for_calc_of_ellipse=0;
if(contours.size()!=0)
{do
{
    for (minSatured=60;minSatured<180;minSatured+=step)
    {
        rgb=img_origin.clone();
        red_ball(img,rgb);                     //!<\fn red_ball Detects red ball
        if(ratio_of_radii>ratio_of_radii_max)                            //!< The condition to find the best ratio of radii of ellipse
        {
            minSaturedab=minSatured;           //!< If the condition is fullfilled, the value of Saturation is overwritten and will be uused further
            ratio_of_radii_max=ratio_of_radii;
            if(ratio_of_radii_max>precision)   //!< If the condition is fullfilled, the program will calculate the distance from camera to the ball and from center of the screen
                {indicator_for_calc_of_ellipse=1;                          //!< \var int k, that launches the calculation of ellipse, distance from ball in red_ball function
                break;
        }}
        if(ratio_of_radii>precision-.01) step=2;            //!< If the ratio of radii of ellipse changes only by the first number after comma, the sep of iteration for satu will be 2
        else
        if(ratio_of_radii>precision-.02) step=5;            //!< If the ratio of radii of ellipse changes only by the second number after comma, the sep of iteration for satu will be 5
        else
        step=10;                               //!< If the ratio of radii of ellipse is beyond desired values, the sep of iteration for satu will be 10
            }
    cout<<"koniec przejscia po saturacji, ratio_of_radii_max="<<ratio_of_radii_max<<" dla saturacji: "<<minSaturedab<<" i wartosci: "<<minValuered<<endl;
    minSatured=minSaturedab;                    //!< After selecting the best saturation value, it assigns best value to the global saturation value
    if(ratio_of_radii_max>precision)                          //!< The condition to find the best ratio of radii of ellipse
       {indicator_for_calc_of_ellipse=1;
        break;}
    ratio_of_radii_max=0;
    step=1;
    for(minValuered=60;minValuered<180;minValuered+=step)
    {
        cout<<"satu: "<<minSatured<<" value: "<<minValuered<<"temporary ratio_of_radii=";
        rgb=img_origin.clone();
        red_ball(img,rgb);
        if(ratio_of_radii>ratio_of_radii_max)                              //!< The condition to find the best ratio of radii of ellipse
        {
            minValueredab=minValuered;           //!< If the condition is fullfilled, the Value pf red colour is overwritten and will be uused further
            ratio_of_radii_max=ratio_of_radii;
                if(ratio_of_radii_max>precision)              //!< If the condition is fullfilled, the program will calculate the distance from camera to the ball and from center of the screen
                {indicator_for_calc_of_ellipse=1;                            //!< \var int k, that launches the calculation of ellipse, distance from ball in red_ball function
                break;}}
        if(ratio_of_radii>precision-.01) step=2;             //!< If the ratio of radii of ellipse changes only by the first number after comma, the sep of iteration for satu will be 2
        else
        if(ratio_of_radii>precision-.02) step=5;             //!< If the ratio of radii of ellipse changes only by the second number after comma, the sep of iteration for satu will be 5
        else
        step=10;                                //!< If the ratio of radii of ellipse is beyond desired values, the sep of iteration for Value will be 10
        }
    cout<<"koniec przejscia po saturacji, ratio_of_radii_max="<<ratio_of_radii_max<<" dla saturacji: "<<minSaturedab<<" i wartosci: "<<minValueredab<<endl;
    minValuered=minValueredab;                   //!< After selecting the best Value, it assigns best value to the global Value
    }while(ratio_of_radii_max<precision & contours.size()!=0);
    red_ball(img,rgb);                      //!<\fn red_ball Detects red ball
    namedWindow("pic",0);                  //!<  The window with the image of the ball with best Saturation and Value
    waitKey(100);
    imshow("pic",rgb);
    waitKey(100);
    cout << "ratio_of_radiimax= " << ratio_of_radii_max << ", s= " << minSaturedab << ", v= " << minValueredab <<  endl;
    }
    else cout<<"no ball detected"<<endl;

    return 0;
    }

//!\fn camera_film It takes the images, calls the functions responsible for conversion of images and detecting ball
//!\param nCount The number of taken images in order to recognize round object
//!\fn  convert_pict_to_color_hsv This function converts taken image from RGB to BGR color and stores image in vector pictures1 \sa convert_pict_to_color_hsv
//!\fn add_track This function selects appropriate values of saturation and value of colour red in order to find the best ball shape \sa //!\fn add_track
//!\var pictures The type image vector, which helds the unconverted  images taken by camera
//!\var pictures1 The type image vector, which helds the converted to BGR images
//!\var pictures2 The type image vector, which helds the converted to HSV images
//!
int camera::camera_film( int nCount)
   {Mat image;                                      //!< Auxiliary variable for image proccess
//! By taking 20 images without storing them,it adjusts camera to the surroundings
    for ( int j=0; j<20; j++ ) {
        Camera.grab();}

    for ( int j=0; j<nCount; j++ ) {
        Camera.grab();
        Camera.retrieve ( image );
        pictures.push_back(image);
    }
    cout<<"Capturing done"<<endl;
    convert_pict_to_color_hsv(pictures,0,nCount-1);
    add_track(pictures1[nCount-1],pictures2[nCount-1]);
    usleep(1e4);
    pictures.clear();
    pictures1.clear();
    pictures2.clear();
    return 0;
   }


//!\fn processCommandLine Sets camera parameters according to user preferences
//!\param raspicam::RaspiCam_Cv &Camera Camera object
void camera::processCommandLine ( raspicam::RaspiCam_Cv &Camera )
{
    Camera.set ( CV_CAP_PROP_FRAME_WIDTH,  800 );
    Camera.set ( CV_CAP_PROP_FRAME_HEIGHT, 600 );
    Camera.set ( CV_CAP_PROP_BRIGHTNESS,50 );
    Camera.set ( CV_CAP_PROP_CONTRAST ,50 );
    Camera.set ( CV_CAP_PROP_SATURATION, 50 );
    }


//!\fn red_ball Detects red ball
//!\param image_after_conversion The image type parameter after conversion to HSV
//!\param image_org The image taken by camera and converted to BGR
//!\fn draw_cont It detects round object, draws rectangle contours around it and calculates distance from camera to object and from the center of the camera to center of the object and stores the results in vectors: distance_x and distance_y
//!\sa draw_cont
//!
void camera::red_ball (Mat image_after_conversion,Mat image_org)
{
Mat red,red2;                   //!< Auxiliary variables for image proccess in order to detect red ball

//!Sets the default range in order to detect red colour. 2 ranges for colour red
if(minHuered<maxHuered)
inRange(image_after_conversion,Scalar(minHuered,minSatured,minValuered),Scalar(maxHuered,maxSatured,maxValuered),red);
else
{inRange(image_after_conversion,Scalar(minHuered,minSatured,minValuered),Scalar(180,maxSatured,maxValuered),red);
inRange(image_after_conversion,Scalar(0,minSatured,minValuered),Scalar(maxHuered,maxSatured,maxValuered),red2);
red=red+red2;
}
draw_cont(red,image_org);
if (contours.size()!=0)
{
drawContours(image_org, contours, -1, CV_RGB(0,0,255),1);
}}
//int l=0;
//stringstream ss;
//string name="cropped_ ";
//string type =".jpg";
//!\fn draw_cont It detects round object, draws contours around it and calculates distance from camera to object and from the center of the camera to center of the object
//!then stores the results in vectors: distancex_cam and distancey_cam
//!\param image_after_conversion The image  converted to HSV
//!\param image_org The image converted to BGR
//!\var contours The vector variable which helds the set of points of found object
//!\var distancex_cam The calculated in milimeters value of distance between center of the round object and camera in x axis
//!\var distancey_cam The calculated in milimeters value of distance between center of the round object and the center of camera in y axis
//!
int  camera::draw_cont(Mat& image_after_conversion, Mat& image_org)
{
        contours.clear();
    //! It finds only extern contours
        findContours(image_after_conversion, contours, RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    //! It removes contours, which are not round
        for (int i = 0; i < contours.size(); i++)
        {
		double area = contourArea(contours[i]);
		double perimeter = arcLength(contours[i], true);
    //! The ratio of perimeter of the square to the surface of the circle is 4 * pi ^ 2 * r ^ 2 / (pi * r ^ 2) = 4 * pi that is 12.56
		if (area < 250 || area > 380000  || perimeter*perimeter / area>40)
		{     //! Remove  given contour
			contours.erase(contours.begin() + i, contours.begin() + i + 1);
			i--;
		}}
    //! It checks if the object was detected, if not it will not do further calculations
    //! It will set the probe_indicator_for_camera to 2 in order to turn robot
        if(contours.size()==0)
        {dis_ball_from_camera=0;
        probe_indicator_for_camera=2;
        cout<<"no ball detected "<<endl;
        return 0;
        }
        else
        {
       //! Find the rotated rectangles and ellipses for each contours
        vector<RotatedRect> minRect( contours.size() );
        vector<RotatedRect> Ellipse( contours.size() );
       //! Calculaton for ellipse contour
        for (int i=0;i<contours.size();i++)
        {
        minRect[i] = minAreaRect( Mat(contours[i]) );
       if( contours[i].size() > 1 )
         { Ellipse[i] = fitEllipse( Mat(contours[i]) );
         a=Ellipse[i].size.width/2;
         b=Ellipse[i].size.height/2;
       //! Calculate the ratio of radii of ellipse
         ratio_of_radii=a/b;
         cout<<"ratio_of_radii: "<<ratio_of_radii<<endl;
       //! If the current ratio of radii of ellipse is greater than previous ratio, the previous value of ratio will be overwritten by the current value
         if(ratio_of_radii>previous_ratio_of_radii) previous_ratio_of_radii=ratio_of_radii;
        }}
       //! Draw contours of ellipse, circle,rotated rectangle
       for( int i = 0; i< contours.size(); i++ )
        {
       drawContours( image_org, contours, i, Scalar(0,0,255), 1, 8, vector<Vec4i>(), 0, Point() );
       ellipse( image_org, Ellipse[i], Scalar(0,0,255), 2, 8 );
       circle( image_org, Ellipse[i].center,5, Scalar(255,0,255), 2, 8 );
       Point2f rect_points[4]; minRect[i].points( rect_points );
       for( int j = 0; j < 4; j++ )
          line(image_org, rect_points[j], rect_points[(j+1)%4], Scalar(0,0,255), 1, 8 );}
       //! Calculation of the distance from camera to the ball and from center of the screen to the center of the ball
        if(indicator_for_calc_of_ellipse==1)
       {for(int i=0;i<Ellipse.size();i++)
            {
        distancex_cam=cos(21.2*deg2rad)*51*.5*image_org.cols/(Ellipse[i].size.width*tan(26.75*deg2rad));
        distancey_cam=(image_org.cols*0.5-Ellipse[i].center.x)*distancex_cam*tan(26.75*deg2rad)/(image_org.cols*0.5);
//        ss<<name<<(l+1)<<type;
//        string filename=ss.str();
//        ss.str("");
//        imwrite(filename,image_org);

        cout<< "wyliczenie x: "<<distancex_cam<<endl;
        cout<< "wyliczenie y: " <<distancey_cam<<endl;
      //!Distinguish between left and right side of the robot
      //!if distancey_cam(the distance between center of the screen and the ball) is non-positive the ball is on the right side of the robot
      //!in other case it is on the left
        if(distancey_cam<0)
         {probe_indicator_for_camera=0;}
        else {probe_indicator_for_camera=1;}
        }}}
        return 0;}

//!\fn calculate_median Calculates median values from distance vectors: distance_x and distance_y,which are created in function draw_cont   \sa draw_cont
//!Camera measurments are in milimeters.
//!\var n The middle element of vector  distance_x
//!\var n1 The middle element of vector  distance_y
//!\var dis_ball_from_camera The medium value of vector  distance_x. Given in centimeters
//!\var dis_ball_from_scr_center The medium value of vector  distance_y.Given in centimeters
//!\var probe_indicator_for_camera The value 0 indicates that robot will go right,1 indicates that robot will go left
//!
  void camera::calculate_median()
    {
vector <int> distance_x,distance_y;
    size_t n=distance_x.size()/2;
     size_t n1=distance_y.size()/2;
    nth_element(distance_x.begin(),distance_x.begin()+n,distance_x.end());
     nth_element(distance_y.begin(),distance_y.begin()+n1,distance_y.end());
    dis_ball_from_camera=distance_x[n];
     dis_ball_from_src_center=distance_y[n1];
    if(distance_x.size()%2==1)
    {dis_ball_from_camera=dis_ball_from_camera/10;
    cout<<"median: " <<dis_ball_from_camera<<endl;}
    else
    {nth_element(distance_x.begin(),distance_x.begin()+n-1,distance_x.end());
    dis_ball_from_camera =0.5*(dis_ball_from_camera+distance_x[n-1]);
    distance_x.clear();
    dis_ball_from_camera=dis_ball_from_camera/10;
    cout<<"median: " <<dis_ball_from_camera<<endl;;}

    if(distance_y.size()%2==1)
    {dis_ball_from_src_center=dis_ball_from_src_center/10;
    cout<<"median: " <<dis_ball_from_src_center<<endl;}
    else
    {nth_element(distance_y.begin(),distance_y.begin()+n1-1,distance_y.end());
    dis_ball_from_src_center =0.5*(dis_ball_from_src_center+distance_y[n1-1]);
    distance_y.clear();
    dis_ball_from_src_center=dis_ball_from_src_center/10;
    cout<<"median: " <<dis_ball_from_src_center<<endl;;}
        if(dis_ball_from_src_center<0) {probe_indicator_for_camera=0;
        }
        else {probe_indicator_for_camera=1;
    }}
    //!camera class deconstructor
    //!
    camera::~camera()
    {
    //dtor
    }
