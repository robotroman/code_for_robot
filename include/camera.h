#ifndef CAMERA_H
#define CAMERA_H
#include <ctime>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include "raspicam_cv.h"
#include <cv.h>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "objdetect.hpp"

#include <functional>
#include "stdafx.h"




using namespace std;
using namespace cv;

//!\brief In the class camera there are functions, which proccess images received from camera and calculate distances to tracked objects
//!\brief It includes commands: \sa processCommandLine \sa red_ball
//!\sa draw_cont \sa camera_film \sa calculate_median
//!\sa camera  class camera  constructor
//!\sa ~camera class camera deconstructor
//!\class camera inherites from class stdafx  \sa stdafx
//!\class camera shares its results of calculation with the class robot \sa robot
class camera:public stdafx
{
    private:
     raspicam::RaspiCam_Cv Camera;                                                         //!< Camera object
     void processCommandLine ( raspicam::RaspiCam_Cv &Camera );                            //!< sets camera parameters according to user preferences
     void red_ball (Mat image_after_conversion,Mat image_org);                             //!< Detects red ball
     int indicator_for_calc_of_ellipse=0;                                                   //!< Indicator, which allows to calculate distances between ball and the camera

     float a,b,ratio_of_radii;                                                             //!< a-the width of ellipse,b-the height of ellipse,the ratio of radii of ellipse
     float previous_ratio_of_radii=0;
     int convert_pict_to_color_hsv(vector<Mat> img, int b, int e);                         //!< This function converts taken image from RGB to BGR color and stores image in vector pictures1 and pictures2
     int add_track(Mat img_origin, Mat img);                                               //!< This function selects appropriate values of saturation and value of colour red in order to find the best ball shape
     vector<Mat> pictures;                                                                 //!< The type image vector, which helds the unconverted  images taken by camera
     vector<Mat> pictures1;                                                                //!< The type image vector, which helds the converted to BGR images
     vector<Mat> pictures2;                                                                //!< The type image vector, which helds the converted to HSV image s
     float ratio_of_radii_max=0;
     int temp_pos_minValue=100;                                                             //!< The temporary minimal position Value of red colour
     int temp_pos_minSat=130;                                                               //!< The temporary minimal position value of saturation of red colour
     int minValueredab=temp_pos_minValue;                                                   //!< After selecting the best Value, it assigns best value to the global Value
     int minSaturedab=temp_pos_minSat;                                                      //!< After selecting the best saturation value, it assigns best value to the global saturation value
    public:
      camera();                                                                             //!< class camera constructor
      int camera_film( int nCount);                                                         //!< It takes the images, calls the functions responsible for conversion of images and detecting ball
      double distancex_cam, distancey_cam;                                                  //!< distancex_cam The calculated in milimeters value of distance between center of the round object detected and camera in x axis
                                                                                            //!< distancey_cam The calculated in milimeters value of distance between center of the round object detected and the center of camera in y axis
      void calculate_median();                                                              //!< calculates median values from distance vectors: distance_x and distance_y,which are created in function draw_cont   \sa draw_cont
      int probe_indicator_for_camera;                                                       //!< The value 0 indicates that robot will go right,1 indicates that robot will go left, 2 the robot will turn around left
        int draw_cont(Mat& image_after_conversion, Mat& image_org);                         //!< It detects round object, draws contours around it and calculates distance from camera to object and from the center of the camera to center of the object
        double dis_ball_from_src_center, dis_ball_from_camera;
      vector <vector<Point> > contours;                                                     //!< The vector variable which helds the set of points of found object
      virtual ~camera();                                                                    //!< The deconstructor of class camera

    };

#endif // CAMERA_H

