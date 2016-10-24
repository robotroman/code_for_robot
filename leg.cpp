#include "leg.h"

#include "servo.h"
#include "point3d.h"
#include <algorithm>
#include <cfloat>

#define rad2deg 57.29577951308232087679815481410517033240547246656432154916
#define deg2rad 0.017453292519943295769236907684886127134428718885417254560


//! class leg constructor
//!It turns on the three servos in given one leg
//!\param int n Legs number

leg::leg(int n)
{
	number = n;
	int servo_number;
	int servo_channel;
	double servo_minimal_value, servo_middle_value, servo_maximal_value;
	servo_number = tab_of_legs[n][0];                       //!< The selection of robot side   \sa tab_of_legs
	side_of_robot = tab_of_legs[n][3];                      //!< The selection of robot side    \sa tab_of_servos
	int i;
	for (i = 0; i < 3; i++)
	{
		servo_number = tab_of_legs[n][i];                                   //!< The choice of servo in leg
		servo_channel = tab_of_servos[servo_number][0];                     //!< The choice of servo number from table of servos
		servo_minimal_value = tab_of_servos[servo_number][1];               //!< The selection of servo minimal value from table of servos
		servo_middle_value = tab_of_servos[servo_number][2];                //!< The selection of servo medium value from table of servos
		servo_maximal_value = tab_of_servos[servo_number][3];               //!< The selection of servo maximal value from table of servos
		servos[i] = new servo(servo_channel, servo_minimal_value, servo_maximal_value, servo_middle_value);   //!< Call servo in leg
	}
}

//!\fn calc_vector_for_trajec Function, which calculates the distance between two given nodes and devide the result by given value of samples multiplied by the value of given factor
//!\param point3d v1,v2 The 3 dimensional(point3d) variables, which hold the starting and next value of given nodes
//!\param double a Sample, which divide leg trajectory by given value
//!\var result The point3d(3 dimensional) vector,which holds the calculation results for trajectory
//!

vector	<point3d> leg::calc_vector_for_trajec(point3d v1, point3d v2, float a)
{
	vector	<point3d> result;

//! Calculate distance between start and next nodes and then devide this distance by given value of sample
//!
	double deltax = (v2.x - v1.x) / a;              //!< Auxiliary variable for calculation.Holds the result for x plane
	double deltay = (v2.y - v1.y) / a;              //!< Auxiliary variable for calculation.Holds the result for y plane
	double deltaz = (v2.z - v1.z) / a;              //!< Auxiliary variable for calculation.Holds the result for z plane

	point3d tmp;                                    //!< Auxiliary variable for calculation.

//! Calculates next subpoints from the calculated distance in order to create trajectory.
//!
	for (int i = 0; i < a; i++)
	{
		tmp.x = v1.x + i*deltax;                    //!< Auxiliary variable for calculation.Holds the result for x plane
		tmp.y = v1.y + i*deltay;                    //!< Auxiliary variable for calculation.Holds the result for y plane
		tmp.z = v1.z + i*deltaz;                    //!< Auxiliary variable for calculation.Holds the result for z plane
		result.push_back(tmp);
	}

	return result;
}




//!\fn calc_trajectory_for_crawling in this function the longest distance between two points is devided by 6.
//!Additionally we devide those calculated vectors  by 3, which gives us total 6*3=18 point for 4 nodes.(The sample value could not be changed)
//!\param nodes Value of points of trapezoid trajectory given by the user  in x,y and z axis
//!\param float factor The factor, given by the user in order to control leg`s step
//!\fn calc_vector_for_trajec Function, which calculates the distance between two given nodes and devide the result by given value of samples multiplied by the value of given factor
//!\var traj_crawling The  point3d(3 dimensional)  vector,which holds the calculation results for trajectory
//!\sa calc_vector_for_trajec
//!
int  leg::calc_trajectory_for_crawling(vector<point3d>nodes,float factor)
{
    traj_crawling.clear();                                      //!< clear the trajectory in order not to get segmentation fault
	traj_crawling = calc_vector_for_trajec(nodes[0],nodes[1], factor*(6 * (1 +nodes.size()-2 )));
	for (int i = 2; i <nodes.size(); i++)
		traj_crawling.push_back(nodes[i]);
		return 0;
}

//!\fn calc_for_trajectory_tripod Calculates trajectory for tripod gait from given nodes and samples
//!\param nodes Value of points of trapezoid trajectory given by the user  in x,y and z axis
//!\param int a Samples, which divide leg trajectory by given value
//!\param int paramater The parameter which indicates if it is left or right leg
//!\param float factor The factor, given by the user in order to control leg`s step
//!\var result0,result1,result2,result3,result4 The vector type point3d(3 dimensional)  variables , which hold the result calculation from function calc_vector_for_trajec
//!\fn calc_vector_for_trajec Function, which calculates the distance between two given nodes and devide the result by given value of samples multiplied by the value of given factor
//!\var traj_tripod The  point3d(3 dimensional) vector,which holds the calculation results for trajectory: result0,result1,result2,result3,result4
//!
int leg::calc_for_trajectory_tripod(vector<point3d>nodes, int a, int parameter,float factor)
{
	std::vector	<point3d> result0,result1, result2, result3,result4;
	result0 = calc_vector_for_trajec(nodes[0],nodes[1], factor*a);                  //!\sa calc_vector_for_trajec
	result1 = calc_vector_for_trajec(nodes[1],nodes[2], factor*a/4);
	result2 = calc_vector_for_trajec(nodes[2],nodes[3], factor*a/2);
	result3 = calc_vector_for_trajec(nodes[3],nodes[0], factor*a/4);


    traj_tripod.clear();                                                          //!< clear the trajectory in order not to get segmentation fault
	if (parameter == 0)
	{
		traj_tripod.insert(traj_tripod.end(), result0.begin(), result0.end());
		traj_tripod.insert(traj_tripod.end(), result1.begin(), result1.end());
		traj_tripod.insert(traj_tripod.end(), result2.begin(), result2.end());
		traj_tripod.insert(traj_tripod.end(), result3.begin(), result3.end());
	}
	if (parameter !=0)
	{
		traj_tripod.insert(traj_tripod.end(), result1.begin(), result1.end());
		traj_tripod.insert(traj_tripod.end(), result2.begin(), result2.end());
		traj_tripod.insert(traj_tripod.end(), result3.begin(), result3.end());
		traj_tripod.insert(traj_tripod.end(), result0.begin(), result0.end());
	}




	return 0;
}



//!\fn forward_kinematics Calculates forward kinematics for robot movement. It calculates x,y,z cordinates in milimeters for inverse kinematics
//!\param pi1,pi2,pi3 Given angles of hip, between hip and thigh and between thigh and tibia in degrees
//!\var po21,pos2,pos3 Calculated x,y,z coordinates for leg
//!
int leg::forward_kinematics(double pi1, double pi2, double pi3)
{   int success;
	double pos1, pos2, pos3;

	  double e= hip+ thigh*cos(pi2*deg2rad) + tibia*cos((pi2+pi3)*deg2rad);   //!< Auxiliary variable for calculation
	pos3 =  thigh*sin((pi2)*deg2rad) + tibia*sin((pi2+pi3)*deg2rad);
    if (abs(pos3)< DBL_EPSILON) pos3=0;                                       //!< Eliminating the numerical errors
    pos2=e*cos(pi1*deg2rad);
	if(abs(pos2)<DBL_EPSILON) pos2=0;                                         //!< Eliminating the numerical errors
    pos1 = e*sin(pi1*deg2rad);
	if(abs(pos1)<DBL_EPSILON) pos1=0;                                         //!< Eliminating the numerical errors
    cout<<"pos1: "<<pos1<<"pos2: "<<pos2<<"pos3 "<<pos3<<endl;
    move_leg_to_xyz(pos1,pos2,pos3);                                          //!< Move leg to calculated x,y,z cordinates by using inverse kinematics


//	 move_leg_to_xyz(pos1, pos2, pos3);
 //   if(success!=0)
   // {perror(("error in leg::forward_kinematics"));
    //    return -1;}
	return 0;
}

//!\fn canon_position Moves leg to canonical position. Sets servos value 0,0,90 degrees for one leg
//!It returns 0 if succeeded or -1 if fails
//!
int leg::canon_position()
{
int success=set_servos_in_leg(0, 0, 90);
if(success!=0)
 {perror(("error in leg::canon_position"));
 return -1;}
	return 0;
}

//!\fn leg_poweroff Turns leg off
//!It returns 0 if succeeded or -1 if fails
//!
int leg::leg_poweroff()
{
	int j;
    int success;
	for (j = 0; j<3; j++)
	{success=(servos[j]->poweroff_servo());
    if(success!=0)
     {perror(("error in leg::leg_poweroff"));
 return -1;
     }}
    return 0;
};

//!\fn set_servos_in_leg Sets desired servos values in one leg
//!returns 0 if it is a success, it returns -1 if fails
//!\param h,m,n Values in miliseconds for first,second and third servo
//!\var side_of_robot indicator for left or right side of robot.0 for left side, 1 for right side
//!
int leg::set_servos_in_leg(double h, double m, double n)
{
    int success;
	if (side_of_robot ==right_side)                 //!< Left side of robot is opposite to right side of robot
	{
    success=(servos[0]->go(-h));
    success=(servos[1]->go(-m));
    success=(servos[2]->go(n));
    }
	else
	{
		 success=(servos[0]->go(h));
        success=(servos[1]->go(m));
        success=(servos[2]->go(-n));
	}
	if(success!=0)
	{perror(("error in leg::set_servos_in_leg"));
    return -1;
	}
	return 0;
}

//!\fn move_leg_to_xyz Calculates inverse kinematics for robot movement and as a result it moves leg(three servos) to calculated position
//! if it succeeds it returns the setting of one leg, if it fails it returns -1
//!\param x,y,z X,y,Z coordinates for leg in milimeters
//!\var hip_angle,thigh_angle,tibia_angle Calculated values for angles, which are:hip angle,angle between hip and thigh, angle between thigh and tibia
//!
int leg::move_leg_to_xyz(double x, double y, double z)
{
    int success;
    double hip_angle, thigh_angle,tibia_angle;

	hip_angle = atan2(x,y) *rad2deg ;                                  //!< Hip angle calculated by arcus tangent from y divided by x and result converted to degrees


	double c = sqrt(x*x + y*y);                                        //!< Auxiliary variable for calculation. Square root from x plus y milimeters
	if(hip_angle>90 || hip_angle<-90)                                  //!< Condition to check if angle of hip exceeds 90 or -90 degrees. If it exceeds these values, it takes -square root from x plus y
	{
	c=-c;
	if(thigh_angle >90)                                                //!< If angle between hip and angle is more than 90 degrees, the hip angle will be decreased by 180 degrees, in other case it will increase by 180 degrees
	{ hip_angle-=180;}
	else {hip_angle+=180;}
	if(hip_angle>0) hip_angle-=180;                                    //!< If hip angle is more than 0, the angle will be decreased by 180 degrees, in other case it will increase hip angle by 180 degrees
	else hip_angle+=180;
}


	double r = sqrt((c-hip)*(c-hip)+z*z);                              //!< Auxiliary variable for calculation.
	double se =  (r*r - tibia*tibia + thigh*thigh)/(2 * r*thigh);      //!< Auxiliary variable for calculation. Cosine theorem for triangle
	double pi1 = acos(se)* rad2deg;                                    //!< Auxiliary variable for calculation. Calculating cosine from auxiliary variable se and changing the result to degrees
	double se2=atan2(c-hip,z)*rad2deg;                                 //!< Auxiliary variable for calculation. Calculating arcus tangent and changing the result to degrees

	thigh_angle = 90 - pi1 -se2;                                       //!< Calculating theangle between thigh and hip





	double w =-r*r+tibia*tibia+thigh*thigh;                            //!< Auxiliary variable for calculation.
	double v = 2 * tibia*thigh;                                        //!< Auxiliary variable for calculation.
	double wv = w / v;                                                 //!< Auxiliary variable for calculation. Cosine theorem for triangle

	tibia_angle = 180-(acos(wv)*rad2deg);                              //!< Calculating angle between tibia and thigh
  cout<<"pos1: "<<hip_angle<<"pos2: "<<thigh_angle<<"pos3 "<<tibia_angle<<endl;
   return  set_servos_in_leg(hip_angle, thigh_angle,tibia_angle);       //!< Sets calculated servos values for leg


}

//!\fn move_leg_traje_tripod this function moves leg according to calculated trajectory tripod
int leg::move_leg_traje_tripod(vector <point3d> traj_tripod)
{

	for (int i = 0; i < traj_tripod.size(); i++)
	{
	move_leg_to_xyz(traj_tripod[i].x, traj_tripod[i].y, traj_tripod[i].z);

    sleep(10);

	}

	return 0;
}


//!\fn set_leg_speed Sets leg speed by setting the speed of three servos
//! if succeedded return 0, if failure returns -1
//!\param number Number of leg
//!\param speed Desired speed
//!\var tab_of_legs Table with number of legs dedicated to left or right side of robot and servos which belong to dedicated leg
//!\sa tab_of_legs
//!var m,n,z This indicates which of three servos in one leg will be used.m-first,n-second,z-third servo
//!
int leg::set_leg_speed(int number, int speed)
{
	int m = tab_of_legs[number][0];
	int n = tab_of_legs[number][1];
	int z = tab_of_legs[number][2];


int success;

success=(servos[0]->maestroSetSpeed(fd, m, speed));             //!< Sets first servo speed by using maestroSetSpeed command with parameters fd-port,m-servo number,speed
success=(servos[1]->maestroSetSpeed(fd, n, speed));             //!< Sets second servo speed by using maestroSetSpeed command with parameters fd-port,m-servo number,speed
success=(servos[2]->maestroSetSpeed(fd, z, speed));             //!< Sets third servo speed by using maestroSetSpeed command with parameters fd-port,m-servo number,speed
                                                                //!<\sa maestroSetSpeed
 if(success=!1)
    { perror("error in set_leg_speed");
    return -1;
  }
return 0;
}

//! class leg deconstructor
leg::~leg()
{
    //dtor
}
