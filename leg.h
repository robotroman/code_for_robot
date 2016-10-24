#ifndef LEG_H
#define LEG_H
#include "stdafx.h"
#include "servo.h"
#include "point3d.h"

//!\brief The class leg controls one leg(3 servos) and to do it, it uses class servo
//!\brief It calculates trajectory from given nodes for tripod and crawl gait, inverse and forward kinematics in order to move leg to given position
//!\brief It includes commands: \sa leg_poweroff \sa canon_position \sa set_servos_in_leg \sa move_leg_to_xyz \sa forward_kinematics \sa calc_for_trajectory_fastwalk \sa  calc_trajectory_for_crawling \sa set_leg_speed
//!\sa leg  class leg  constructors
//!\sa ~leg class leg deconstructor
//!\class leg inherites from class stdafx  \sa stdafx
//! \var number The number of leg from 0 to 5
//!
class leg:public stdafx
{
    private:
	int number;
 //   double hip_angle, thigh_angle,tibia_angle;
	servo *servos[3];																			              //!< indicator to servo class
	std::vector <point3d> calc_vector_for_trajec(point3d v1, point3d v2, float a);				              //!< calculates vector of points from two given nodes. Used to calculate trajectories
    int move_leg_traje_tripod(std::vector<point3d> traj_fastwalk);
    public:
    leg(int number);                                                                                          //!<  class constructor
    std::vector <point3d> traj_tripod;													                  //!< vector with trajectory for tripod ga it
	std::vector <point3d>traj_crawling;														                  //!< vector with trajectory for crawl gait
	int leg_poweroff();																		                  //!< turns off leg
	int canon_position();																	                  //!< moves leg to default canonical position
	int set_servos_in_leg(double h, double m, double n);										              //!< sets 3 servos in leg
	int move_leg_to_xyz(double x, double y, double z);										                  //!< inverse kinematics.Calculates the angles from given xyz values, and then moves leg to calculated angles
    int side_of_robot;                                                                                        //!< indicates the left or right side of robot
    int initial_trajectory_position, current_trajectory_position;                                             //!< auxiliary variables used to calculate trajecotry for crawl gait
	int forward_kinematics(double pi1, double pi2, double pi3);								                  //!< forward kinematics.Calculates position xyz from given angles
    int calc_for_trajectory_tripod(std::vector<point3d>nodes,  int a, int parameter, float factor=1);         //!< calculates leg trajectory for tripod gait
	int calc_trajectory_for_crawling(std::vector<point3d>nodes,float factor=1);                               //!< calculates leg trajectory for crawl gait
	int set_leg_speed(int number,int speed);																  //!< sets 3 servos speed in leg


        virtual ~leg();

};

#endif // LEG_H
