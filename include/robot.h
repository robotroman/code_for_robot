#ifndef ROBOT_H
#define ROBOT_H
#include "leg.h"
#include "stdafx.h"
#include "servo.h"
#include "camera.h"
#include "point3d.h"


//!\brief The class robot controls every proccess on robot. It opens the port for connection, generates and controls gaits,legs movement.
//!\brief It includes commands: \sa move_legs_to_xyz \sa zero \sa gen_traj \sa walk \sa maestroGetMovingState
//!\sa robot_canon_posit \sa crawling \sa fastwalk \sa legs_speed \sa walk_forwards \sa walk_sidewalk \sa kick_the_ball
//!\sa robot  class robot  constructor
//!\sa ~robot class robot deconstructor
//!\class robot inherites from class stdafx  \sa stdafx

class robot:public stdafx
{
    private:
    int gen_traj( std::vector <point3d>nodes, int side_of_robot,float factor=1);		        //!< using given values of nodes,sets initial positions of legs for crawling gait
                                                                                   //!< indicator to class camera \sa \class camera
    leg* legs_indicator[6];                                                                     //!< indicator to class leg \sa \class leg
    std::vector <point3d> traj_crawling;                                                        //!< point3d(3 dimensional) vector with points of trajectory for crawl gait
    std::vector <point3d> forward_points;                                                       //!< the point3d (3 dimensional) vector variable, which has points in the appropriate order to  walk forward
    std::vector <point3d> backwards_points;                                                     //!< the point3d (3 dimensional) vector variable, which has points in the appropriate order to  walk backward
    std::vector <point3d> turn_points_left;                                                     //!< the point3d (3 dimensional) vector variable, which has points in the appropriate order to turn left
    std::vector <point3d> turn_points_right;                                                    //!< the point3d (3 dimensional) vector variable, which has points in the appropriate order to turn right
    std::vector <point3d> sidewalk_left_points;                                                 //!< the point3d (3 dimensional) vector variable, which has points in the appropriate order to walk left side
    std::vector <point3d> sidewalk_right_points;                                                //!< the point3d (3 dimensional) vector variable, which has points in the appropriate order to walk right side
    vector <point3d> create_default_nodes(int direction,float beta);                            //!< contains default nodes and creates trajectories for different ways of movement
    double size_of_step_forwards,size_of_step_sidewalk;                                         //!< the size of step calculated in centimeters for one leg for forwards/backwards/sidewalk  gait
    int crawling(std::vector <point3d> nodes, std::vector <point3d> nodes2,int k,float factor_left_leg=1, float factor_right_leg=1);					//!< crawl gait. The default value of sample  is 18 and it could not be changed
	int tripod(int number, std::vector <point3d> nodes, std::vector <point3d> nodes2, int m,int k,float factor_left_leg=1, float factor_right_leg=1);	//!< robot tripod gait. 3 possibilities of movement: forwards/backwards and turn

    public:
    robot();
     camera* cam;                                                                                    //!< class robot constructor
    std::vector <point3d>left_side_nodes;
    std::vector <point3d>right_side_nodes;
    std::vector <point3d>create_nodes(int no,float a, float b, float c);                        //!< creates from given by the user points, nodes for left side and right side
    int move_legs_to_xyz(double x, double y, double z);											//!< inverse kinematics. Moves each leg to x, y, z points given by the user
	int zero();																					//!< turns off robot
    void walk( int gait , int direction ,int samples, int iteration, int speed,float beta, float factor_left_leg=1, float factor_right_leg=1);          //!< moves robot with desired  gait, direction, speed, samples and iteration                                                                   //robot walk
	int maestroGetMovingState(int fd);                                                                                                                  //!< this command is used to determine if the servo has reached their target or is still changing
	int robot_canon_posit();                                                                                                                            //!< moves legs to canonical position. Sets servos value 0,0,90 degrees for each  leg
    int legs_speed(int speed);                                                                                                                          //!< sets the desired speed for legs
    int walk_forwards(int gait,double dis_from_ball,int speed, float beta=1,float factor_left_leg=1, float factor_right_leg=1);            //!< moves robot forward/backward with desired gait,calculated distances in class camera
    int walk_sidewalk(int gait,int probe,double dis_from_cent,int speed,int prep_for_ball, float beta=1, float factor_left_leg=1, float factor_right_leg=1);  //!< moves robot left or right  with desired  gait, calculated distance and value of probe in the class camera
    void kick_the_ball(int probe);                                                                                                         //!< it kicks the ball after the robot tracked it and moved towards its direction
    void go_to_the_ball(int gait,int speed,float beta=1,float factor_left_leg=1, float factor_right_leg=1);                                //!< it moves robot to the ball using functions walk_forwards and walk_sidewalk, distances calculated in class camera and kicks the ball
    void pos_for_walk(int probe,std::vector <point3d> nodes, std::vector <point3d> nodes2,int beta);                                       //!< sets legs to position to walk forwards or sidewalk for tripod gait
    void up_and_down(int probe);                                                                                                           //!< the robot rises up in order to walk forwards or lowers down in order to turn around or go sideways
//!\enum gait It allows to choose the gait
//! Two types of gait: tripod(fast) and crawl(slow)
//!
struct  gait{
	int tripod=0;
	int crawl=1;
	}rgait ;
//!\enum direction It allows to choose the direction of gait
//! Six types of direction:forwards,backwards,turn left,turn right, walk left side, walk right side
//!
struct direction    {
    int forwards=0;
    int backwards=1;
    int turn_left=2;
    int turn_right=3;
    int sidewalk_left=4;
    int sidewalk_right=5;
    } rdirection;
                                      //!
        virtual ~robot();                                               //!< class robot deconstructor

};


#endif // ROBOT_H
