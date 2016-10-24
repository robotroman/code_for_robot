#include "robot.h"

//!
//!constructor of the class robot
//!turns on robot legs, also opens port connection fd, imports table of servos values for legs from class stdafx inherited
//!\var legs_indicator Refers to each leg of robot
//!
robot::robot()
{
     stdafx* r=new stdafx();                        //!< Calls class stdafx in order to open the fd port
     cam=new camera();                              //!< Calls class camera in order to open connection with camera and functions to calculate distance bewteen camera and ball
     int j;
     for (j = 0; j < 6; j++)
     legs_indicator[j]=new leg(j);
    }


//!\fn maestroGetMovingState This command is used to determine if the servo has reached their target or is still changing
//!will return 1 if servo is still changing
//!servo must have assigned speed
//!\param int fd Port of the robot
//!
int robot::maestroGetMovingState(int fd)
{
    unsigned char command[] = {0x93};             //!<0x93-pololu compact protocol \sa www.pololu.com/dpcs/0J40/5.e
    if(write(fd, command, sizeof(command)) == -1)
    {
    perror("error writing");
    return -1;
    }
    unsigned char response[1];
    if(read(fd,response,1) != 1)
    {
    perror("error reading");
    return -1;
    }

    return response[0];
}

//!\fn gen_traj Using given values of nodes,sets initial positions of legs for crawling gait
//! each leg is offset from the previous one by 1/6
//!\param nodes Value of points of trapezoid trajectory given by the user  in x,y and z axis
//!\param side_of_robot Left or right side of robot.0 for left side, 1 for right side
//!\var legs_indicator Refers to each leg of robot
//!\fn calc_trajectory_for_crawling Calculates set of points for trapezoid trajectory from given values of nodes
//!

int robot::gen_traj( vector <point3d>nodes,int side_of_robot,float factor){
		for (int i = 0; i < 6; i++)
		{
			if (legs_indicator[i]->side_of_robot == side_of_robot)
			{
				legs_indicator[i]->calc_trajectory_for_crawling(nodes,factor);
				legs_indicator[i]->initial_trajectory_position = i * 3;
			}
		}

	return 0;
}


//!\fn create_default_nodes Contains default nodes and creates trajectories for different ways of movement
//!\param direction In which way the robot should move
//!\param beta The factor, which the user could control the length of one step for each leg with
//!\var p1,p2,p3,p4,p5,p6,p7,p8  point3d(3 dimensional) variables , which contain default points of trapezoid trajectory for forward,turn and side -walk
//!\var size_of_step_forwards The size of step calculated in centimeters for one leg for forwards/backwards gait
//!\var size_of_step_sidewalk The size of step calculated in centimeters for one leg for sidewalk gait
//!
vector <point3d> robot::create_default_nodes(int direction,float beta)
{
    if (beta>1)
   perror("beta could not be greater then 1");
  //!points for default forward/backward walk
   point3d p1(50*beta,thigh + hip-30, tibia+30);
   point3d p2(-50*beta,thigh + hip-30,tibia+30);
   point3d p3(-20*beta,thigh +hip-20, tibia -20);
   point3d p4(20*beta,thigh + hip-20,tibia -20);
  size_of_step_forwards=(p1.x-p2.x)/10;

   //!points for default sidewalk
    point3d p5(0,111,103);
    point3d p6(0,36,103);
    point3d p7(0,36,78);
    point3d p8(0,121,73);
   size_of_step_sidewalk=2*(p5.y-p6.y)/10;

//!\var forward_points The point3d (3 dimensional) vector variable, which has points in the appropriate order to walk forward
//!\var backwards_points The point3d (3 dimensional) vector variable, which has points in the appropriate order to walk backwards
//!\var turn_points_left The point3d (3 dimensional) vector variable, which has points in the appropriate order to turn left
//!\var turn_points_right The point3d (3 dimensional) vector variable, which has points in the appropriate order to turn right
//!\var sidewalk_left_points The point3d (3 dimensional) vector variable, which has points in the appropriate order to walk left side
//!\var sidewalk_right_points The point3d (3 dimensional) vector variable, which has points in the appropriate order to walk right side
//!

    switch(direction)
    { case 0:
    forward_points.clear();
    forward_points.push_back(p2);
    forward_points.push_back(p1);
    forward_points.push_back(p4);
    forward_points.push_back(p3);
    return forward_points;
    break;
    case 1:
    backwards_points.clear();
    backwards_points.push_back(p1);
    backwards_points.push_back(p2);
    backwards_points.push_back(p3);
    backwards_points.push_back(p4);
    return backwards_points;
    break;
    case 2: case 3:
    turn_points_left.clear();
    turn_points_left.push_back(p1);
    turn_points_left.push_back(p2);
    turn_points_left.push_back(p3);
    turn_points_left.push_back(p4);

     turn_points_right.clear();
    turn_points_right.push_back(p2);
    turn_points_right.push_back(p1);
    turn_points_right.push_back(p4);
    turn_points_right.push_back(p3);

    return (turn_points_left,turn_points_right);
    break;
    case 4: case 5:
    if(beta==1)
    {
    sidewalk_left_points.clear();
    sidewalk_left_points.push_back(p5);
    sidewalk_left_points.push_back(p6);
    sidewalk_left_points.push_back(p7);
	sidewalk_left_points.push_back(p8);
     sidewalk_right_points.clear();
    sidewalk_right_points.push_back(p6);
    sidewalk_right_points.push_back(p5);
    sidewalk_right_points.push_back(p8);
    sidewalk_right_points.push_back(p7);}
    else
    {
  //! calculation in order to control leg`s step for sidewalk. Divide distance in half
    float z= (p5.y+p6.y)/2;               //!< First calculate the medium value of the distance between two legs
    float g=((p5.y-p6.y)*beta)/2;         //!< Calculate how much the distance will change due to beta factor
    float h= (p7.y+p8.y)/2;
    float i=((p8.y-p7.y)*beta)/2;
    point3d p5(0,z+g,103);                 //!< Add medium point  and the calculated change(upper half)
    point3d p6(0,z-g,103);                 //!< Subtract from medium point the calculated change (lower half)
    point3d p7(0,h-i,78);
    point3d p8(0,h+i,78);
    sidewalk_left_points.clear();
    sidewalk_left_points.push_back(p5);
    sidewalk_left_points.push_back(p6);
    sidewalk_left_points.push_back(p7);
	sidewalk_left_points.push_back(p8);
     sidewalk_right_points.clear();
    sidewalk_right_points.push_back(p6);
    sidewalk_right_points.push_back(p5);
    sidewalk_right_points.push_back(p8);
    sidewalk_right_points.push_back(p7);
    }
    return (sidewalk_left_points,sidewalk_right_points);
    break;
    }}

//!\fn create_nodes  Creates from given by the user points, nodes for left side and right side
//!\param side_of_robot
//!\param a,b,c The given by the user points for x,y,z axis
//!
vector <point3d>robot::create_nodes(int side_of_robot,float a, float b, float c)
{
    point3d p(a,b,c);
    if(side_of_robot=left_side)
    {left_side_nodes.push_back(p);
    return left_side_nodes;
    }
    else
    {right_side_nodes.push_back(p);
    return right_side_nodes;}}


//!\fn Crawling Crawl gait. The default value of sample  is 18 and it could not be changed
//!\param nodes,nodes2 Values of points of trapezoid trajectory given by the user  in x,y and z axis
//!\param k the number of loop`s iteration
//!\param factor_left_leg,factor_right_leg The factors, given by the user in order to control left or right leg step
//!\fn gen_traj Using given values of nodes,sets initial positions of legs for crawling gait   \sa gen_traj
//!
int robot::crawling(vector <point3d> nodes, vector <point3d> nodes2,int k,float factor_left_leg, float factor_right_leg)
{
	gen_traj( nodes, left_side,factor_left_leg);
	gen_traj( nodes2,right_side,factor_right_leg);
    double x, y, z;                                          //!< Auxiliary variable for calculation. X,Y,Z axes.
	for (int i = 0; i < 6; i++)
	{
        legs_indicator[i]->current_trajectory_position = legs_indicator[i]->initial_trajectory_position;
		/*x = legs_indicator[i].traj_crawling[legs_indicator[i].initial_trajectory_position].x;
		y = legs_indicator[i].traj_crawling[legs_indicator[i].initial_trajectory_position].y;
		z = legs_indicator[i].traj_crawling[legs_indicator[i].initial_trajectory_position].z;*/
    }
    for (int w = 0; w < k; w++)
	{
    for (int j = 0; j < legs_indicator[0]->traj_crawling.size(); j++)
		{
			for (int i = 0; i < 6; i++)
			{
				legs_indicator[i]->current_trajectory_position++;
				legs_indicator[i]->current_trajectory_position = legs_indicator[i]->current_trajectory_position%legs_indicator[i]->traj_crawling.size();  //!< It does not allow to go beyond the given limitation of 18 samples
				x = legs_indicator[i]->traj_crawling[legs_indicator[i]->current_trajectory_position].x;
				y = legs_indicator[i]->traj_crawling[legs_indicator[i]->current_trajectory_position].y;
				z = legs_indicator[i]->traj_crawling[legs_indicator[i]->current_trajectory_position].z;
                legs_indicator[i]->move_leg_to_xyz(x, y, z);
            }
			while (maestroGetMovingState(fd));
                }}
        return 0;}

//!\fn pos_for_walk Sets legs to position to walk forwards or sidewalk for tripod gait
//!\param probe If this variable is 0 it sets legs to move forward, if 1 it sets legs for sidewalk
//!\param nodes Value of points of trapezoid trajectory given by the user  in x,y and z axis for right side of robot
//!\param nodes2 Value of points of trapezoid trajectory given by the user  in x,y and z axis for left side of robot
//!\param beta The factor, which the user could control the length of one step for each leg with
//!\var legs_indicator It chooses from  class leg (\sa \class leg) leg,which should move to desired values
//!\fn move_leg_to_xyz inverse kinematics in class leg(\sa \class leg).Calculates the angles from given xyz values, and then moves leg to calculated angles
//!\var hip,thigh,tibia The constant physical leg value given in milimeters
//!\fn maestroGetMovingState This command is used to determine if the servo has reached their target or is still changing
//!\var fd The port variable, which is used to open connection with device
//!
void robot::pos_for_walk(int probe,std::vector <point3d> nodes, std::vector <point3d> nodes2,int beta)
{
if(probe==0)
{
        legs_indicator[0]->move_leg_to_xyz(0,thigh + hip-20,tibia -20);
        while (maestroGetMovingState(fd));
        legs_indicator[0]->move_leg_to_xyz(nodes2[0].x*(0.4),thigh + hip-20,tibia -20);
         while (maestroGetMovingState(fd));
        legs_indicator[0]->move_leg_to_xyz(nodes2[0].x*(0.4),nodes[0].y,nodes[0].z);
         while (maestroGetMovingState(fd));
        legs_indicator[1]->move_leg_to_xyz(0,thigh + hip-20,tibia -20);
        while (maestroGetMovingState(fd));
        legs_indicator[1]->move_leg_to_xyz(nodes2[0].x*(-1),thigh + hip-20,tibia -20);
        while (maestroGetMovingState(fd));
        legs_indicator[1]->move_leg_to_xyz(nodes2[0].x*(-1),nodes[0].y,nodes[0].z);
         while (maestroGetMovingState(fd));

         legs_indicator[2]->move_leg_to_xyz(0,thigh + hip-20,tibia -20);
        while (maestroGetMovingState(fd));
        legs_indicator[2]->move_leg_to_xyz(nodes2[0].x*(-1),thigh + hip-20,tibia -20);
        while (maestroGetMovingState(fd));
        legs_indicator[2]->move_leg_to_xyz(nodes2[0].x*(-1),nodes[0].y,nodes[0].z);
         while (maestroGetMovingState(fd));

        legs_indicator[3]->move_leg_to_xyz(0,thigh + hip-20,tibia -20);
        while (maestroGetMovingState(fd));
        legs_indicator[3]->move_leg_to_xyz(nodes2[0].x*(0.4),thigh + hip-20,tibia -20);
         while (maestroGetMovingState(fd));
        legs_indicator[3]->move_leg_to_xyz(nodes2[0].x*(0.4),nodes[0].y,nodes[0].z);
         while (maestroGetMovingState(fd));

         legs_indicator[4]->move_leg_to_xyz(0,thigh + hip-20,tibia -20);
        while (maestroGetMovingState(fd));
        legs_indicator[4]->move_leg_to_xyz(nodes2[0].x*(0.4),thigh + hip-20,tibia -20);
         while (maestroGetMovingState(fd));
        legs_indicator[4]->move_leg_to_xyz(nodes2[0].x*(0.4),nodes[0].y,nodes[0].z);
         while (maestroGetMovingState(fd));

         legs_indicator[5]->move_leg_to_xyz(0,thigh + hip-20,tibia -20);
        while (maestroGetMovingState(fd));
        legs_indicator[5]->move_leg_to_xyz(nodes2[0].x*(-1),thigh + hip-20,tibia -20);
        while (maestroGetMovingState(fd));
        legs_indicator[5]->move_leg_to_xyz(nodes2[0].x*(-1),nodes[0].y,nodes[0].z);
         while (maestroGetMovingState(fd));
        }


    if(probe==1)
    {   legs_indicator[0]->move_leg_to_xyz(0,121,73);
        while (maestroGetMovingState(fd));
        legs_indicator[0]->move_leg_to_xyz(0,111,103);
        while (maestroGetMovingState(fd));

        legs_indicator[1]->move_leg_to_xyz(0,36,78);
        while (maestroGetMovingState(fd));
        legs_indicator[1]->move_leg_to_xyz(0,36,103);
        while (maestroGetMovingState(fd));
        legs_indicator[2]->move_leg_to_xyz(0,36,78);
       while (maestroGetMovingState(fd));
        legs_indicator[2]->move_leg_to_xyz(0,36,103);
        while (maestroGetMovingState(fd));

        legs_indicator[3]->move_leg_to_xyz(0,121,73);
       while (maestroGetMovingState(fd));
        legs_indicator[3]->move_leg_to_xyz(0,111,103);
      while (maestroGetMovingState(fd));

         legs_indicator[4]->move_leg_to_xyz(0,121,73);
        while (maestroGetMovingState(fd));
        legs_indicator[4]->move_leg_to_xyz(0,111,103);
       while (maestroGetMovingState(fd));

        legs_indicator[5]->move_leg_to_xyz(0,36,78);
        while (maestroGetMovingState(fd));
        legs_indicator[5]->move_leg_to_xyz(0,36,103);
      while (maestroGetMovingState(fd));
    }
}




//!\fn tripod  Robot tripod gait. 3 possibilities of movement: forwards/backwards and turn
//!\param number The type of movement: 0- forwards/backwards, 1-sidewalk/turn,2-turn/sidewalk
//!\param nodes,nodes2 The given nodes of trajectory
//!\param m The number of samples,which divide leg trajectory by given value
//!\param k Number of loop iteration
//!\param factor_left_leg,factor_right_leg The factors, given by the user in order to control left or right leg step
//!
int robot::tripod(int number, vector <point3d> nodes, vector <point3d> nodes2, int m,int k,float factor_left_leg, float factor_right_leg)
{

	if (number == 0)

{       legs_indicator[0]->calc_for_trajectory_tripod(nodes, m, 0,factor_left_leg);
		legs_indicator[1]->calc_for_trajectory_tripod(nodes2, m, 1,factor_right_leg);
		legs_indicator[2]->calc_for_trajectory_tripod(nodes2, m, 1,factor_right_leg);
		legs_indicator[3]->calc_for_trajectory_tripod(nodes, m, 0,factor_left_leg);
		legs_indicator[4]->calc_for_trajectory_tripod(nodes, m, 0,factor_left_leg);
		legs_indicator[5]->calc_for_trajectory_tripod(nodes2, m, 1,factor_right_leg);

	}
	if (number == 1)

	{
        legs_indicator[0]->calc_for_trajectory_tripod(nodes, m, 0,factor_left_leg);
		legs_indicator[1]->calc_for_trajectory_tripod(nodes2, m, 1,factor_right_leg);
		legs_indicator[2]->calc_for_trajectory_tripod(nodes, m, 1,factor_right_leg);
		legs_indicator[3]->calc_for_trajectory_tripod(nodes2, m, 0,factor_left_leg);
		legs_indicator[4]->calc_for_trajectory_tripod(nodes, m, 0,factor_left_leg);
		legs_indicator[5]->calc_for_trajectory_tripod(nodes2, m, 1,factor_right_leg);
	}
    if(number ==2)
    {
        legs_indicator[0]->calc_for_trajectory_tripod(nodes, m, 0,factor_left_leg);
		legs_indicator[1]->calc_for_trajectory_tripod(nodes2, m, 1,factor_right_leg);
		legs_indicator[2]->calc_for_trajectory_tripod(nodes, m,1,factor_right_leg);
		legs_indicator[3]->calc_for_trajectory_tripod(nodes2, m, 0,factor_left_leg);
		legs_indicator[4]->calc_for_trajectory_tripod(nodes, m, 0,factor_left_leg);
		legs_indicator[5]->calc_for_trajectory_tripod(nodes2, m, 1,factor_right_leg);
        }
    for (int g = 0; g < k; g++)
	{
		for (int i = 0; i < legs_indicator[1]->traj_tripod.size(); i++)
		{
            legs_indicator[0]->move_leg_to_xyz(legs_indicator[0]->traj_tripod[i].x, legs_indicator[0]->traj_tripod[i].y, legs_indicator[0]->traj_tripod[i].z);
            legs_indicator[1]->move_leg_to_xyz(legs_indicator[1]->traj_tripod[i].x, legs_indicator[1]->traj_tripod[i].y, legs_indicator[1]->traj_tripod[i].z);
			legs_indicator[2]->move_leg_to_xyz(legs_indicator[2]->traj_tripod[i].x, legs_indicator[2]->traj_tripod[i].y, legs_indicator[2]->traj_tripod[i].z);
			legs_indicator[3]->move_leg_to_xyz(legs_indicator[3]->traj_tripod[i].x, legs_indicator[3]->traj_tripod[i].y, legs_indicator[3]->traj_tripod[i].z);
			legs_indicator[4]->move_leg_to_xyz(legs_indicator[4]->traj_tripod[i].x, legs_indicator[4]->traj_tripod[i].y, legs_indicator[4]->traj_tripod[i].z);
            legs_indicator[5]->move_leg_to_xyz(legs_indicator[5]->traj_tripod[i].x, legs_indicator[5]->traj_tripod[i].y, legs_indicator[5]->traj_tripod[i].z);
        while (maestroGetMovingState(fd));}}
	return 0;
}


//!\fn legs_speed Sets the desired speed for legs
//!\param speed The desired speed given by the user
//!\var legs_indicator Refers to each leg of robot
//!\fn Set_leg_speed Sets speed for one leg \sa set_leg_speed
//!
int robot::legs_speed(int speed)
{
	for (int j = 0; j < 6; j++)
	{
		legs_indicator[j]->set_leg_speed(j,speed);
	}
	return 0;
}


//!\fn move_legs_to_xyz Moves leg to given in milimeteres by the user x,y, z coordinates
//!\var legs_indicator Refers to each leg of robot
//!\var side_of_robot Left or right side of robot. 0 for left side, 1 for right side
//!\var hip,thigh,tibia The constant value of hip,thigh,tibia. It is: hip=29,thigh=57,tibia=108 milimeters
//!
int robot::move_legs_to_xyz(double x, double y, double z)
{

	for (int i = 0; i < 6; i++)
	if (legs_indicator[i]->side_of_robot == left_side)
    legs_indicator[i]->move_leg_to_xyz(0 - x, (thigh +hip) + y, (tibia)-z);
    else
	legs_indicator[i]->move_leg_to_xyz(0 - x, (thigh +hip) - y, (tibia)-z);

	return 0;
}



//!\fn zero It turns off robot legs
//!\var legs_indicator Refers to each leg of robot
//!\fn leg_poweroff It turns one leg off \sa leg_poweroff
//!
int robot::zero()
{

	for (int j = 0; j<6; j++)
    legs_indicator[j]->leg_poweroff();
    return 0;
}



//! class robot deconstructor
//! It closes the port connection
//!
robot::~robot()
{
	close(fd);

}

//!\fn robot_canon_posit Moves legs to canonical position. Sets servos value 0,0,90 degrees for each  leg
//!\var legs_indicator Refers to each leg of robot(/sa /class leg)
//!\fn canon_position Moves leg to canonical position. Sets servos value 0,0,90 degrees for one leg \sa canon_position
//!\fn usleep The break in miliseconds
//!\fn maestroGetMovingState This command is used to determine if the servo has reached their target or is still changing
//!\var fd The port variable, which is used to open connection with device
//!\fn forward_kinematics Calculates position xyz from given angles

int robot::robot_canon_posit()
{
    legs_speed(200);
	int j;
	for (j = 0; j < 6; j++)
	{
		 legs_indicator[j]->canon_position();

	}
	 while (maestroGetMovingState(fd));
	for (j=0;j<6;j++)
	{
	legs_indicator[j]->forward_kinematics(0,-20,135);
	usleep(2e4);
	legs_indicator[j]->forward_kinematics(0,-30,121);
    usleep(2e4);
	for (int i=0;i<30;i++)
	{legs_indicator[j]->forward_kinematics(0,-30+i,121-i);
    usleep(2e4);}}

return 0;
};

//!\fn walk Moves robot with desired  gait, direction, speed, samples and iteration
//!\param gait The type of gait: crawl or tripod
//!\param direction The direction of the robot gait:forward,backward, turn left or right, left or right sidewalk
//!\param samples Sample, which divide leg trajectory by given value. The default value for crawling is 18(it could not be changed), for tripod default value is 20
//!\param iteration The number of loop iteration
//!\param speed The desired speed of legs. The default value of speed is: for crawling 200 ms, for tripod 1000 ms
//!\param beta The factor, which the user could control the length of one step for each leg with
//!\param factor_left_leg,factor_right_leg The factors, given by the user in order to control left or right leg step
//! In order to enter default values for samples and speed, the user should give in their places 0 value
//!
void robot::walk(int gait , int direction ,int samples, int iteration, int speed,float beta,float factor_left_leg, float factor_right_leg)
{
    legs_speed(speed);
    create_default_nodes(direction,beta);               //!< create_default_nodes Contains default nodes and creates trajectories for different ways of movement \sa create_default_nodes

    if(gait==1)
    { if(speed==0) legs_speed(200);
    switch (direction)
  {case 0:
  crawling(forward_points,forward_points,iteration,factor_left_leg, factor_right_leg);
  break;
  case 1:
  crawling(backwards_points,backwards_points,iteration,factor_left_leg, factor_right_leg);
  break;
  case 2:
  crawling(turn_points_left,turn_points_right,iteration,factor_left_leg, factor_right_leg);
  break;
  case 3:
  crawling(turn_points_right,turn_points_left,iteration,factor_left_leg, factor_right_leg);
  break;
  case 4:
  crawling(sidewalk_left_points,sidewalk_right_points,iteration,factor_left_leg, factor_right_leg);
  break;
  case 5:
  crawling(sidewalk_right_points,sidewalk_left_points,iteration,factor_left_leg, factor_right_leg);
  break;
}};

    if(gait==0)
    {if(speed==0) legs_speed(1000);
    if(samples==0) samples=20;
    switch (direction)
    {
    case 0:
    tripod(0,forward_points,forward_points,samples,iteration,factor_left_leg,factor_right_leg);
    break;
     case 1:
    tripod(0,backwards_points,backwards_points,samples,iteration,factor_left_leg,factor_right_leg);
    break;
     case 2:
    tripod(1,turn_points_right,turn_points_left,samples,iteration,factor_left_leg,factor_right_leg);
    break;
     case 3:
    tripod(1,turn_points_left,turn_points_right,samples,iteration,factor_left_leg,factor_right_leg);
    break;
     case 4:
    tripod(1,sidewalk_right_points,sidewalk_left_points,samples,iteration,factor_left_leg,factor_right_leg);
    break;
    case 5:
    tripod(1,sidewalk_left_points,sidewalk_right_points,samples,iteration,factor_left_leg,factor_right_leg);
    break;
}}}



//!\fn walk_forwards Moves robot forward/backward with desired gait,calculated distances in class camera
//!\param gait The type of gait: crawl or tripod
//!\param dis_from_ball The calculated distance between camera and ball in class camera.The distance is given in centimeters(\sa \class camera)
//!\param speed The desired speed of legs. Default value of speed is: for crawling 200 ms, for tripod 1000 ms
//!\param factor_left_leg,factor_right_leg The factors, given by the user in order to control left or right leg step
//!\param beta The factor, which the user could control the length of one step for each leg with
//!\fn create_default_nodes Contains default nodes and creates trajectories for different ways of movement
//!\var remnant The calculated value from calculated distance from camera and factor beta multiplied by value of robot step
//!\fn walk Moves robot with desired  gait, direction, speed, samples and iteration \sa walk
//!\var size_of_step_forwards The size of step calculated in centimeters for one leg for forwards/backwards gait
//!\fn pos_for_walk Sets legs to position to walk forwards or sidewalk for tripod gait
//!
int robot::walk_forwards(int gait,double dis_from_ball,int speed, float beta,float factor_left_leg, float factor_right_leg)
{
    create_default_nodes(rdirection.forwards,beta);
    //!<Additional subtraction by 5 centimeters assures that robot will not pass the ball
    double remnant=std::fmod((dis_from_ball-5),size_of_step_forwards);
    cout<< "dis_from_ball: "<<(dis_from_ball-5)<<" ";
    cout<< "remnant: "<<remnant<<" ";
    cout<<"rozmiar kroku: "<<size_of_step_forwards<<" ";
    cout<<"kroki: "<<floor(((dis_from_ball)/size_of_step_forwards)*0.5)<<" ";
    cout<<"kroki2: "<<(remnant/size_of_step_forwards)*0.5<<" ";
    if(remnant==0)
    {
    if(gait==0) pos_for_walk(0,forward_points,forward_points,beta);
    usleep(1e5);
    walk(gait,rdirection.forwards,0,((dis_from_ball-5)/size_of_step_forwards)*0.5,speed,beta, factor_left_leg, factor_right_leg);}
    else
    {
    if(gait==0) pos_for_walk(0,forward_points,forward_points,beta);
    usleep(1e5);
    walk(gait,rdirection.forwards,0,floor(((dis_from_ball-5)/size_of_step_forwards)*0.5),speed,beta, factor_left_leg, factor_right_leg);
    usleep(1e5);
    walk(gait,rdirection.forwards,0,1,speed,(remnant/size_of_step_forwards)*0.5, factor_left_leg, factor_right_leg);}
    return 0;}




//!\fn walk_sidewalk Moves robot left or right  with desired  gait, calculated distance and value of probe in the class camera
//!\param gait The type of gait: crawl or tripod
//!\param probe Given value by the class camera.The value 0 indicates that robot will go right,1 indicates that robot will go left    \sa calculate_median
//!\param dis_from_ball The calculated distance in the class camera.The distance is given in centimeters         \sa calculate_median
//!\param speed The desired speed of legs. Default value of speed is: for crawling 200 ms, for tripod 1000 ms
//!\param factor_left_leg,factor_right_leg The factors, given by the user in order to control left or right leg step
//!\param beta The factor, which the user could control the length of one step for each leg with
//!\var remnant The calculated value between center of the screen and the ball and the factor beta multiplied by value of robot step
//!\fn walk Moves robot with desired  gait, direction, speed, samples and iteration \sa walk
//!\fn create_default_nodes Contains default nodes and creates trajectories for different ways of movement
//!\var size_of_step_sidewalk The size of step calculated in centimeters for one leg for sidewalk
//!\fn pos_for_walk Sets legs to position to walk forwards or sidewalk for tripod gait
//!
int robot::walk_sidewalk(int gait,int probe,double dis_from_cent,int speed,int prep_for_ball, float beta,float factor_left_leg, float factor_right_leg)
{   //!4-sidewalk_left_points,5-sidewalk_right_points
    create_default_nodes(4,beta);
    create_default_nodes(5,beta);
    double remnant;
    cout<< "dis_from_cent: "<<dis_from_cent<<" ";
    cout<< "remnant: "<<fmod((-dis_from_cent),size_of_step_sidewalk)<<" ";
    cout<<"rozmiar kroku: "<<size_of_step_sidewalk<<" ";
    cout<<"kroki: "<<floor((-dis_from_cent)/size_of_step_sidewalk)<<" ";
    cout<<"kroki2: "<<(remnant/size_of_step_sidewalk)<<" ";
    //!right side of robot
    if (probe==0)
        {remnant=std::fmod((-dis_from_cent),size_of_step_sidewalk);
        if(remnant==0)
    {
    if(gait==0) pos_for_walk(1,sidewalk_left_points,sidewalk_right_points,beta);
    usleep(1e5);
    walk(gait,rdirection.sidewalk_right,0,((-dis_from_cent)/size_of_step_sidewalk),speed,beta,factor_left_leg,factor_right_leg);}
    else
    {
    if(gait==0) pos_for_walk(1,sidewalk_left_points,sidewalk_right_points,beta);
    usleep(1e5);
     if(prep_for_ball==0)
     {
    walk(gait,rdirection.sidewalk_right,0,floor((-dis_from_cent)/size_of_step_sidewalk),speed,beta,factor_left_leg,factor_right_leg);
    cout<<"dis_from_cenetr "<<floor((-dis_from_cent)/size_of_step_sidewalk)<<endl;
    usleep(1e5);
    walk(gait,rdirection.sidewalk_right,0,1,speed,(remnant/size_of_step_sidewalk),factor_left_leg,factor_right_leg);
    cout<<"remnant: "<<remnant/(size_of_step_sidewalk)<<endl;
    usleep(1e5);}
   if(prep_for_ball==1)
   walk(gait,rdirection.sidewalk_left,0,2,speed,0.4,factor_left_leg,factor_right_leg);
   }}
   //!left side of robot
    else if(probe==1)
    {remnant=std::fmod((dis_from_cent),size_of_step_sidewalk);
    if(remnant==0)
    {
    if(gait==0) pos_for_walk(1,sidewalk_left_points,sidewalk_right_points,beta);
    usleep(1e5);
    walk(gait,rdirection.sidewalk_left,0,((dis_from_cent)/size_of_step_sidewalk),speed,beta,factor_left_leg,factor_right_leg);}
    else
    {
    if(gait==0) pos_for_walk(1,sidewalk_left_points,sidewalk_right_points,beta);
    usleep(1e5);
    if(prep_for_ball==0)
    {walk(gait,rdirection.sidewalk_left,0,floor((dis_from_cent)/size_of_step_sidewalk),speed,beta,factor_left_leg,factor_right_leg);
    usleep(1e5);
    walk(gait,rdirection.sidewalk_left,0,1,speed,(remnant/size_of_step_sidewalk),factor_left_leg,factor_right_leg);
    usleep(1e5);}
    if(prep_for_ball==1)
    walk(gait,rdirection.sidewalk_right,0,3,speed,0.4,factor_left_leg,factor_right_leg);
    }
    if(probe==3)
    walk(gait,rdirection.turn_left,0,1,speed,beta,factor_left_leg,factor_right_leg);
        }
return 0;}



//!\fn kick_the_ball It kicks the ball after the robot tracked it and moved towards its direction.
//!\param probe Given value by class camera.The value 0 indicates that robot will go right,1 indicates that robot will go left    \sa \class camera
//!\var legs_indicator Refers to each leg of robot
//!\fn set_servos_in_leg sets 3 servos in leg to desired values
//!\fn usleep The break in miliseconds
//!
void robot::kick_the_ball(int probe)
{
    legs_speed(1000);
    if (probe==0)
    {legs_indicator[0]->set_servos_in_leg(0,-25,10);
    usleep(5e5);
    legs_indicator[0]->set_servos_in_leg(45,25,0);
    usleep(5e5);
    legs_indicator[0]->set_servos_in_leg(-45,25,0);
    usleep(5e5);
    legs_indicator[0]->set_servos_in_leg(0,0,90);
    }
    else
    {legs_indicator[1]->set_servos_in_leg(0,-25,10);
    usleep(5e5);
    legs_indicator[1]->set_servos_in_leg(45,25,0);
    usleep(5e5);
    legs_indicator[1]->set_servos_in_leg(-45,25,0);
    usleep(5e5);
    legs_indicator[1]->set_servos_in_leg(0,0,90);
}}



//!\fn up_and_down The robot rises up in order to walk forwards or lowers down in order to turn around or go sideways
//!\param probe If 0 it lowers down , if 1 it rises up
//!\var legs_indicator Refers to each leg of robot
//!\fn move_leg_to_xyz inverse kinematics.Calculates the angles from given xyz values, and then moves leg to calculated angles
//!\var hip,thigh,tibia The constant value of hip,thigh,tibia. It is: hip=29,thigh=57,tibia=108 milimeters
//!\fn maestroGetMovingState This command is used to determine if the servo has reached their target or is still changing
//!\var fd The port variable, which is used to open connection with device
//!\fn usleep The break in miliseconds
//!
void robot::up_and_down(int probe)
    {if(probe==0)
    {for (int j=0;j<6;j++)
    legs_indicator[j]->move_leg_to_xyz(0,thigh + hip, tibia);
    while (maestroGetMovingState(fd));
    }
    else
    {for (int j=0;j<6;j++)
    legs_indicator[j]->move_leg_to_xyz(0,thigh + hip-30, tibia+30);
    while (maestroGetMovingState(fd));}}


//!\fn go_to_the_ball It moves robot to the ball using functions walk_forwards and walk_sidewalk, distances calculated in class camera and kicks the ball
//!\sa robot_canon_posit \sa \class camera \sa camera_film \sa up_and_dow \sa  walk_forwards \sa  walk_sidewalk \sa  kick_the_ball
//!\param gait The type of gait: crawl or tripod
//!\param speed The desired speed of legs. Default value of speed is: for crawling 200 ms, for tripod 1000 ms
//!\param beta The factor, which the user could control the length of one step for each leg with
//!\param factor_left_leg,factor_right_leg The factors, given by the user in order to control left or right leg step
//!\var dis_x,dis_y distances calculated in centimeters transferred by class camera
//!\fn robot_canon_posit Moves legs to canonical position. Sets servos value 0,0,90 degrees for each  leg
//!\fn camera_film It takes the images, calls the functions responsible for conversion of images and detecting ball
//!\fn usleep The break in miliseconds
//!\var contours The vector variable which helds the set of points of found object in class camera
//!\fn up_and_down The robot rises up in order to walk forwards or lowers down in order to turn around or go sideways
//!\fn walk_forwards Moves robot forwards/backwards with desired gait,calculated distances in class camera
//!\fn walk_sidewalk Moves robot left or right  with desired  gait, calculated distance and value of probe in the class camera
//!\fn kick_the_ball It kicks the ball after the robot tracked it and moved towards its direction
//!
 void robot::go_to_the_ball(int gait,int speed,float beta,float factor_left_leg, float factor_right_leg)
 {
    double dis_x=0, dis_y=0;
    while(1)
    {robot_canon_posit();
    usleep(1e5);
    cam->camera_film(5);
    usleep(1e5);
    if (cam->contours.size()!=0)
    {up_and_down(1);
    usleep(1e5);
   //!minimal safe distance to see the ball for second time is  30 cm for robot
    dis_x=((cam->distancex_cam)/10)-30;
   //!minimal safe distance to see the ball for second time is  6 cm right side
    dis_y=((cam->distancey_cam)/10);
   //! minimal safe distance to see the ball for second time is 10 cm left side
    //if(cam->distancey_cam<0)
  //  else dis_y=((cam->distancey_cam)/10)+10;
    cout<<"dis_x: "<<dis_x<<endl;
    cout<<"dis_y:"<<dis_y<<endl;
    walk_forwards(gait,dis_x,speed,beta,factor_left_leg,factor_right_leg);
    usleep(1e5);
    up_and_down(0);
    robot_canon_posit();
    usleep(1e5);
    walk_sidewalk(gait,cam->probe_indicator_for_camera,dis_y,speed,0);
    usleep(1e5);
    robot_canon_posit();
    cam->camera_film(5);
    if(cam->contours.size()!=0)
    {up_and_down(1);
    dis_x=((cam->distancex_cam)/10);
    dis_y=((cam->distancey_cam)/10);

    usleep(1e5);
    walk_forwards(gait,dis_x,speed,beta,factor_left_leg,factor_right_leg);
    usleep(1e5);
    up_and_down(0);
    robot_canon_posit();
    usleep(1e5);
    walk_sidewalk(gait,cam->probe_indicator_for_camera,dis_y,speed,1);
    usleep(1e5);
    robot_canon_posit();
    usleep(1e5);
    kick_the_ball(cam->probe_indicator_for_camera);
}
//!If ball is not detected, the robot will turn left in order to search for it
    else
    {up_and_down(1);
    usleep(1e5);
    walk(gait,rdirection.turn_left,0,1,speed, 0.8 ,factor_left_leg, factor_right_leg);
    usleep(1e5);
    up_and_down(0);}}
//!If ball is not detected, the robot will turn left in order to search for it
    else
    {up_and_down(1);
    usleep(1e5);
    walk(gait,rdirection.turn_left,0,1,speed, 0.8 ,factor_left_leg, factor_right_leg);
    usleep(1e5);
    up_and_down(0);}}}

