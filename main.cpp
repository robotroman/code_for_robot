#include "robot.h"


#include "camera.h"
using namespace std;
using namespace cv;



int main(int argc,char **argv)
{robot r;
//camera cam;
//r.robot_canon_posit();
//r.up_and_down(1);
////r.walk_forwards(r.rgait.tripod,42,30,0.8);

r.zero();
////getchar();
//r.go_to_the_ball(r.rgait.tripod,50,0.8);
////usleep(1e6);
////r.legs_indicator[0]->move_leg_to_xyz(0,0,r.tibia);
////	//usleep(1e5);
////r.legs_indicator[1]->move_leg_to_xyz(0,r.hip+r.thigh,r.tibia);
//r.legs_indicator[1]->forward_kinematics(0,-30,121);
//getchar();
//for(int i =0;i<30;i++)
//{r.legs_indicator[1]->forward_kinematics(0,-30+i,121-i);
//usleep(1e5);
//}
////
////
////	pos1 = e*sin(-a*pi/180);
////r.legs_indicator[2]->move_leg_to_xyz(pos1,pos2,pos3);
//
//	//pos2 = c*cos(pi1);
//  // pos3=thigh*sin(pi2)+tibia*sin(pi3+pi2);
//     //pos2=hip+thigh*cos(pi2)+tibia*sin(pi3-90+pi2);
////cout<<"pos1,pos2,pos3"<<endl;
////cout<<"\n\rx:"<<setprecision(3)<<pos1<<" ";
////cout<<"y:"<<setprecision(3)<<pos2<<" ";
////cout<<"z:"<<setprecision(3)<<pos3<<endl;
//
//
////r.legs_indicator[2]->move_leg_to_xyz(pos1,pos2,pos3);
//
//
//if(cyfra=='d')
//{
//
//  r.legs_indicator[0]->move_leg_to_xyz(50,r.thigh + r.hip, r.tibia);
//    usleep(1e6);
//r.legs_indicator[0]->move_leg_to_xyz(-50, r.thigh + r.hip,r.tibia);
//    usleep(1e6);
//r.legs_indicator[0]->move_leg_to_xyz(-20, r.thigh + r.hip, r.tibia - 30);
//    usleep(1e6);
//r.legs_indicator[0]->move_leg_to_xyz(20,r.thigh + r.hip,r.tibia - 30);
//    usleep(1e6);
//
//
//}
//}

////
//////
//////}
////
////





//}
//r.legs_indicator[0]->forward_kinematics(0,60,-87.3);

//r.legs_indicator[0]->forward_kinematics(0,60,-87.3);

//r.legs_indicator[0]->forward_kinematics(0,0,0);
//r.robot_canon_posit();
//r.legs_indicator[0]->set_servos_in_leg(0,0,90);
//usleep(1e6);/for (int i=5;i<40;i++)
//for (int i=0;i<50;i+=5)
//{
////r.legs_indicator[0]->forward_kinematics(0,0+i, -90);
////usleep(1e5);
//r.legs_indicator[2]->move_leg_to_xyz(0,-10, 90);
//usleep(1e5);
//}
//r.legs_indicator[0]->move_leg_to_xyz(0,0,0);
//usleep(1e5);


//r.legs_indicator[1]->servos[1]->go(100);

//usleep(10000000);


//w druga strone ruszanie sidewalk
//r.legs_indicator[5]->move_leg_to_xyz(0,r.thigh + r.hip-20, r.tibia+10);
// usleep(10000000);
// r.legs_indicator[5]->move_leg_to_xyz(0,r.thigh + r.hip+40, r.tibia-10);
//  usleep(10000000);
//   r.legs_indicator[5]->move_leg_to_xyz(0,r.thigh + r.hip, r.tibia-30);


//r.legs_indicator[0]->move_leg_to_xyz(0,r.thigh + r.hip+40, r.tibia-10);
 //second move
//r.legs_indicator[0]->move_leg_to_xyz(0,r.thigh + r.hip, r.tibia);
//third move

//r.legs_indicator[0]->move_leg_to_xyz(0,r.thigh + r.hip+20, r.tibia-10);


//r.legs_indicator[0]->servos[0]->maestroSetTarget(r.fd,1,40);
//r.legs_indicator[0]->servos[0]->maestroGetPosition(r.fd,1);
//  int position = r.legs_indicator[0]->servos[0]->maestroGetPosition(r.fd,1);
 // printf("Current position is %d.\n", position);

 //pthread_t t;
//  pthread_create(&t,NULL,&robot::robot_canon_posit,NULL);
  //  pthread_join(t,NULL);

 //  r.walk(r.tripod,r.sidewalk_left,12,15,1000);
      //  r.fastwalk(1,nodes_left_side,nodes_right_side,4ĵĵĵĵĵĵĵĵĵĵĵĵ0,10);


//  r.zero();
















    return 0;
}
