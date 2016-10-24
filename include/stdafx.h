#ifndef STDAFX_H
#define STDAFX_H


#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <fstream>
#include <thread>
#include <algorithm>
#include <thread>

#ifdef _WIN32
#define O_NOCTTY 0
#else
#include <termios.h>
#endif
using namespace std;


# define left_side 0
# define right_side 1
#define wszystkie 2

//!\brief Class stdafx contains: libraries, constant physical leg value , tables of: servo number, minimal,medium,maximal values
//!tables of assigned servos number to each leg and indicator, which shows to which side leg belongs
//!\class stdafx, is inherited by each class in the project
//!\var hip,thigh,tibia The constant physical leg value given in milimeters
//!\var fd The port variable, which is used to open connection with device
//!tab_of_servos The table, which contains the number of sevo, minimal,medium,maximal value of servo given by the allowed pulse widths for the servo, in units of microseconds
//!tab_of_legs The table,which contains the number of leg, count of servos,which belong to each leg and the side of robot,which each leg belongs to
//!\sa stdfax class constructor
//!\sa ~stdafx class deconstructor
//!
class stdafx
{
    protected:

    double pi = 3.14159265358979323846;


    int hip=29;
    int thigh=57;
    int tibia=108 ;
    int fd;
 int tab_of_servos[18][4] = { { 0, 550, 1496, 2500 },
{ 1, 550, 2095, 2500 }, //
{ 2, 550, 967, 2500 },
{ 3, 550, 1535, 2500 },
{ 4, 550, 1395, 2500 },
{ 5, 550, 1123 + 90 * 10.3, 2500 },
{ 6, 550, 1566, 2500 },
{ 7, 550, 2127, 2500 },
{ 8, 550, 1933 - 90 * 10.3, 2500 },
{ 9, 550, 1465, 2500 },
{ 10, 550, 1370, 2500 },
{ 11, 550, 1176 + 90 * 10.3, 2500 },
{ 12, 550, 1545, 2500 },
{ 13, 550, 2150, 2500 },
{ 14, 550, 1958 - 90 * 10.3, 2500 },
{ 15, 550, 1600, 2500 },
{ 16, 550, 1450, 2500 },
{ 17, 550, 1079 + 90 * 10.3, 2500 } };


int tab_of_legs[6][4] = { { 0, 1, 2,right_side },
{ 3, 4, 5, left_side },
{ 6, 7, 8,right_side },
{ 9, 10, 11, left_side },
{ 12, 13, 14,right_side },
{ 15, 16, 17, left_side } };
        public:
        stdafx();
        virtual ~stdafx();


};

#endif // STDAFX_H

