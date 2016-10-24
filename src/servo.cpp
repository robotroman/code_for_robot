#include "servo.h"


//! class servo constructor
//!\param ch Servo channel (0-17)
//!\param e Servo  minimal value
//!\param f Servo maximal value
//!\param g Servo medium value
//!\var a The factor by which values of left legs were multiplied in order to match to values of right legs
//!\sa channel
//!
servo::servo(int ch, double e, double f, double g)
{
	min = e;
	max = f;
	center = g;
	a = 10.3;
	r = 10.5;
	channel = ch;

}
//!empty  servo constructor
//! second possibility to use class servo
//!
servo::servo()
{
}


//!\fn maestroGetPosition Implements the Maestro's Get Position serial command.
//! channel: Channel number from 0 to 17
//! position: A pointer to the returned position value (for a servo channel, the units are quarter-milliseconds)
//! Returns servo response on success, -1 on failure.
//!\param fd Port of robot
//!\param Servo channel (0-18)
//!\var command It contains Polulu Compact Protocol Command (for this case 0x90) and channel number
//!\var response It contains the servo response for this command
//!
int servo::maestroGetPosition( int fd, unsigned char channel)
{
  unsigned char command[] = {0x90, channel};
  if(write(fd, command, sizeof(command)) == -1)
  {
    perror("error writing");
    return -1;
  }

  unsigned char response[2];
  if(read(fd,response,2) != 2)
  {
    perror("error reading");
    return -1;
  }

  return response[0] + 256*response[1];
};

//!\fn maestroSetTarget Implements the Maestro's Set Target serial command.
//! channel: Channel number from 0 to 18
//! target: Sets the target value (for a servo channel, the units are quarter-milliseconds)
//! Returns 0 on success, -1 on failure. */
//!\param fd Port of robot
//!\param target The servo value given by the user.
//!\var command It contains Polulu Compact Protocol Command (for this case 0x84), channel number and givrn by user servo value,
//! which will be sent in 2 bytes
//!\ref tab_of_servos Table of servos numbers(0-17), minimal,medium and miximal values
//!
int servo::maestroSetTarget(int fd, unsigned char channel, unsigned short target)
{
  unsigned char command[] = {0x84, channel, target & 0x7F, target >> 7 & 0x7F};  //!< Given servo value sent by 2 bytes
  if(target==0)
  {write(fd,command,sizeof(command));
  return 0;
  }
  if(target*0.25<tab_of_servos[channel][1])                                     //!< The condition to check if servo has exceeded minimal value assigned from table of servos
  {cout<<"servos limit min " <<channel<<endl;
  return -1;}
  else if(target*0.25>tab_of_servos[channel][3])                                //!< The condition to check if servo has exceeded maximal value assigned from table of servos
  { cout<<"servos limit max "<<channel<<endl;
  return -1;}
  if (write(fd, command, sizeof(command)) == -1)
  {
    perror("error writing");
    return -1;
  }
  return 0;
};

//!\fn maestroSetSpeed This command limits the speed at which a servo channel output value changes.
//! The speed limit is given in units of (0.25 μs)/(10 ms),
//!\param fd Port of robot
//!\param Servo channel (0-18)
//!\param target The servo value given by the user.
//!\var command It contains Polulu Compact Protocol Command (for this case 0x87), channel number and givrn by user servo value,
//! which will be sent in 2 bytes
//!
int servo::maestroSetSpeed(int fd, unsigned char channel, unsigned short target)
{
  unsigned char command[] = {0x87, channel, target & 0x7F, target >> 7 & 0x7F};
  if (write(fd,command, sizeof(command)) == -1)
  {
    perror("error writing");
    return -1;
  }
  return 0;
};



//!\fn go Sets the desired servo value
//! function returns 0 if succeeded, -1 if failure
//!\param b Given servo value
//!\var c The calculated servo value. It is calculated by adding to medium servo value the factor for leg multiplied by the given value
//!
int servo::go(int b)
{double c=0;
c = center + b*a;
int success;
 success=maestroSetTarget( fd, channel, c * 4);       //!<Sets desired servo value.Multilied by 4 due to the units,which are quarter-milliseconds
 if(success!=0)                                       //!< \sa maestroSetTarget
 {perror(("error in servo::go"));
 return -1;}
return 0;
};

//!\fn poweroff_servo Turns servo off
//!
int servo::poweroff_servo()
{int success=maestroSetTarget(fd, channel, 0);      //!< 0 value turns servo off \sa maestrosetTarget
    if(success!=0)
 {perror(("error in servo::poweroff_servo"));
 return -1;}
return 0;
};

//!class servo deconstructor
servo::~servo()
{
    //dtor
}
