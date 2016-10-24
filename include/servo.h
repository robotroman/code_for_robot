#ifndef SERVO_H
#define SERVO_H
#include "stdafx.h"

//!\brief The class servo is the lowest class and contains commands to manage only  one servo
//!\brief It includes commands: \sa go \sa poweroff_servo \sa maestroGetPosition \sa maestroSetTarget \sa maestroSetSpeed
//!\sa servo class constructors
//!\sa ~servo servo class deconstructor
//!\class servo inherites from class stdafx  \sa stdafx
//!
class servo:public stdafx
{
    private:
    int channel;                                                                    //!< servo channel (0-17)
	double a;												                        //!< multipier for left side servos
	double min;												                        //!< minimal value of servo position determined by pulse width in microseconds
	double max;												                        //!< maximal value of servo position determined by pulse width in microseconds
	double center;	                                                                //!< medium value of servo position determined by pulse width in microseconds
    double r;												                        //!< multiplier for right side servos
	public:
	int go(int b);										                            //!< sets servo to desired position
    servo(int ch, double min, double max, double center);	                        //!< first class constructor
    int poweroff_servo();									                        //!< turns off  servo
    servo();                                                                        //!< second class constructor
    int maestroGetPosition(int fd, unsigned char channel);                          //!< returnes the position value (for a servo channel, the units are quarter-milliseconds)
    int maestroSetTarget(int fd, unsigned char channel, unsigned short target);     //!< sets the  target value (for a servo channel, the units are quarter-milliseconds)
	int maestroSetSpeed(int fd, unsigned char channel, unsigned short target);      //!< limits the speed at which a servo channel output value changes.
    virtual ~servo();                                                               //!< servo class deconstructor

};

#endif // SERVO_H
