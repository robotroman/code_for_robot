#include "stdafx.h"

//!class stdafx constructor. Opens port fd in order to communicate with robot
//!
stdafx::stdafx()
{// Open the Maestro's virtual COM fd.
  const char * device = "/dev/ttyACM0";  // Windows, "\\\\.\\COM6" also works
  //const char * device = "/dev/ttyACM0";  // Linux
  //const char * device = "/dev/cu.usbmodem00034567";  // Mac OS X
fd = open(device, O_RDWR | O_NOCTTY);
  if (fd == -1)
  {
    perror(device);
    printf("error\n");
  }

#ifdef _WIN32
  _setmode(fd, _O_BINARY);
#else
  struct termios options;
  tcgetattr(fd, &options);
  options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
  options.c_oflag &= ~(ONLCR | OCRNL);
  options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tcsetattr(fd, TCSANOW, &options);
#endif

}


//!class stdafx deconstructor. It closes the port fd.
//!
stdafx::~stdafx()
{
    close(fd);
}
