#define LASER_CLOSE_CFILE " #include <stdio.h>\n#include <termios.h>\n#include <fcntl.h>\n#include <strings.h>\n\
int main(void)\n\
{\n\
  int fd = open(\"/dev/ttyUSB"
#define LASER_CLOSE_CFILE_P2 "\", O_RDWR);\n\
  const char * buf = \"s0c\\n\";\n\
  int res;\n\
  struct termios newtio;\n\
      if(!isatty(fd)) \n\
	{\n \
	  printf(\"port %d is not a tty\\n\", fd);\n\
	  return -1; \n\
	}    \n\
      bzero(&newtio, sizeof(newtio));\n\
      if(cfsetispeed(&newtio, B19200) < 0 || cfsetospeed(&newtio, B19200) < 0)\n\
	{\n\
	  printf(\"Impossible to set baud rates of port %d\\n\", fd);\n\
	  return -1;\n\
	}\n  \
      newtio.c_cflag |= PARENB;\n\
      newtio.c_cflag &= ~PARODD;\n\
      newtio.c_cflag &=  ~CSTOPB;\n\
      newtio.c_cflag &= ~CSIZE;\n\
      newtio.c_cflag |= CS7;\n\
      newtio.c_cflag |= (CREAD | CLOCAL);\n\
      newtio.c_cflag &= ~CRTSCTS;\n\
      newtio.c_lflag = 0;\n\
      newtio.c_iflag |= IGNCR;\n\
      newtio.c_iflag |= IGNPAR; \n\
      newtio.c_iflag &= ~(IXON | IXOFF | IXANY);\n\
      newtio.c_oflag |= ONLCR;\n\
      newtio.c_cc[VMIN] = 13; \n\
      newtio.c_cc[VTIME] = 1; \n\
      tcflush( fd, TCIFLUSH );\n\
      if(tcsetattr(fd, TCSANOW, &newtio)<0)\n\
	{\n\
	  printf(\"Unable to apply given port settings\\n\");\n\
	  return -1;\n\
	}\n\
  res = write(fd, buf, 4);\n\
  if(close(fd)<0)\n\
    printf(\"Close port %d error in script\\n\", fd);\n\
  return 0;\n\
}\n "

