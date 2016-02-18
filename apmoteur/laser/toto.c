 #include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <strings.h>
int main(void)
{
  int fd = open("/dev/ttyUSB0", O_RDWR);
  const char * buf = "s0c\n";
  int res;
  struct termios newtio;
      if(!isatty(fd)) 
	{
 	  printf("port %d is not a tty\n", fd);
	  return -1; 
	}    
      bzero(&newtio, sizeof(newtio));
      if(cfsetispeed(&newtio, B19200) < 0 || cfsetospeed(&newtio, B19200) < 0)
	{
	  printf("Impossible to set baud rates of port %d\n", fd);
	  return -1;
	}
        newtio.c_cflag |= PARENB;
      newtio.c_cflag &= ~PARODD;
      newtio.c_cflag &=  ~CSTOPB;
      newtio.c_cflag &= ~CSIZE;
      newtio.c_cflag |= CS7;
      newtio.c_cflag |= (CREAD | CLOCAL);
      newtio.c_cflag &= ~CRTSCTS;
      newtio.c_lflag = 0;
      newtio.c_iflag |= IGNCR;
      newtio.c_iflag |= IGNPAR; 
      newtio.c_iflag &= ~(IXON | IXOFF | IXANY);
      newtio.c_oflag |= ONLCR;
      newtio.c_cc[VMIN] = 13; 
      newtio.c_cc[VTIME] = 1; 
      tcflush( fd, TCIFLUSH );
      if(tcsetattr(fd, TCSANOW, &newtio)<0)
	{
	  printf("Unable to apply given port settings\n");
	  return -1;
	}
  res = write(fd, buf, 4);
  if(close(fd)<0)
    printf("Close port %d error in script\n", fd);
  return 0;
}
 