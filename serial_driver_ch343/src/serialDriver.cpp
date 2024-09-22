#include "serial_driver_ch343/serialDriver.hpp"

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#define termios asmtermios
#include <asm/termios.h>
#undef termios
#include <termios.h>

namespace rm_serial_driver
{
extern "C" int ioctl(int d, int request, ...);

Port::Port(std::shared_ptr<SerialConfig> ptr) { config = ptr; }

bool Port::init()
{
  // init port
  struct termios newtio;
  struct termios oldtio;
  bzero(&newtio, sizeof(newtio));
  bzero(&oldtio, sizeof(oldtio));

  if (tcgetattr(fd, &oldtio) != 0) {
    perror("tcgetattr");
    return false;
  }
  newtio.c_cflag |= CLOCAL | CREAD;
  newtio.c_cflag &= ~CSIZE;

  /* set data bits */
  switch (config->databits) {
    case 5:
      newtio.c_cflag |= CS5;
      break;
    case 6:
      newtio.c_cflag |= CS6;
      break;
    case 7:
      newtio.c_cflag |= CS7;
      break;
    case 8:
      newtio.c_cflag |= CS8;
      break;
    default:
      fprintf(stderr, "unsupported data size\n");
      return false;
  }
  /* set parity */
  switch (config->parity) {
    case Parity::NONE:
      newtio.c_cflag &= ~PARENB; /* Clear parity enable */
      newtio.c_iflag &= ~INPCK;  /* Disable input parity check */
      break;
    case Parity::ODD:
      newtio.c_cflag |= (PARODD | PARENB); /* Odd parity instead of even */
      newtio.c_iflag |= INPCK;             /* Enable input parity check */
      break;
    case Parity::EVEN:
      newtio.c_cflag |= PARENB;  /* Enable parity */
      newtio.c_cflag &= ~PARODD; /* Even parity instead of odd */
      newtio.c_iflag |= INPCK;   /* Enable input parity check */
      break;
    case Parity::MARK:
      newtio.c_cflag |= PARENB; /* Enable parity */
      newtio.c_cflag |= CMSPAR; /* Stick parity instead */
      newtio.c_cflag |= PARODD; /* Even parity instead of odd */
      newtio.c_iflag |= INPCK;  /* Enable input parity check */
      break;
    case Parity::SPACE:
      newtio.c_cflag |= PARENB;  /* Enable parity */
      newtio.c_cflag |= CMSPAR;  /* Stick parity instead */
      newtio.c_cflag &= ~PARODD; /* Even parity instead of odd */
      newtio.c_iflag |= INPCK;   /* Enable input parity check */
      break;
    default:
      fprintf(stderr, "unsupported parity\n");
      return false;
  }

  /* set stop bits */
  switch (config->stopbit) {
    case StopBit::ONE:
      newtio.c_cflag &= ~CSTOPB;
      break;
    case StopBit::TWO:
      newtio.c_cflag |= CSTOPB;
      break;
    default:
      perror("unsupported stop bits\n");
      return false;
  }

  if (config->flowcontrol)
    newtio.c_cflag |= CRTSCTS;
  else
    newtio.c_cflag &= ~CRTSCTS;

  newtio.c_cc[VTIME] = 10; /* Time-out value (tenths of a second) [!ICANON]. */
  newtio.c_cc[VMIN] = 0;   /* Minimum number of bytes read at once [!ICANON]. */
  tcflush(fd, TCIOFLUSH);

  if (tcsetattr(fd, TCSANOW, &newtio) != 0) {
    perror("tcsetattr");
    return false;
  }

  struct termios2 tio;

  if (ioctl(fd, TCGETS2, &tio)) {
    perror("TCGETS2");
    return false;
  }

  tio.c_cflag &= ~CBAUD;
  tio.c_cflag |= BOTHER;
  tio.c_ispeed = config->baudrate;
  tio.c_ospeed = config->baudrate;

  if (ioctl(fd, TCSETS2, &tio)) {
    perror("TCSETS2");
    return false;
  }

  if (ioctl(fd, TCGETS2, &tio)) {
    perror("TCGETS2");
    return false;
  }
  isinit = true;
  return true;
}

int Port::openPort()
{
  for (auto device_name : device_names) {
    fd = open(device_name, O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd < 0) {
      perror("open device failed");
      isopen = false;
    } else {
      config->devname = device_name;
      break;
    }
  }
  // fd = open(config->devname, O_RDWR | O_NOCTTY | O_NDELAY);

  // if (fd < 0) {
  //   perror("open device failed");
  //   isopen = false;
  // }

  flags = fcntl(fd, F_GETFL, 0);
  flags &= ~O_NONBLOCK;

  if (fcntl(fd, F_SETFL, flags) < 0) {
    // printf("fcntl failed.\n");
    std::cout << "fcntl failed.\n" << std::endl;
    isopen = false;
  }

  if (isatty(fd) == 0) {
    // printf("not tty device.\n");
    std::cout << "not tty device.\n" << std::endl;
    isopen = false;
  }

  else {
    // printf("tty device test ok.\n");
    std::cout << "tty device test ok.\n" << std::endl;
    isopen = true;
    init();
  }

  return fd;
}

int Port::transmit(uint8_t * buff, int writeSize)
{
  int num_per_write = write(fd, buff, writeSize);
  if (num_per_write > 0)
    return num_per_write;
  else
    return -1;
}

int Port::receive(uint8_t * buffer)
{
  //do not change the 64 -> size of the usb driver.
  num_per_read = read(fd, buffer, 64);
  return num_per_write;
}

bool Port::closePort()
{
  isopen = false;
  return close(fd);
}

bool Port::reopen()
{
  if (isPortOpen()) closePort();

  if (openPort()) return true;

  return false;
}

bool Port::isPortInit()
{
  if (isinit)
    return true;
  else
    return false;
}

bool Port::isPortOpen()
{
  if (isopen)
    return true;
  else
    return false;
}

Port::~Port() {}
SerialConfig::~SerialConfig() {}

}  // namespace rm_serial_driver
