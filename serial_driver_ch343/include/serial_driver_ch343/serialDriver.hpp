// C++ system
#include <atomic>
#include <functional>
#include <future>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

// C system
namespace rm_serial_driver
{

enum PkgState : uint8_t {
  COMPLETE = 0,
  HEADER_INCOMPLETE,
  PAYLOAD_INCOMPLETE,
  CRC_HEADER_ERRROR,
  CRC_PKG_ERROR,
  OTHER
};

enum StopBit : uint8_t { ONE = 0, ONE_POINT_FIVE, TWO };

enum Parity : uint8_t { NONE = 0, ODD, EVEN, MARK, SPACE };

class SerialConfig
{
public:
  SerialConfig() = delete;
  SerialConfig(int bps, int databit, bool flow, StopBit stopbits, Parity paritys)
  : baudrate(bps), databits(databit), flowcontrol(flow), stopbit(stopbits), parity(paritys)
  {
  }
  ~SerialConfig();

  char * devname = (char *)"/dev/ttyCH343USB0";
  int baudrate = 2000000;
  int databits = 8;
  bool flowcontrol = 0;
  StopBit stopbit = StopBit::ONE;
  Parity parity = Parity::NONE;
};

class Port
{
public:
  Port(std::shared_ptr<SerialConfig> ptr);
  ~Port();

  // port function
  int openPort();
  bool closePort();
  bool init();
  bool reopen();
  bool isPortInit();
  bool isPortOpen();

  //rx tx function
  int transmit(uint8_t * buff, int writeSize);
  int receive(uint8_t * buffer);
  int fd;

private:
  std::shared_ptr<SerialConfig> config;
  std::vector<char *> device_names = {
    "/dev/ttyCH343USB0", "/dev/ttyCH343USB1", "/dev/ttyCH343USB2"};
  int flags = 0;
  int num_per_read = 0;
  int num_per_write = 0;
  bool isinit = false;
  bool isopen = false;
  PkgState frameState;
};
}  // namespace rm_serial_driver
