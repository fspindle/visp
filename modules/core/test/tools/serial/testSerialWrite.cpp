#include <iostream>
#include <stdlib.h>

#include <visp3/core/vpSerial.h>

int main(int argc, char **argv)
{
  std::string port;

  unsigned long baud = 9600;
  for (int i = 0; i < argc; i++) {
    if (std::string(argv[i]) == "--port")
      port = std::string(argv[i + 1]);
    else if (std::string(argv[i]) == "--baud") {
      baud = (unsigned long)atol(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--help") {
      std::cout << "\nUsage: " << argv[0] << " [--port <serial name>] [--baud <baud rate>] [--help]\n" << std::endl;
      return 0;
    }
  }

  if (port.empty()) {
    std::cout << "\nSerial port not specified." << std::endl;
    std::cout << "\nUsage: " << argv[0] << " [--port <serial name>] [--baud <baud rate>] [--help]\n" << std::endl;
    return 0;
  }

  std::cout << "Try to connect to port \"" << port << "\" with baud rate " << baud << std::endl;
  vpSerial serial(port, baud);

  serial.write("hello\n");
}
