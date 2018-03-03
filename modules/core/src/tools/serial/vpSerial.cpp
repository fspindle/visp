/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Serial communication.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <visp3/core/vpSerial.h>
#include <visp3/core/vpException.h>

#ifndef TIOCINQ
#  ifdef FIONREAD
#    define TIOCINQ FIONREAD
#  else
#    define TIOCINQ 0x541B
#  endif
#endif

vpSerial::vpSerial(const std::string &port, unsigned long baudrate,
                   bytesize_t bytesize, parity_t parity, stopbits_t stopbits,
                   flowcontrol_t flowcontrol)
  : m_port(port), m_fd(-1), m_is_open(false), m_xonxoff(false), m_rtscts(false),
    m_baudrate(baudrate), m_parity(parity),
    m_bytesize(bytesize), m_stopbits(stopbits), m_flowcontrol(flowcontrol)
{
  if (m_port.empty () == false)
    open();
}

vpSerial::~vpSerial()
{

}

int vpSerial::available()
{
  if (!m_is_open) {
    return 0;
  }
  int count = 0;
  if (-1 == ioctl (m_fd, TIOCINQ, &count)) {
    throw(vpException(vpException::fatalError, "Cannot check is serial port data available"));
  } else {
    return count;
  }
}

void vpSerial::close()
{
  if (m_is_open == true) {
    if (m_fd != -1) {
      int ret;
      ret = ::close (m_fd);
      if (ret == 0) {
        m_fd = -1;
      } else {
        throw(vpException(vpException::fatalError, "Cannot close serial port"));
      }
    }
    m_is_open = false;
  }
}

void vpSerial::open()
{
  if (m_port.empty ()) {
    throw(vpException(vpException::fatalError, "Serial port empty"));
  }
  if (m_is_open == true) {
    throw(vpException(vpException::fatalError, "Serial port already open"));
  }

  m_fd = ::open (m_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (m_fd == -1) {
    switch (errno) {
    case EINTR:
      // Try again because this is a recoverable error.
      open();
      return;
    case ENFILE:
    case EMFILE:
      throw(vpException(vpException::fatalError, "Serial port has to many handles open"));
    default:
      throw(vpException(vpException::fatalError, "Serial port opening error"));
    }
  }

  configure();
  m_is_open = true;
}

std::string vpSerial::read()
{

}

void vpSerial::write(const std::string &s)
{
  if (m_is_open == false) {
    throw(vpException(vpException::fatalError, "Serial port not opened"));
  }

  ssize_t r = ::write(m_fd, s.c_str(), s.size());
  if (r != (ssize_t)(s.size())) {
    throw(vpException(vpException::fatalError, "Serial port write error"));
  }
}

void vpSerial::configure()
{
  if (m_fd == -1) {
    throw(vpException(vpException::fatalError, "Serial port not opened"));
  }

  struct termios options;

  if (tcgetattr(m_fd, &options) == -1) {
    ::close(m_fd);
    throw vpException(vpException::fatalError, "Cannot get serial configuration");
  }

  // set up raw mode / no echo / binary
  options.c_cflag |= (tcflag_t)  (CLOCAL | CREAD);
  options.c_lflag &= (tcflag_t) ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL |
                                       ISIG | IEXTEN); //|ECHOPRT

  options.c_oflag &= (tcflag_t) ~(OPOST);
  options.c_iflag &= (tcflag_t) ~(INLCR | IGNCR | ICRNL | IGNBRK);

  speed_t baudrate;
  switch(m_baudrate) {
  case 9600: baudrate = B9600; break;
  case 19200: baudrate = B19200; break;
  case 38400: baudrate = B38400; break;
  case 76800: baudrate = B76800; break;
  }
  ::cfsetispeed(&options, baudrate);
  ::cfsetospeed(&options, baudrate);

  // setup char len
  options.c_cflag &= (tcflag_t) ~CSIZE;
  switch(m_bytesize) {
  case eightbits: options.c_cflag |= CS8; break;
  case sevenbits: options.c_cflag |= CS7; break;
  case sixbits:   options.c_cflag |= CS6; break;
  case fivebits:  options.c_cflag |= CS5; break;
  }

  switch(m_stopbits) {
  case stopbits_one: options.c_cflag &= (tcflag_t) ~(CSTOPB); break;
  case stopbits_two: options.c_cflag |= (CSTOPB); break;
  }

  // setup parity
  options.c_iflag &= (tcflag_t) ~(INPCK | ISTRIP);
  switch(m_parity) {
  case parity_none:
    options.c_cflag &= (tcflag_t) ~(PARENB | PARODD); break;
  case parity_even:
    options.c_cflag &= (tcflag_t) ~(PARODD);
    options.c_cflag |=  (PARENB); break;
  case parity_odd:
    options.c_cflag |=  (PARENB | PARODD); break;
  }

  // setup flow control
  switch(m_flowcontrol) {
  case flowcontrol_none:
    m_xonxoff = false;
    m_rtscts = false;
    break;
  case flowcontrol_software:
    m_xonxoff = true;
    m_rtscts = false;
    break;
  case flowcontrol_hardware:
    m_xonxoff = false;
    m_rtscts = true;
    break;
  }

  // xonxoff
  if (m_xonxoff)
    options.c_iflag |=  (IXON | IXOFF);
  else
    options.c_iflag &= (tcflag_t) ~(IXON | IXOFF);

  // rtscts
  if (m_rtscts)
    options.c_cflag |=  (CRTSCTS);
  else
    options.c_cflag &= (unsigned long) ~(CRTSCTS);

  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;

  // activate settings
  ::tcsetattr (m_fd, TCSANOW, &options);
}
