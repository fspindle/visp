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

#ifndef __vpSerial_h_
#define __vpSerial_h_

#include <string>

#include <visp3/core/vpConfig.h>

class VISP_EXPORT vpSerial
{
public:
  /*!
   * Defines the possible byte sizes for the serial port.
   */
  typedef enum {
    fivebits = 5,
    sixbits = 6,
    sevenbits = 7,
    eightbits = 8
  } bytesize_t;

  /*!
   * Defines the possible parity types for the serial port.
   */
  typedef enum {
    parity_none = 0,
    parity_odd = 1,
    parity_even = 2
  } parity_t;

  /*!
   * Defines the possible stopbit types for the serial port.
   */
  typedef enum {
    stopbits_one = 1,
    stopbits_two = 2,
  } stopbits_t;

  /*!
   * Defines the possible flowcontrol types for the serial port.
   */
  typedef enum {
    flowcontrol_none = 0,
    flowcontrol_software,
    flowcontrol_hardware
  } flowcontrol_t;

  vpSerial(const std::string &port, unsigned long baudrate = 9600,
           bytesize_t bytesize = eightbits, parity_t parity = parity_none, stopbits_t stopbits = stopbits_one,
           flowcontrol_t flowcontrol = flowcontrol_none);
  ~vpSerial();

  int available();
  void close();
  void open();
  std::string read();
  void write(const std::string &s);

private:
  void configure();

  std::string m_port;
  int m_fd;

  bool m_is_open;
  bool m_xonxoff;
  bool m_rtscts;

  unsigned long m_baudrate;

  parity_t m_parity;
  bytesize_t m_bytesize;
  stopbits_t m_stopbits;
  flowcontrol_t m_flowcontrol;

};

#endif
