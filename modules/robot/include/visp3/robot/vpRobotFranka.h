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
 * Interface for the Franka robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef __vpRobotFranka_h__
#define __vpRobotFranka_h__

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_FRANKA

#include <iostream>
#include <thread>
#include <atomic>
#include <vector>

#include <stdio.h>

#include <franka/exception.h>
#include <franka/robot.h>

#include <visp3/core/vpColVector.h>
#include <visp3/robot/vpRobot.h>
#include <visp3/core/vpException.h>

/*!
  \class vpRobotFranka

  \ingroup group_robot_real_arm

  \brief Control of Franka robot.

*/
class VISP_EXPORT vpRobotFranka : public vpRobot
{
private:
  /*!
    Copy constructor not allowed.
   */
  vpRobotFranka(const vpRobotFranka &robot);
  /*!
    This function is not implemented.
   */
  void get_eJe(vpMatrix &) {};
  /*!
    This function is not implemented.
   */
  void get_fJe(vpMatrix &) {};
  /*!
    This function is not implemented.
   */
  void getDisplacement(const vpRobot::vpControlFrameType frame, vpColVector &q) {};
  inline franka::RobotState getRobotInternalState(); // Should be absolutely inline
  void init();

  franka::Robot *m_handler; //!< Robot handler
  double m_positionningVelocity;

  std::thread m_controlThread;
  std::atomic_bool m_controlThreadRunning;

  std::array<double, 7> m_q_min;    // Joint min position
  std::array<double, 7> m_q_max;    // Joint max position
  std::array<double, 7> m_dq_max;   // Joint max velocity
  std::array<double, 7> m_ddq_max;  // Joint max acceleration

  franka::RobotState m_robot_state; // Robot state protected by mutex
  std::mutex m_mutex;               // Mutex to protect m_robot_state

public:
  vpRobotFranka();

  vpRobotFranka(const std::string &franka_address,
                franka::RealtimeConfig realtime_config = franka::RealtimeConfig::kEnforce);

  virtual ~vpRobotFranka();

  void connect(const std::string &franka_address,
               franka::RealtimeConfig realtime_config = franka::RealtimeConfig::kEnforce);

  /*!
   * Get robot handler to access native libfranka functions.
   *
   * \return Robot handler if it exists, an exception otherwise.
   */
  franka::Robot *getHandler() {
    if (!m_handler) {
      throw(vpException(vpException::fatalError, "Cannot get Franka robot handler: robot is not connected"));
    }

    return m_handler;
  }

  void getPosition(const vpRobot::vpControlFrameType frame, vpColVector &joint);
  void getPosition(const vpRobot::vpControlFrameType frame, vpPoseVector &pose);

  void setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &position);
  void setPositioningVelocity(const double velocity);

  vpRobot::vpRobotStateType setRobotState(vpRobot::vpRobotStateType newState);
  void setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel);
};

// Should be absolutely inline
franka::RobotState vpRobotFranka::getRobotInternalState()
{
  if (!m_handler) {
    throw(vpException(vpException::fatalError, "Cannot get Franka robot state: robot is not connected"));
  }
  franka::RobotState robot_state;
  if (! m_controlThreadRunning) {
    robot_state = m_handler->readOnce();

    std::lock_guard<std::mutex> lock(m_mutex);
    m_robot_state = robot_state;
  }
  else { // robot_state is updated in the velocity control thread
    std::lock_guard<std::mutex> lock(m_mutex);
    robot_state = m_robot_state;
  }

  return robot_state;
}

#endif
#endif // #ifndef __vpRobotFranka_h__
