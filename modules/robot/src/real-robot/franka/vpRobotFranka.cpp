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

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_FRANKA

#include <visp3/robot/vpRobotFranka.h>

/*!

  Default constructor.

*/
vpRobotFranka::vpRobotFranka()
  : vpRobot(), m_handler(NULL)
{
}

/*!
 * Establishes a connection with the robot.
 * \param[in] franka_address IP/hostname of the robot.
 * \param[in] realtime_config If set to kEnforce, an exception will be thrown if realtime priority cannot
 * be set when required. Setting realtime_config to kIgnore disables this behavior.
 */
vpRobotFranka::vpRobotFranka(const std::string &franka_address, franka::RealtimeConfig realtime_config)
  : m_handler(NULL)
{
  connect(franka_address, realtime_config);
}


/*!

  Destructor.

*/
vpRobotFranka::~vpRobotFranka()
{
  if (m_handler)
    delete m_handler;
}

/*!
 * Establishes a connection with the robot.
 * \param[in] franka_address IP/hostname of the robot.
 * \param[in] realtime_config If set to kEnforce, an exception will be thrown if realtime priority cannot
 * be set when required. Setting realtime_config to kIgnore disables this behavior.
 */
void vpRobotFranka::connect(const std::string &franka_address, franka::RealtimeConfig realtime_config)
{
  init();
  if(franka_address.empty()) {
    throw(vpException(vpException::fatalError, "Cannot connect Franka robot: IP/hostname is not set"));
  }
  if (m_handler)
    delete m_handler;

  m_handler = new franka::Robot(franka_address, realtime_config);
}

/*!
 * Get robot joint position.
 * \param[in] frame Type of position to retrieve. Value could be only vpRobot::JOINT_SPACE.
 * \param[out] joint Robot joint position. This vector is 7-dim.
 *
 * If you want to get a cartesian position, use rather
 * getPosition(const vpRobot::vpControlFrameType, vpPoseVector &)
 */
void vpRobotFranka::getPosition(const vpRobot::vpControlFrameType frame, vpColVector &joint)
{
  if (!m_handler) {
    throw(vpException(vpException::fatalError, "Cannot get Franka robot position: robot is not connected"));
  }

  franka::RobotState state = m_handler->readOnce();
  switch(frame) {
  case JOINT_STATE: {
    joint.resize(nDof);
    for (int i=0; i < nDof; i++)
      joint[i] = state.q[i];
    break;
  }
  default: {
    throw(vpException(vpException::fatalError, "Cannot get Franka cartesian position: wrong method"));
    break;
  }
  }
}

/*!
 * Get robot cartesian position.
 * \param[in] frame Type of cartesian position to retrieve.
 * \param[out] pose Robot cartesian position.
 * - If \e frame = vpRobot::END_EFFECTOR_FRAME : return the fMe position as a pose vector
 *   (translation + \f$\theta_u\f$)
 */
void vpRobotFranka::getPosition(const vpRobot::vpControlFrameType frame, vpPoseVector &pose)
{
  if (!m_handler) {
    throw(vpException(vpException::fatalError, "Cannot get Franka robot position: robot is not connected"));
  }

  franka::RobotState state = m_handler->readOnce();
  switch(frame) {
  case END_EFFECTOR_FRAME: {
    std::array<double, 16> pose_array = m_handler->readOnce().O_T_EE;
    vpHomogeneousMatrix fMe;
    for (unsigned int i=0; i< 4; i++) {
      for (unsigned int j=0; j< 4; j++) {
        fMe[i][j] = pose_array[j*4 + i];
      }
    }
    pose.buildFrom(fMe);
    break;
  }
  default: {
    throw(vpException(vpException::fatalError, "Cannot get Franka cartesian position: not implemented"));
    break;
  }
  }
}

/*!
  Initialise internal vars.
 */
void vpRobotFranka::init()
{
  nDof = 7;
}


#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_robot.a(vpRobotFranka.cpp.o) has
// no symbols
void dummy_vpRobotFranka(){};
#endif

