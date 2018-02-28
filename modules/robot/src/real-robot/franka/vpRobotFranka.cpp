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

#include <visp3/robot/vpRobotException.h>
#include <visp3/robot/vpRobotFranka.h>

#include "vpJointPosTrajGenerator_impl.h"
#include "vpJointVelTrajGenerator_impl.h"

/*!

  Default constructor.

*/
vpRobotFranka::vpRobotFranka()
  : vpRobot(), m_handler(NULL), m_positionningVelocity(20.), m_controlThread(), m_controlThreadRunning(false)
{
  init();
}

/*!
 * Establishes a connection with the robot.
 * \param[in] franka_address IP/hostname of the robot.
 * \param[in] realtime_config If set to kEnforce, an exception will be thrown if realtime priority cannot
 * be set when required. Setting realtime_config to kIgnore disables this behavior.
 */
vpRobotFranka::vpRobotFranka(const std::string &franka_address, franka::RealtimeConfig realtime_config)
  : vpRobot(), m_handler(NULL), m_positionningVelocity(20.), m_controlThread(), m_controlThreadRunning(false)
{
  init();
  connect(franka_address, realtime_config);
}

/*!
 * Initialize internal vars, such as min, max joint positions, max velocity and acceleration.
 */
void vpRobotFranka::init()
{
  nDof = 7;

  m_q_min   = std::array<double, 7> {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
  m_q_max   = std::array<double, 7> {12.8973, 1.7628, 2.8973, 0.0175, 2.8973, 3.7525, 2.8973};
  m_dq_max  = std::array<double, 7> {2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100};
  m_ddq_max = std::array<double, 7> {14.25, 7.125, 11.875, 11.875, 14.25, 19.0, 19.0};
}

/*!

  Destructor.

*/
vpRobotFranka::~vpRobotFranka()
{
  std::cout << "DBG: call destructor -----------------------" << std::endl;
  setRobotState(vpRobot::STATE_STOP);

//  if (m_controlThread.joinable()) {
//    m_controlThread.join();
//  }
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
      joint[i] = state.q_d[i];
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

void vpRobotFranka::setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &position)
{
  if (!m_handler) {
    throw(vpException(vpException::fatalError, "Cannot set Franka robot position: robot is not connected"));
  }
  if (vpRobot::STATE_POSITION_CONTROL != getRobotState()) {
    std::cout << "Robot was not in position-based control. "
                 "Modification of the robot state" << std::endl;
    setRobotState(vpRobot::STATE_POSITION_CONTROL);
  }

  double speed_factor = m_positionningVelocity / 100.;

  std::array<double, 7> q_goal;
  for (size_t i = 0; i < 7; i++) {
    q_goal[i] = position[i];
  }

  vpJointPosTrajGenerator joint_pos_traj_generator(speed_factor, q_goal);
  m_handler->control(joint_pos_traj_generator);
}

/*!

  Set the maximal velocity percentage to use for a position control.

  \param velocity : Percentage of the maximal velocity. Values should
  be in ]0:100].

*/
void vpRobotFranka::setPositioningVelocity(const double velocity)
{
  m_positionningVelocity = velocity;
}

/*!

  Change the robot state.

  \param newState : New requested robot state.
*/
vpRobot::vpRobotStateType vpRobotFranka::setRobotState(vpRobot::vpRobotStateType newState)
{
  switch (newState) {
  case vpRobot::STATE_STOP: {
    // Start primitive STOP only if the current state is Velocity
    if (vpRobot::STATE_VELOCITY_CONTROL == getRobotState()) {
      // Stop the robot
      std::cout << "DBG: ask to stop the thread setting m_controlThreadRunning = false" << std::endl;
      m_controlThreadRunning = false;
      if(m_controlThread.joinable()) {
        std::cout << "DBG: Stop joint vel thread to stop the robot" << std::endl;
        m_controlThread.join();
        std::cout << "DBG: control thread joined" << std::endl;
      }
    }
    break;
  }
  case vpRobot::STATE_POSITION_CONTROL: {
    if (vpRobot::STATE_VELOCITY_CONTROL == getRobotState()) {
      std::cout << "Change the control mode from velocity to position control.\n";
      // Stop the robot
      if(m_controlThread.joinable()) {
        std::cout << "DBG: Stop joint vel thread to swith to position control" << std::endl;
        m_controlThread.join();
        std::cout << "DBG: control thread joined" << std::endl;
      }
    }
    break;
  }
  case vpRobot::STATE_VELOCITY_CONTROL: {
    if (vpRobot::STATE_VELOCITY_CONTROL != getRobotState()) {
      std::cout << "Change the control mode from stop to velocity control.\n";
    }
    std::cout << "DBG: Start joint vel thread" << std::endl;
    //m_threadJointVel = std::thread(jointVel_thread);

    break;
  }
  default:
    break;
  }

  return vpRobot::setRobotState(newState);
}

void vpRobotFranka::setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel)
{
  if (vpRobot::STATE_VELOCITY_CONTROL != getRobotState()) {
    throw vpRobotException(vpRobotException::wrongStateError,
                           "Cannot send a velocity to the robot. "
                           "Use setRobotState(vpRobot::STATE_VELOCITY_CONTROL) first.");
  }

  {
    std::array<double, 7> dq_des;
    if (dq_des.size() != vel.size()) {
      throw vpRobotException(vpRobotException::wrongStateError,
                             "Joint velocity vector (%d) is not of size 7", vel.size());

    }
    for (size_t i = 0; i < dq_des.size(); i++) {
      dq_des[i] = vel[i];
    }

//    {
//      std::cout << "DBG: in vpRobotFranka::setVelocity() m_dq_max: ";
//      for(int i=0; i<7; i++) {
//        std::cout << m_dq_max[i] << " ";
//      }
//      std::cout << std::endl;
//    }

    if(! m_controlThreadRunning) {
      m_controlThread = std::thread(&vpJointVelTrajGenerator::control_thread, vpJointVelTrajGenerator(),
                                    m_handler, std::ref(m_controlThreadRunning), std::ref(dq_des),
                                    std::cref(m_q_min), std::cref(m_q_max), std::cref(m_dq_max), std::cref(m_ddq_max));
    }
  }
//  std::cout << "DBG: m_controlThreadRunning: " << m_controlThreadRunning << std::endl;

}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_robot.a(vpRobotFranka.cpp.o) has
// no symbols
void dummy_vpRobotFranka(){};
#endif

