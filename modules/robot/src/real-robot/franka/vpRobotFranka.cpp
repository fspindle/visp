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
  : vpRobot(), m_handler(NULL), m_positionningVelocity(20.), m_controlThread(), m_controlThreadRunning(false),
    m_q_min(), m_q_max(), m_dq_max(), m_ddq_max(), m_robot_state(),
    m_mutex(), m_dq_des()
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
  : vpRobot(), m_handler(NULL), m_positionningVelocity(20.), m_controlThread(), m_controlThreadRunning(false),
    m_q_min(), m_q_max(), m_dq_max(), m_ddq_max(), m_robot_state(),
    m_mutex()
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
 * Establishes a connection with the robot and set default behavior.
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

  m_handler->setCollisionBehavior(
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
  m_handler->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  m_handler->setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
  m_handler->setFilters(100, 100, 100, 100, 100);
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

  franka::RobotState robot_state = getRobotInternalState();

  switch(frame) {
  case JOINT_STATE: {
    joint.resize(nDof);
    for (int i=0; i < nDof; i++)
      joint[i] = robot_state.q_d[i];
    break;
  }
  default: {
    throw(vpException(vpException::fatalError, "Cannot get Franka cartesian position: wrong method"));
    break;
  }
  }
}

/*!
 * Given the joint position of the robot, computes the forward kinematics (direct geometric model) as an
 * homogeneous matrix \f${^f}{\bf M}_e\f$ that gives the position of the end-effector in the robot base frame.
 *
 * \param[in] q : Joint position as a 7-dim vector.
 * \return Position of the end-effector in the robot base frame.
 */
vpHomogeneousMatrix vpRobotFranka::get_fMe(const vpColVector &q)
{
  if (!m_handler) {
    throw(vpException(vpException::fatalError, "Cannot get Franka robot position: robot is not connected"));
  }
  if (q.size() != (unsigned int)nDof) {
    throw(vpException(vpException::fatalError, "Joint position is not a %d-dim vector", q.size()));
  }

  franka::Model model = m_handler->loadModel(); // TODO see if this function cost time

  std::array< double, 7 > q_array;
  for (size_t i = 0; i < (size_t)nDof; i++)
    q_array[i] = q[i];

  franka::RobotState robot_state = getRobotInternalState();

  std::array<double, 16> pose_array = model.pose(franka::Frame::kEndEffector, q_array, robot_state.F_T_EE, robot_state.EE_T_K);
  vpHomogeneousMatrix fMe;
  for (unsigned int i=0; i< 4; i++) {
    for (unsigned int j=0; j< 4; j++) {
      fMe[i][j] = pose_array[j*4 + i];
    }
  }

  return fMe;
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

  franka::RobotState robot_state = getRobotInternalState();

  switch(frame) {
  case END_EFFECTOR_FRAME: {
    std::array<double, 16> pose_array = robot_state.O_T_EE;
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

void vpRobotFranka::get_eJe(vpMatrix &eJe)
{
  if (!m_handler) {
    throw(vpException(vpException::fatalError, "Cannot get Franka robot position: robot is not connected"));
  }

  franka::RobotState robot_state = getRobotInternalState();

  franka::Model model = m_handler->loadModel();

  std::array<double, 42> jacobian = model.bodyJacobian(franka::Frame::kEndEffector, robot_state); // column-major
  eJe.resize(6, 7); // row-major
  for (size_t i = 0; i < 6; i ++) {
    for (size_t j = 0; j < 7; j ++) {
      eJe[i][j] = jacobian[j*6 + i];
    }
  }
  // TODO check from vpRobot fJe and fJeAvailable

}

void vpRobotFranka::get_fJe(vpMatrix &fJe)
{
  if (!m_handler) {
    throw(vpException(vpException::fatalError, "Cannot get Franka robot position: robot is not connected"));
  }

  franka::RobotState robot_state = getRobotInternalState();

  franka::Model model = m_handler->loadModel(); // TODO see if this function cost time

  std::array<double, 42> jacobian = model.zeroJacobian(franka::Frame::kEndEffector, robot_state); // column-major
  fJe.resize(6, 7); // row-major
  for (size_t i = 0; i < 6; i ++) {
    for (size_t j = 0; j < 7; j ++) {
      fJe[i][j] = jacobian[j*6 + i];
    }
  }
  // TODO check from vpRobot fJe and fJeAvailable
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
    if (m_dq_des.size() != vel.size()) {
      throw vpRobotException(vpRobotException::wrongStateError,
                             "Joint velocity vector (%d) is not of size 7", vel.size());
    }
    for (size_t i = 0; i < m_dq_des.size(); i++) {
      m_dq_des[i] = vel[i];
    }

    if(! m_controlThreadRunning) {
      m_controlThreadRunning = true;
      m_controlThread = std::thread(&vpJointVelTrajGenerator::control_thread, vpJointVelTrajGenerator(),
                                    std::ref(m_handler), std::ref(m_controlThreadRunning), std::ref(m_dq_des),
                                    std::cref(m_q_min), std::cref(m_q_max), std::cref(m_dq_max), std::cref(m_ddq_max),
                                    std::ref(m_robot_state), std::ref(m_mutex));
    }
  }
}

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

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_robot.a(vpRobotFranka.cpp.o) has
// no symbols
void dummy_vpRobotFranka(){};
#endif

