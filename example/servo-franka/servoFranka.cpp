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
 *   tests the control law
 *   eye-in-hand control
 *   velocity computed in the camera frame
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example servoFranka.cpp

  Example of ...

*/

#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_FRANKA)

#include <visp3/robot/vpRobotFranka.h>

int main(int argc, char **argv)
{
  if (argc != 2) {
    std::cerr << "Usage: ./echo_robot_state <robot-hostname>" << std::endl;
    return -1;
  }

  try {
    // Test 1
    std::cout << "Start test 1" << std::endl;
    vpRobotFranka robot;
    robot.connect(argv[1]);

    vpColVector q;

    for (unsigned i = 0; i < 10; i++) {
      robot.getPosition(vpRobot::JOINT_STATE, q);
      std::cout << "Joint position: " << q.t() << std::endl;
      vpTime::wait(10);
    }

    vpPoseVector fPe;
    robot.getPosition(vpRobot::END_EFFECTOR_FRAME, fPe);
    std::cout << "fMe pose vector: " << fPe.t() << std::endl;
    std::cout << "fMe pose matrix: \n" << vpHomogeneousMatrix(fPe) << std::endl;

  }
  catch(const vpException &e) {
    std::cout << "Exception: " << e.what() << std::endl;
  }

  try {
    // Test 2
    std::cout << "Start test 2" << std::endl;
    vpRobotFranka robot(argv[1]);

    franka::Robot *handler = robot.getHandler();

    // Get end-effector cartesian position
    std::array<double, 16> pose = handler->readOnce().O_T_EE;
    vpHomogeneousMatrix oMee;
    for (unsigned int i=0; i< 4; i++) {
      for (unsigned int j=0; j< 4; j++) {
        oMee[i][j] = pose[j*4 + i];
      }
    }
    std::cout << "oMee: \n" << oMee << std::endl;

    // Get flange to end-effector frame transformation
    pose = handler->readOnce().F_T_EE;
    vpHomogeneousMatrix fMee;
    for (unsigned int i=0; i< 4; i++) {
      for (unsigned int j=0; j< 4; j++) {
        fMee[i][j] = pose[j*4 + i];
      }
    }
    std::cout << "fMee: \n" << fMee << std::endl;

    // Get end-effector to K frame transformation
    pose = handler->readOnce().EE_T_K;
    vpHomogeneousMatrix eeMk;
    for (unsigned int i=0; i< 4; i++) {
      for (unsigned int j=0; j< 4; j++) {
        eeMk[i][j] = pose[j*4 + i];
      }
    }
    std::cout << "eeMk: \n" << eeMk << std::endl;

    std::cout << "Start read callback" << std::endl;
    size_t count = 0;
    handler->read([&count](const franka::RobotState &robot_state) {
      // Printing to std::cout adds a delay. This is acceptable for a read
      // loop such as this, but should not be done in a control loop.
      std::cout << "Robot state:\n" << robot_state << std::endl;
      return false;
    });

    // Set Joint position
    std::cout << "Set joint position" << std::endl;
#if 0
    auto initial_position = handler->readOnce().q_d;
    franka::JointPositions qd = {{initial_position[0], initial_position[1], initial_position[2],
                                  initial_position[3], initial_position[4],
                                  initial_position[5], initial_position[6] + vpMath::rad(10)}};

    double time = 0.0;
    handler->control([=, &time](const franka::RobotState &, franka::Duration time_step) -> franka::JointPositions {
      time += time_step.toSec();

      return qd;
    });
#endif

#if 1
    handler->stop();
    auto initial_position = handler->readOnce().q_d;
    double time = 0.0;
    std::cout << "Start control thread" << std::endl;
    handler->control([=, &time](const franka::RobotState &, franka::Duration time_step) -> franka::JointPositions {
      time += time_step.toSec();

      double delta_angle = M_PI / 8 * (1 - std::cos(M_PI / 5.0 * time));

      franka::JointPositions output = {{initial_position[0], initial_position[1], initial_position[2],
                                        initial_position[3] + delta_angle, initial_position[4] + delta_angle,
                                        initial_position[5], initial_position[6] + delta_angle}};

      if (time >= 10.0) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(output);
      }
      std::cout << "In the control thread, time: " << time << std::endl;
      return output;
    });
    std::cout << "After start control thread" << std::endl;
#endif
#if 0

    double time_max = 4.0;
    double omega_max = 0.2;
    double time = 0.0;
    handler->control([=, &time](const franka::RobotState &, franka::Duration time_step) -> franka::JointVelocities {
      time += time_step.toSec();

      double cycle = std::floor(std::pow(-1.0, (time - std::fmod(time, time_max)) / time_max));
      double omega = cycle * omega_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * time));

      franka::JointVelocities velocities = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, omega}};

      if (time >= 2 * time_max) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(velocities);
      }
      return velocities;
    });
#endif

  }
  catch(const vpException &e) {
    std::cout << "Exception: " << e.what() << std::endl;
  }

  std::cout << "The end" << std::endl;
  return EXIT_SUCCESS;
}

#else
int main()
{
  std::cout << "ViSP is not build with libfranka..." << std::endl;
}
#endif
