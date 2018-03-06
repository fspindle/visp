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
#include <vector>
#include <unistd.h> // TODO: remove

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_FRANKA)

#include <visp3/robot/vpRobotFranka.h>
#include <visp3/gui/vpPlot.h>

int main(int argc, char **argv)
{
  if (argc != 2) {
    std::cerr << "Usage: ./" << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  try {
    {
      vpColVector qd_d(7, 0);
      vpRobotFranka robot;
      robot.connect(argv[1]);

      vpColVector q;

      robot.getPosition(vpRobot::JOINT_STATE, q);
      std::cout << "Joint position: " << q.t() << std::endl;

      robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

      vpPlot plotter(1, 600, 600);
      plotter.initGraph(0, 7);
      for (size_t i = 0; i < 7; i++) {
        std::stringstream ss;
        ss << "q" + i;
        plotter.setLegend(0, i, ss.str());
      }

      double t0 = vpTime::measureTimeSecond();
//      qd_d[5] = vpMath::rad(-10.);
      qd_d[6] = vpMath::rad(5.);
      std::cout << "DBG: main() new vel sent to thread: " << qd_d.t() << std::endl;
      unsigned long iter = 0;

      do {
        robot.setVelocity(vpRobot::JOINT_STATE, qd_d);
        robot.getPosition(vpRobot::JOINT_STATE, q);
        if (iter < 3) std::cout << "Joint position: " << q.t() << std::endl;
        plotter.plot(0, iter++, q);

        vpTime::wait(10);
      } while (vpTime::measureTimeSecond() - t0 < 4.);
//      qd_d[5] = vpMath::rad(5.);
      qd_d[6] = vpMath::rad(-10.);
      std::cout << "DBG: main() new vel sent to thread: " << qd_d.t() << std::endl;
      robot.setVelocity(vpRobot::JOINT_STATE, qd_d);
      usleep(3*1000000);

      std::cout << "DBG: main() ask to stop the robot: " << qd_d.t() << std::endl;
      robot.setRobotState(vpRobot::STATE_STOP);

      std::cout << "End test" << std::endl;
      return 0;
    }

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

    // Move last joint +10 deg
    q[q.size() - 1] += vpMath::rad(10);

//    q[0] = 0;
//    q[1] = -M_PI_4;
//    q[2] = 0;
//    q[3] = -3 * M_PI_4;
//    q[4] = 0;
//    q[5] = M_PI_2;
//    q[6] = M_PI_4;
    std::cout << "Move to joint position: " << q.t() << std::endl;
    robot.setPosition(vpRobot::JOINT_STATE, q);

    robot.getPosition(vpRobot::JOINT_STATE, q);
    std::cout << "Reached joint position: " << q.t() << std::endl;

    // Move last joint -10 deg
    std::cout << "Move to joint position: " << q.t() << std::endl;
    q[q.size() - 1] -= vpMath::rad(10);
    robot.setPosition(vpRobot::JOINT_STATE, q);

    robot.getPosition(vpRobot::JOINT_STATE, q);
    std::cout << "Reached joint position: " << q.t() << std::endl;
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
