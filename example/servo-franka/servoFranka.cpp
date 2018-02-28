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
      robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

      double t0 = vpTime::measureTimeSecond();
//    qd_d[5] = vpMath::rad(-10.);
      qd_d[6] = vpMath::rad(10.);
      std::cout << "DBG: main() new vel sent to thread: " << qd_d.t() << std::endl;
      do {
        robot.setVelocity(vpRobot::JOINT_STATE, qd_d);
        vpTime::wait(10);
      } while (vpTime::measureTimeSecond() - t0 < 4.);
//      usleep(1*1000000);
      qd_d[5] = vpMath::rad(10.);
      qd_d[6] = vpMath::rad(10.);
      std::cout << "DBG: main() new vel sent to thread: " << qd_d.t() << std::endl;
      robot.setVelocity(vpRobot::JOINT_STATE, qd_d);
      usleep(1*1000000);
//      qd_d[6] = 0;
//      std::cout << "DBG: main() new vel sent to thread: " << qd_d.t() << std::endl;
//      robot.setVelocity(vpRobot::JOINT_STATE, qd_d);
//      usleep(0.1*1000000);
      std::cout << "DBG: main() ask to stop the robot: " << qd_d.t() << std::endl;
      robot.setRobotState(vpRobot::STATE_STOP);

//      for (int i = 0; i < 5; i++) {
//        qd_d[0] += 0.1;
//        robot.setVelocity(vpRobot::JOINT_STATE, qd_d);
//      }
      std::cout << "End test" << std::endl;
      return 0;
    }

    // Test 1
    std::cout << "Start test 1" << std::endl;
    vpRobotFranka robot;
    robot.connect(argv[1]);

//    // Set additional parameters always before the control loop, NEVER in the
//    // control loop! Set collision behavior.
//    franka::Robot *handler = robot.getHandler();
//    handler->setCollisionBehavior({{10.0, 10.0, 9.0, 9.0, 8.0, 7.0, 6.0}}, {{10.0, 10.0, 9.0, 9.0, 8.0, 7.0, 6.0}},
//                               {{10.0, 10.0, 9.0, 9.0, 8.0, 7.0, 6.0}}, {{10.0, 10.0, 9.0, 9.0, 8.0, 7.0, 6.0}},
//                               {{10.0, 10.0, 10.0, 12.5, 12.5, 12.5}}, {{10.0, 10.0, 10.0, 12.5, 12.5, 12.5}},
//                               {{10.0, 10.0, 10.0, 12.5, 12.5, 12.5}}, {{10.0, 10.0, 10.0, 12.5, 12.5, 12.5}});


    vpColVector q(7);

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

//    std::cout << "Start read callback" << std::endl;
//    size_t count = 0;
//    handler->read([&count](const franka::RobotState &robot_state) {
//      // Printing to std::cout adds a delay. This is acceptable for a read
//      // loop such as this, but should not be done in a control loop.
//      std::cout << "Robot state:\n" << robot_state << std::endl;
//      return false;
//    });


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

#if 0
    // Set Joint velocity
    std::cout << "Set joint velocity" << std::endl;
    double time_max = 4.0;
    double time = 0.0;

    size_t njoints = 7;
    std::vector<double> q(njoints);
    std::vector<double> q_min(njoints);
    std::vector<double> q_max(njoints);
    std::vector<double> q_dot_max(njoints);
    std::vector<double> q_dot_dot_max(njoints);
    float delta_t = 0.001;

    std::array<double, 7> initial_position = handler->readOnce().q_d;

    for (size_t i=0; i<njoints; i++) {
      q[i] = initial_position[i];
      q_dot_dot_max[i] = 0.087; // 10 deg/s atteint en 2 s
      q_dot_max[i] = 3.1415/2.; // 90 deg/s
      q_min[i] = -1.57;   // Butee min
      q_max[i] =  1.57;   // Butee max
    }
    // Test
    q_min[1] = -1.57/2.;   // Butee min
    q_max[1] =  1.57/2.;   // Butee max

    std::vector<double> q_dot(njoints);
    for(size_t i=0; i<q_dot.size();i++)
      q_dot[i] = 0;
    q_dot[6] = -0.1;


    vpTrajectoryGenerator traj;
    traj.init(q, q_min, q_max, q_dot_max, q_dot_dot_max, delta_t);

    handler->control([=, &time, &traj](const franka::RobotState &, franka::Duration time_step) -> franka::JointVelocities {
      time += time_step.toSec();

      std::vector<double> q = traj.applyVel(q_dot);

//      franka::JointVelocities velocities = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, q[6]}};
      franka::JointVelocities velocities = {{q[0], q[1], q[2], q[3], q[4], q[5], q[6]}};

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
