// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>
#include <fstream>

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_FRANKA

#include <franka/exception.h>
#include <franka/robot.h>

/**
 * @example franka_generate_joint_velocity_motion.cpp
 * An example showing how to generate a joint velocity motion.
 *
 * @warning Before executing this example, make sure there is enough space in
 * front of the robot.
 *
 * This example is part of libfranka FCI C++ API:
 * https://frankaemika.github.io/libfranka See
 * https://frankaemika.github.io/docs for more details.
 */

int main(int argc, char **argv)
{
  if (argc == 1) {
    std::cout << "\nUsage: " << argv[0] << " <robot-hostname> [--save] [--help]\n" << std::endl;
    return -1;
  }
  try {
    bool save = false;
    for (int i = 0; i < argc; i++) {
      if (std::string(argv[i]) == "--save")
        save = true;
      else if (std::string(argv[i]) == "--help") {
        std::cout << "\nUsage: " << argv[0] << " <robot-hostname> [--save] [--help]\n" << std::endl;
        return 0;
      }
    }

    franka::Robot robot(argv[1]);
    std::ofstream f;

    if (save) {
      std::string filename("joint-vel-command.txt");
      std::cout << "Save control velocities in: " << filename << std::endl;
      f.open(filename);
    }

    // Set additional parameters always before the control loop, NEVER in the
    // control loop! Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    double time_max = 4.0;
    double omega_max = 0.2;
    double time = 0.0;
    robot.control([=, &time, &f](const franka::RobotState &, franka::Duration time_step) -> franka::JointVelocities {
      time += time_step.toSec();

      double cycle = std::floor(std::pow(-1.0, (time - std::fmod(time, time_max)) / time_max));
      double omega = cycle * omega_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * time));

      franka::JointVelocities velocities = {{0.0, 0.0, 0.0, omega, omega, omega, omega}};

      if (save) {
        f << time;
        for (size_t i = 0; i < 7; i ++) {
          f << " " << velocities.dq[i];
        }
        f << std::endl;
      }

      if (time >= 2 * time_max) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(velocities);
      }
      return velocities;
    });

    if (save) {
      f.close();
    }

  } catch (const franka::Exception &e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}

#else
int main() { std::cout << "This example needs libfranka to control Panda robot." << std::endl; }
#endif
