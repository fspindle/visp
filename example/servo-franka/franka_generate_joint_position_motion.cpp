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
 * @example franka_generate_joint_position_motion.cpp
 * An example showing how to generate a joint position motion.
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
      std::string filename("joint-pos-command.txt");
      std::cout << "Save control joint in: " << filename << std::endl;
      f.open(filename);
    }

    // Set additional parameters always before the control loop, NEVER in the
    // control loop! Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    auto initial_position = robot.readOnce().q_d;
    double time = 0.0;
    robot.control([=, &time, &f](const franka::RobotState &, franka::Duration time_step) -> franka::JointPositions {
      time += time_step.toSec();

      double delta_angle = M_PI / 8 * (1 - std::cos(M_PI / 5.0 * time));

      franka::JointPositions output = {{initial_position[0], initial_position[1], initial_position[2],
                                        initial_position[3] + delta_angle, initial_position[4] + delta_angle,
                                        initial_position[5], initial_position[6] + delta_angle}};
      if (save) {
        f << time;
        for (size_t i = 0; i < 7; i ++) {
          f << " " << output.q[i];
        }
        f << std::endl;
      }

      if (time >= 10.0) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(output);
      }
      return output;
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
