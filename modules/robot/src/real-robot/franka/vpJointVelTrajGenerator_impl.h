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

#ifndef __vpJointVelTrajGenerator_impl_h_
#define __vpJointVelTrajGenerator_impl_h_

#include <array>
#include <iostream>
#include <atomic>

#include <franka/exception.h>
#include <franka/robot.h>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot_state.h>

class vpJointVelTrajGenerator {
public:
  vpJointVelTrajGenerator() : m_status(), m_deltaQmax(), m_deltaQ(), m_Qacc(),
    m_consFin(), m_signeDep(), m_q_cmd(), m_dq_cmd(), m_dist_AD(), m_dq_des(), m_dq_des_old(), m_ecart(),
    m_flagSpeed(),
    m_q_min(), m_q_max(), m_dq_max(), m_ddq_max(), m_njoints(7)  {}
  ~vpJointVelTrajGenerator() {}

  void applyVel(const std::array<double, 7> &dq_des,
                std::array<double, 7> &q_cmd,
                std::array<double, 7>  &dq_cmd);

  void control_thread(franka::Robot *robot, std::atomic_bool &running,
                      std::array<double, 7> &dq_des, // TODO should be const
                      const std::array<double, 7> &q_min,
                      const std::array<double, 7> &q_max,
                      const std::array<double, 7> &dq_max,
                      const std::array<double, 7> &ddq_max);

  void init (const std::array<double, 7> &q,
             const std::array<double, 7> &q_min,
             const std::array<double, 7> &q_max,
             const std::array<double, 7> &dq_max,
             const std::array<double, 7> &ddq_max,
             const float delta_t);

  std::array<double, 7> limitRate(const std::array<double, 7>& max_derivatives,
                                  const std::array<double, 7>& desired_values,
                                  const std::array<double, 7>& last_desired_values);

private:
  enum {
    FLAGACC,	// Axe en acceleration
    FLAGCTE,  // Axe en vitesse constante
    FLAGDEC,	// Axe en deceleration
    FLAGSTO 	// Axe stoppe
  };

//  std::vector<float> m_q_dot; // protected by mutex
//  std::vector<float> m_q;     // Current position


  std::vector<int>	 m_status;	    // Axis status
  std::vector<double> m_deltaQmax;   // Increment de consigne maximum
  std::vector<double> m_deltaQ;		  // Increment de consigne en cours
  std::vector<double> m_Qacc;		    // Increment d'increment de consigne
  std::vector<double> m_consFin;    	// Consigne finale (butee)
  std::vector<int>   m_signeDep;	  // Signe du deplacement: +1 = incrementer consigne
  std::array<double, 7> m_q_cmd;     // Joint position to apply in rad
  std::array<double, 7> m_dq_cmd;   // Joint velocity to apply in rad
  std::vector<double> m_dist_AD;		  // Distance requise pour acce et decel
  std::vector<double> m_dq_des;		  // Desired velocity
  std::vector<double> m_dq_des_old;
  std::vector<double> m_ecart;		    // Difference entre consigne et consigne finale
  std::vector<double> m_flagSpeed;

  // Constant
  std::array<double, 7>  m_q_min;
  std::array<double, 7>  m_q_max;
  std::array<double, 7>  m_dq_max;
  std::array<double, 7>  m_ddq_max;

  size_t m_njoints;
  float m_delta_t;

  bool m_flagJointLimit;

  const float m_offset_joint_limit = 0.01; // stop before joint limit
  const float m_deltaQMin =	0.0001;	// Delta Q minimum (rad)
};

#endif
