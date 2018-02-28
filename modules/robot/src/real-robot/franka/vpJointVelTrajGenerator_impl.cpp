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

#include "vpJointVelTrajGenerator_impl.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

#include <visp3/core/vpException.h>
#include <visp3/core/vpTime.h>
#include <visp3/core/vpMath.h> // TODO only for debug

#define USE_ROBOT
#define USE_MY_ROBOT_CONTROLLER

void vpJointVelTrajGenerator::control_thread(franka::Robot *robot,
                                             std::atomic_bool &running,
                                             std::array<double, 7> &dq_des,
                                             const std::array<double, 7> &q_min,
                                             const std::array<double, 7> &q_max,
                                             const std::array<double, 7> &dq_max,
                                             const std::array<double, 7> &ddq_max)
{
  std::cout << "DBG: control_thread() in: " << m_njoints <<  std::endl;

  running = true;

#ifdef USE_ROBOT
  // Set additional parameters always before the control loop, NEVER in the control loop!
  // Set collision behavior.
  robot->setCollisionBehavior(
  {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
  {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
  {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
  {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
#endif

#if defined(USE_ROBOT) && !defined(USE_MY_ROBOT_CONTROLLER)
  double omega_max = 1.0;
#endif
  double time = 0.0;

  std::array<double, 7> q;

  float delta_t = 0.001;

#ifdef USE_ROBOT
  franka::RobotState state = robot->readOnce();
  q = state.q_d; // Initial measured position (should be insite joint limits)
#else
  for (size_t i=0; i<m_njoints; i++)
    q[i] = 0; // Initial position (should be insite joint limits)
#endif

  vpJointVelTrajGenerator joint_vel_traj_generator;
  joint_vel_traj_generator.init(q, q_min, q_max, dq_max, ddq_max, delta_t);

  auto q_prev = q;

#ifdef USE_ROBOT
  robot->control([=, &time, &joint_vel_traj_generator, &q_prev, &dq_des, &running](const franka::RobotState& state,
                 franka::Duration period) -> franka::JointVelocities {
    time += period.toSec();

#ifdef USE_MY_ROBOT_CONTROLLER
    if (! running) { // Stop asked
      for (auto & dq_ : dq_des) {
        dq_ = 0.0;
      }
    }

    std::array<double, 7> q_cmd; // TODO remove since not used
    std::array<double, 7> dq_cmd;

    joint_vel_traj_generator.applyVel(dq_des, q_cmd, dq_cmd);

    //    std::cout << "New vel: ";
    //    for (size_t i=0; i < 7; i++)
    //      std::cout << qd_cmd[i] << " ";
    //    std::cout << std::endl;

    franka::JointVelocities velocities = {{dq_cmd[0], dq_cmd[1], dq_cmd[2], dq_cmd[3], dq_cmd[4], dq_cmd[5], dq_cmd[6]}};
    //    franka::JointVelocities velocities = {{0, 0, 0, 0, 0, 0, qd_cmd[6]}};

    static bool display_dbg = true;
    if (!running) {
      if (display_dbg) {
        std::cout << std::endl << "DBG: control_thread() asked to finish waiting for joint stop" << std::endl;
        display_dbg = false;
      }
      unsigned int stop = 0;
      static int cpt_dbg = 0;
      const double q_eps = (1.0/100000.);
      for(size_t i=0; i < 7; i++) {
        if (std::abs(state.q_d[i] - q_prev[i]) < q_eps) {
          stop ++;
        }
      }
      cpt_dbg ++;
      if (stop == 7) {
        //        std::cout << std::endl << "DBG: control_thread() all joints are stopped after " << cpt_dbg << " iter, shutting down example" << std::endl;
        return franka::MotionFinished(velocities);
      }
    }
    else {
      display_dbg = true;
    }

    q_prev = state.q_d;
#else
    double time_max = 1.0;
    double cycle = std::floor(std::pow(-1.0, (time - std::fmod(time, time_max)) / time_max));
    double omega = cycle * omega_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * time));
    franka::JointVelocities velocities = {{0.0, 0.0, 0.0, omega, omega, omega, omega}};


    if (time >= 4 * time_max) {
      std::cout << std::endl << "DBG: control_thread() Finished motion after 2 cycles, shutting down example" << std::endl;
      return franka::MotionFinished(velocities);
    }
#endif


    // state.q_d contains the last joint velocity command received by the robot.
    // In case of packet loss due to bad connection or due to a slow control loop
    // not reaching the 1kHz rate, even if your desired velocity trajectory
    // is smooth, discontinuities might occur.
    // Saturating the acceleration computed with respect to the last command received
    // by the robot will prevent from getting discontinuity errors.
    // Note that if the robot does not receive a command it will try to extrapolate
    // the desired behavior assuming a constant acceleration model
    std::array<double, 7> limited_velocities = limitRate(ddq_max, velocities.dq, state.dq_d);
    std::cout << "vel to apply joint 6: " << dq_cmd[6] << " should be = to " << velocities.dq[6] << " limited to: " << limited_velocities[6] << std::endl;
    return limited_velocities;
    //return limitRate(ddq_max, velocities.dq, state.dq_d);
  });
#else
  auto dq_des_prev = dq_des;
  dq_des_prev[0] = -dq_des_prev[0];
  //  for(size_t i =0; i <500; i++) {
  while(1) {
    bool differ = false;
    for (size_t i=0; i<m_njoints; i++) {
      if (dq_des_prev[i] != dq_des[i]) {
        differ = true;
      }
    }
    if (differ) {
      std::cout << "DBG: control_thread() Apply new joint vel: ";
      for(int i=0; i<7;i++)
        std::cout << dq_des[i] << " ";
      std::cout << std::endl;
    }
    if (! running) {
      std::cout << "DBG: control_thread() End for loop in thread" << std::endl;
      break;
    }
    dq_des_prev = dq_des;
    vpTime::sleepMs(1);
  }
#endif

}

void vpJointVelTrajGenerator::init(const std::array<double, 7> &q,
                                   const std::array<double, 7> &q_min,
                                   const std::array<double, 7> &q_max,
                                   const std::array<double, 7> &dq_max,
                                   const std::array<double, 7> &ddq_max,
                                   const float delta_t)
{
  if (m_njoints != q_min.size() || m_njoints != q_max.size()
      || m_njoints != dq_max.size() || m_njoints != ddq_max.size()) {
    throw(vpException(vpException::dimensionError, "Inconsistent number of joints"));
  }
  m_q_min = q_min;
  m_q_max = q_max;
  m_dq_max = dq_max;
  m_ddq_max = ddq_max;

  m_delta_t = delta_t;

  m_q_cmd = q;
  m_dq_cmd = std::array<double, 7>{0,0,0,0,0,0,0};

  m_status.resize(m_njoints);
  m_deltaQmax.resize(m_njoints);
  m_deltaQ.resize(m_njoints);
  m_Qacc.resize(m_njoints);
  m_Qacc_sav.resize(m_njoints);
  m_consFin.resize(m_njoints);
  m_signeDep.resize(m_njoints);
  m_dist_AD.resize(m_njoints);
  m_pcspeed.resize(m_njoints);
  m_pcspeed_old.resize(m_njoints);
  m_ecart.resize(m_njoints);
  m_flagSpeed.resize(m_njoints);

  for (size_t i=0; i<m_njoints; i++)
  {
    m_flagSpeed[i] = false;
    m_status[i] 	= FLAGSTO;
    m_deltaQ[i] 	= m_deltaQmax[i] 	= 0.0;
    m_Qacc_sav[i] = m_Qacc[i] = m_ddq_max[i] * m_delta_t * m_delta_t;
    m_consFin[i] 	= q[i];
    m_dist_AD[i] 	= 0.0;
    m_pcspeed[i]	= m_pcspeed_old[i] = 0.0;
    m_status[i]   = FLAGSTO;
    m_signeDep[i]	= 0;
    m_consFin[i] = 0;
  }
}


/*!
 * Compute the joint position and velocity to reach desired joint velocity.
 * \param dq_des : Desired joint velocity
 * \param q_cmd : Position to apply.
 * \param dq_cmd : Velocity to apply.
 */
void vpJointVelTrajGenerator::applyVel(const std::array<double, 7> &dq_des,
                                       std::array<double, 7> &q_cmd,
                                       std::array<double, 7>  &dq_cmd)
{
  for (size_t i=0; i < m_njoints; i++) {
    m_pcspeed[i] = dq_des[i]; // TODO fuse in same var

    if (m_pcspeed[i] != m_pcspeed_old[i]) m_flagJointLimit = false;

    if (m_pcspeed[i] != m_pcspeed_old[i]) {
      m_Qacc[i] = m_Qacc_sav[i];

      if (m_pcspeed[i] > m_dq_max[i]) {
        m_pcspeed[i] = m_dq_max[i];
        if (m_flagQMax == false)
        {
          m_flagQMax = true;
        }
      }
      else if (m_pcspeed[i] < (-m_dq_max[i])) {
        m_pcspeed[i] = -m_dq_max[i];
        if (m_flagQMax == false)
        {
          m_flagQMax = true;
        }
      }
      else m_flagQMax = false;

      if (m_flagSpeed[i] == false) {
        //if (pt_movespeed.m_flagSpeed[i] == false) {
        /* Changement de consigne et non en phase de chang sens */

        if ( m_status[i] == FLAGSTO) /* Si arret */
        {
          if (m_pcspeed[i] > 0)
          {
            m_deltaQmax[i] = m_pcspeed[i]*m_delta_t;
            m_signeDep[i] = 1;
            m_consFin[i] = m_q_max[i] - m_offset_joint_limit;
            m_deltaQ[i] = 0;
            m_status[i] = FLAGACC;
          }
          else if (m_pcspeed[i] < 0)
          {
            m_deltaQmax[i] = - m_pcspeed[i]*m_delta_t;
            m_signeDep[i] = -1;
            m_consFin[i] = m_q_min[i] + m_offset_joint_limit;
            m_deltaQ[i] = 0;
            m_status[i] = FLAGACC;
          }
        }
        // Si non en arret et changement de sens


        else if ( (m_pcspeed[i] * m_signeDep[i]) < 0) {
          m_flagSpeed[i] = true;
          m_status[i] = FLAGDEC;
          m_deltaQmax[i] = 0;
        }
        // Pas de changement de sens

        else {	/* Non arret et pas de changement de sens */
          if ( m_signeDep[i] == 1) {
            if ( m_pcspeed[i] > m_pcspeed_old[i])
              m_status[i] = FLAGACC;
            else  m_status[i] = FLAGDEC;
            m_deltaQmax[i] = m_pcspeed[i]*m_delta_t;
          }
          else {
            if ( m_pcspeed[i] > m_pcspeed_old[i])
              m_status[i] = FLAGDEC;
            else  m_status[i] = FLAGACC;
            m_deltaQmax[i] = - m_pcspeed[i]*m_delta_t;
          }
        }

        int n = (int) (m_deltaQmax[i] / m_Qacc[i]);
        m_dist_AD[i]=n*(m_deltaQmax[i]-(n+1)*m_Qacc[i]/2);
      }
      m_pcspeed_old[i] = m_pcspeed[i];
    }

    /*
     * calcul des consignes selon les cas:
     *		- acceleration
     *		- deceleration
     *		- arret
     */

    { // debug
      int axe = 6;
      std::cout << "m_consFin[" << axe << "]: " << m_consFin[axe]
                   << " m_q_cmd[" << axe << "]: " << m_q_cmd[axe]
                      << " m_ecart[" << axe << "]: " << m_ecart[axe]
                         << " m_dist_AD[" << axe << "]: " << m_dist_AD[axe]
                            << " m_q_min[" << axe << "]: " << m_q_min[axe]
                               << " m_q_max[" << axe << "]: " << m_q_max[axe]
                         << std::endl;
    }
    for (size_t i=0; i < m_njoints; i++) {
      /*
       * Securite butee en vitesse constante
       */
      m_ecart[i] = ( m_consFin[i] - m_q_cmd[i]) * m_signeDep[i];
      if ((m_ecart[i] - m_deltaQmax[i]) <=  m_dist_AD[i]) {
        if (m_dist_AD[i] > 0) {
          if (!m_flagJointLimit) printf("Flagbutee axe %lu\n",i);
          m_flagJointLimit = true;
          for(size_t k=0; k < m_njoints; k++)
          {
            if (m_status[k] != FLAGSTO) m_status[k] = FLAGDEC;
            m_deltaQmax[k] = 0;
          }
        }
      }
      /*
       * Deceleration.
       */
      if ( m_status[i] == FLAGDEC) {
        m_deltaQ[i] -=  m_Qacc[i];
        if (m_deltaQ[i] <=  m_deltaQmax[i]) {
          if (m_deltaQmax[i] < m_deltaQMin)  {
            m_status[i] = FLAGSTO;
            m_deltaQ[i] = 0.0;
            // Test si on etait en phase de changement de sens.
            if (m_flagSpeed[i] == true) {
              //if (pt_movespeed.m_flagSpeed[i] == true) {
              if (m_pcspeed[i] > 0) {
                m_deltaQmax[i] = m_pcspeed[i]*m_delta_t;
                m_signeDep[i] = 1;
                m_consFin[i] = m_q_max[i] - m_offset_joint_limit;
              }
              else if (m_pcspeed[i] < 0) {
                m_deltaQmax[i] = -m_pcspeed[i]*m_delta_t;
                m_signeDep[i] = -1;
                m_consFin[i] = m_q_min[i] + m_offset_joint_limit;
              }
              m_status[i] = FLAGACC;
              m_flagSpeed[i] = false;

              int n = (int) (m_deltaQmax[i] / m_Qacc[i]);
              m_dist_AD[i]=n*(m_deltaQmax[i]-(n+1)*m_Qacc[i]/2);
            }
          }
          else if ((m_deltaQmax[i] > 0) && !m_flagJointLimit)  {
            if (m_deltaQmax[i] < (m_deltaQ[i] + 2*m_Qacc[i])) {
              m_deltaQ[i] = m_deltaQmax[i];
              m_status[i] = FLAGCTE;
            }
            else if (!m_flagJointLimit) {
              /* acceleration moins rapide*/
              m_deltaQ[i] += (2*m_Qacc[i]);
              m_status[i] = FLAGACC;
            }
          }
        }
      }
      /*
       * Acceleration.
       */
      else if (m_status[i] == FLAGACC) {
        m_deltaQ[i] += m_Qacc[i];

        if (m_deltaQ[i] >= m_deltaQmax[i]) {
          m_deltaQ[i] = m_deltaQmax[i];
          m_status[i] = FLAGCTE;
        }
      }
      /*
       * Sinon a vitesse constante increment non change.
       */
      m_q_cmd[i] += m_signeDep[i] * m_deltaQ[i];
      m_dq_cmd[i] = m_signeDep[i] * m_deltaQ[i] / m_delta_t;


    } /* endfor */

    // Test si un axe arrive pres des butees. Si oui, arret de tous les axes
    for (size_t i=0; i < m_njoints;i++) {
      float butee = m_q_min[i] + m_offset_joint_limit;
      if (m_q_cmd[i] < butee) {
        for (size_t j=0; j < m_njoints;j++) m_q_cmd[j] -= m_signeDep[j]*m_deltaQ[j];
        m_q_cmd[i] = butee;
        m_dq_cmd[i] = 0;
        printf("Butee axe %lu\n",i);
        break;
      }
      butee = (float) (m_q_max[i] - m_offset_joint_limit);
      if (m_q_cmd[i] > butee) {
        for (size_t j=0; j < m_njoints; j++) m_q_cmd[j] -= m_signeDep[j]*m_deltaQ[j];
        m_q_cmd[i] = butee;
        m_dq_cmd[i] = 0;
        printf("Butee axe %lu\n",i);
        break;
      }
    }
    //    std::cout << "new_pos: ";
    //    for (int i=0;i<jointNames.size();i++)
    //      std::cout << jointNames[i] << "(" << m_q_cmd[i] << ") ";
    //    std::cout << std::endl;

  }
  q_cmd = m_q_cmd;
  dq_cmd = m_dq_cmd;
}

/**
 * Limits the rate of an input vector of per-joint commands considering the maximum allowed time
 * derivatives.
 *
 * @param[in] max_derivatives Per-joint maximum allowed time derivative.
 * @param[in] desired_values Desired values of the current time step.
 * @param[in] last_desired_values Desired values of the previous time step.
 *
 * @return Rate-limited vector of desired values.
 */
std::array<double, 7> vpJointVelTrajGenerator::limitRate(const std::array<double, 7>& max_derivatives,
                                                         const std::array<double, 7>& desired_values,
                                                         const std::array<double, 7>& last_desired_values) {
  std::array<double, 7> limited_values{};
  for (size_t i = 0; i < 7; i++) {
    double desired_difference = (desired_values[i] - last_desired_values[i]) / 1e-3;
    limited_values[i] =
        last_desired_values[i] +
        std::max(std::min(desired_difference, max_derivatives[i]), -max_derivatives[i]) * 1e-3;
  }
  return limited_values;
}

