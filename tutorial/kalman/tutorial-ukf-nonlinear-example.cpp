/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
*****************************************************************************/

//! \example tutorial-ukf-nonlinear-example.cpp

// UKF includes
#include <visp3/core/vpUKSigmaDrawerMerwe.h>
#include <visp3/core/vpUnscentedKalman.h>

// ViSP includes
#include <visp3/core/vpGaussRand.h>
#include <visp3/gui/vpPlot.h>

/**
 * \brief The process function, that updates the prior.
 *
 * \param[in] chi A sigma point.
 * \param[in] dt The period.
 * \return vpColVector The sigma points projected in the future.
 */
vpColVector fx(const vpColVector &chi, const double &dt)
{
  vpColVector point(4);
  point[0] = chi[1] * dt + chi[0];
  point[1] = chi[1];
  point[2] = chi[3] * dt + chi[2];
  point[3] = chi[3];
  return point;
}

/**
 * \brief Class that permits to convert the position of the aircraft into
 * range and elevation angle measurements.
 */
class vpRadarStation
{
public:
  /**
   * \brief Construct a new vpRadarStation object.
   *
   * \param[in] x The position on the ground of the radar.
   * \param[in] y The altitude of the radar.
   * \param[in] range_std The standard deviation of the range measurements.
   * \param[in] elev_angle_std The standard deviation of the elevation angle measurements.
   */
  vpRadarStation(const double &x, const double &y, const double &range_std, const double &elev_angle_std)
    : m_x(x)
    , m_y(y)
    , m_rngRange(range_std, 0., vpTime::measureTimeMicros())
    , m_rngElevAngle(elev_angle_std, 0., vpTime::measureTimeMicros() + 4221)
  { }

  /**
   * \brief Convert the prior of the UKF into the measurement space.
   *
   * \param chi The prior.
   * \return vpColVector The prior expressed in the measurement space.
   */
  vpColVector state_to_measurement(const vpColVector &chi)
  {
    vpColVector meas(2);
    double dx = chi[0] - m_x;
    double dy = chi[2] - m_y;
    meas[0] = std::sqrt(dx * dx + dy * dy);
    meas[1] = std::atan2(dy, dx);
    return meas;
  }

  /**
   * \brief Perfect measurement of the range and elevation angle that
   * correspond to pos.
   *
   * \param pos The actual position of the aircraft (pos[0]: projection of the position
   * on the ground, pos[1]: altitude).
   * \return vpColVector [0] the range [1] the elevation angle.
   */
  vpColVector measureGT(const vpColVector &pos)
  {
    double dx = pos[0] - m_x;
    double dy = pos[1] - m_y;
    double range = std::sqrt(dx * dx + dy * dy);
    double elevAngle = std::atan2(dy, dx);
    vpColVector measurements(2);
    measurements[0] = range;
    measurements[1] = elevAngle;
    return measurements;
  }

  /**
   * \brief Noisy measurement of the range and elevation angle that
   * correspond to pos.
   *
   * \param pos The actual position of the aircraft (pos[0]: projection of the position
   * on the ground, pos[1]: altitude).
   * \return vpColVector [0] the range [1] the elevation angle.
   */
  vpColVector measureWithNoise(const vpColVector &pos)
  {
    vpColVector measurementsGT = measureGT(pos);
    vpColVector measurementsNoisy = measurementsGT;
    measurementsNoisy[0] += m_rngRange();
    measurementsNoisy[1] += m_rngElevAngle();
    return measurementsNoisy;
  }

private:
  double m_x; // The position on the ground of the radar
  double m_y; // The altitude of the radar
  vpGaussRand m_rngRange; // Noise simulator for the range measurement
  vpGaussRand m_rngElevAngle; // Noise simulator for the elevation angle measurement
};

/**
 * \brief Class to simulate a flying aircraft.
 */
class  vpACSimulator
{
public:
  /**
   * \brief Construct a new vpACSimulator object.
   *
   * \param[in] X0 Initial position of the aircraft.
   * \param[in] vel Velocity of the aircraft.
   * \param[in] vel_std Standard deviation of the variation of the velocity.
   */
  vpACSimulator(const vpColVector &X0, const vpColVector &vel, const double &vel_std)
    : m_pos(X0)
    , m_vel(vel)
    , m_rngVel(vel_std, 0.)
  {

  }

  /**
   * \brief Compute the new position of the aircraft after dt seconds have passed
   * since the last update.
   *
   * \param[in] dt Period since the last update.
   * \return vpColVector The new position of the aircraft.
   */
  vpColVector update(const double &dt)
  {
    vpColVector dx = m_vel * dt;
    dx[0] += m_rngVel() * dt;
    dx[1] += m_rngVel() * dt;
    m_pos += dx;
    return m_pos;
  }

private:
  vpColVector m_pos; // Position of the simulated aircraft
  vpColVector m_vel; // Velocity of the simulated aircraft
  vpGaussRand m_rngVel; // Random generator for slight variations of the velocity of the aircraft
};

int main(/*const int argc, const char *argv[]*/)
{
  const double dt = 3.; // Period of 1s
  const double sigmaRange = 5; // Standard deviation of the range measurement: 5m
  const double sigmaElevAngle = vpMath::rad(0.5); // Standard deviation of the elevation angle measurent: 0.5deg

  // Initialize the attributes of the UKF
  vpUKSigmaDrawerMerwe drawer(4, 0.1, 2., -1.);

  vpMatrix R(2, 2, 0.); // The covariance of the noise introduced by the measurement
  R[0][0] = sigmaRange*sigmaRange;
  R[1][1] = sigmaElevAngle*sigmaElevAngle;

  const double processVariance = 0.1;
  vpMatrix Q(4, 4, 0.); // The covariance of the process
  vpMatrix Q1d(2, 2); // The covariance of a process whose states are {x, dx/dt} and for which the variance is 1
  Q1d[0][0] = std::pow(dt, 3) / 3.;
  Q1d[0][1] = std::pow(dt, 2)/2.;
  Q1d[1][0] = std::pow(dt, 2)/2.;
  Q1d[1][1] = dt;
  Q.insert(Q1d, 0, 0);
  Q.insert(Q1d, 2, 2);
  Q = Q * processVariance;

  vpMatrix P0(4, 4); //  The initial guess of the process covariance
  P0.eye(4, 4);
  P0[0][0] = std::pow(300, 2);
  P0[1][1] = std::pow(30, 2);
  P0[2][2] = std::pow(150, 2);
  P0[3][3] = std::pow(30, 2);

  vpColVector X0(4);
  X0[0] = 0.; // x = 0m
  X0[1] = 90.; // dx/dt = 90m/s
  X0[2] = 1100.; // y = 1100m
  X0[3] = 0.; // dy/dt = 0m/s

  vpUnscentedKalman::vpProcessFunction f = fx;
  vpRadarStation radar(0., 0., sigmaRange, sigmaElevAngle);
  using std::placeholders::_1;
  vpUnscentedKalman::vpMeasurementFunction h = std::bind(&vpRadarStation::state_to_measurement, radar, _1);

  // Initialize the UKF
  vpUnscentedKalman ukf(Q, R, &drawer, f, h);
  ukf.init(X0, P0);

  // Initialize the plot
  vpPlot plot(4);
  plot.initGraph(0, 2);
  plot.setTitle(0, "Position along X-axis");
  plot.setUnitX(0, "Time (s)");
  plot.setUnitY(0, "Position (m)");
  plot.setLegend(0, 0, "GT");
  plot.setLegend(0, 1, "Filtered");

  plot.initGraph(1, 2);
  plot.setTitle(1, "Velocity along X-axis");
  plot.setUnitX(1, "Time (s)");
  plot.setUnitY(1, "Velocity (m/s)");
  plot.setLegend(1, 0, "GT");
  plot.setLegend(1, 1, "Filtered");

  plot.initGraph(2, 2);
  plot.setTitle(2, "Position along Y-axis");
  plot.setUnitX(2, "Time (s)");
  plot.setUnitY(2, "Position (m)");
  plot.setLegend(2, 0, "GT");
  plot.setLegend(2, 1, "Filtered");

  plot.initGraph(3, 2);
  plot.setTitle(3, "Velocity along Y-axis");
  plot.setUnitX(3, "Time (s)");
  plot.setUnitY(3, "Velocity (m/s)");
  plot.setLegend(3, 0, "GT");
  plot.setLegend(3, 1, "Filtered");

  // Initialize the simulation
  vpColVector ac_pos(2);
  ac_pos[0] = 0.;
  ac_pos[1] = 1000.;
  vpColVector ac_vel(2);
  ac_vel[0] = 100.;
  ac_vel[1] = 5.;
  vpACSimulator ac(ac_pos, ac_vel, 0.02);
  vpColVector gt_Xprec = ac_pos;
  for (int i = 0; i < 500; ++i) {
    // Perform the measurement
    vpColVector gt_X = ac.update(dt);
    vpColVector gt_V = (gt_X - gt_Xprec) / dt;
    vpColVector z = radar.measureWithNoise(gt_X);

    // Use the UKF to filter the measurement
    ukf.filter(z, dt);
    vpColVector Xest = ukf.getXest();

    // Plot the ground truth, measurement and filtered state
    plot.plot(0, 0, i, gt_X[0]);
    plot.plot(0, 1, i, Xest[0]);

    plot.plot(1, 0, i, gt_V[0]);
    plot.plot(1, 1, i, Xest[1]);

    plot.plot(2, 0, i, gt_X[1]);
    plot.plot(2, 1, i, Xest[2]);

    plot.plot(3, 0, i, gt_V[1]);
    plot.plot(3, 1, i, Xest[3]);

    gt_Xprec = gt_X;
  }
  std::cout << "Press Enter to quit..." << std::endl;
  std::cin.get();
  return 0;
}
