/****************************************************************************
 *
 * $Id: vpRobotCamera.h 2456 2010-01-07 10:33:12Z nmelchio $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Defines the simplest robot : a free flying camera.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


#ifndef vpRobotCamera_H
#define vpRobotCamera_H

/*!
  \file vpRobotCamera.h
  \brief class that defines the simplest robot : a free flying camera
*/

#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpMatrix.h>
#include <visp/vpRobotSimulator.h>

/*!
  \class vpRobotCamera

  \ingroup RobotSimuWithoutVisu

  \brief Class that defines the simplest robot: a free flying camera.

  This free flying camera has 6 dof; 3 in translation and 3 in rotation.
  It evolves as a gentry robot with respect to a world frame. This class
  is similar to vpSimulatorCamera class except that here the position of the robot
  is provided as the transformation from camera frame to world frame; cMw. Since
  the position of the camera frame evolves, this representation is less intuitive
  than the one implemented in vpSimulatorCamera where the transformation from world
  to camera frame is considered; wMc.

  \note We recommend to use vpSimulatorCamera rather than vpRobotCamera.

  For this particular simulated robot, the end-effector and camera frame are confused.
  That means that the cMe transformation is equal to identity.

  The robot jacobian expressed in the end-effector frame
  \f$ {^e}{\bf J}_e \f$ is also set to identity (see get_eJe()).

  The following code shows how to control this robot in position and velocity.
  \code
#include <visp/vpRobotCamera.h>

int main()
{
  vpHomogeneousMatrix cMw;
  vpRobotCamera robot;

  robot.getPosition(cMw); // Position of the camera in the world frame
  std::cout << "Default position of the camera in the world frame cMw:\n" << cMw << std::endl;

  cMw[2][3] = 1.; // World frame is 1 meter along z axis in front of the camera frame
  robot.setPosition(cMw); // Set the new position of the camera wrt the world frame
  std::cout << "New position of the camera wrt the world frame cMw:\n" << cMw << std::endl;

  robot.setSamplingTime(0.100); // Modify the default sampling time to 0.1 second
  robot.setMaxTranslationVelocity(1.); // vx, vy and vz max set to 1 m/s
  robot.setMaxRotationVelocity(vpMath::rad(90)); // wx, wy and wz max set to 90 deg/s

  vpColVector v(6);
  v = 0;
  v[2] = 1.; // set v_z to 1 m/s
  robot.setVelocity(vpRobot::CAMERA_FRAME, v);
  // The robot has moved from 0.1 meters along the z axis
  robot.getPosition(cMw); // Position of the camera wrt the world frame
  std::cout << "New position of the camera cMw:\n" << cMw << std::endl;
}
  \endcode

*/
class VISP_EXPORT vpRobotCamera : public vpRobotSimulator
{
protected:
  vpHomogeneousMatrix cMw_; // camera to world

public:
  vpRobotCamera() ;
  virtual ~vpRobotCamera() ;

  void get_cVe(vpVelocityTwistMatrix &cVe);
  void get_eJe(vpMatrix &eJe)    ;

  void getPosition(vpHomogeneousMatrix &cMw) const   ;
  void getPosition(const vpRobot::vpControlFrameType frame, vpColVector &q);

  void setPosition(const vpHomogeneousMatrix &cMw) ;
  void setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &v)  ;

private:
  void init() ;

  // Non implemented virtual pure functions
  void get_fJe(vpMatrix & /*_fJe */) {};
  void getDisplacement(const vpRobot::vpControlFrameType /* frame */, vpColVector & /* q */) {};
  void setPosition(const vpRobot::vpControlFrameType /* frame */, const vpColVector & /* q */) {};
} ;

#endif
/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */