/*
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
 * Description:
 * Hand-eye calibration.
 */

/*!
 * \file vpHandEyeCalibration.h
 * \brief Tools for hand-eye calibration.
 *
 * \sa The example in calibrate-hand-eye.cpp
 */
#ifndef _vpHandEyeCalibration_h_
#define _vpHandEyeCalibration_h_

#include <vector>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMatrix.h>

BEGIN_VISP_NAMESPACE
/*!
 * \class vpHandEyeCalibration
 *
 * \ingroup group_vision_calib
 *
 * \brief Tool for hand-eye calibration.
*/
class VISP_EXPORT vpHandEyeCalibration
{
public:
  /*!
   * Compute extrinsic camera parameters : the constant transformation from
   * the effector to the camera frames (eMc).
   *
   * \param[in] cMo : vector of homogeneous matrices representing the transformation
   * between the camera and the scene.
   * \param[in] rMe : vector of homogeneous matrices representing the transformation
   * between the effector (where the camera is fixed) and the reference
   * coordinates (base of the manipulator). Must be the same size as cMo.
   * \param[out] eMc : homogeneous matrix representing the transformation
   * between the effector and the camera (output)
   *
   * \return 0 if calibration succeed, -1 if the system is not full rank, 1 if the algorithm doesn't converge.
   */
  static int calibrate(const std::vector<vpHomogeneousMatrix> &cMo, const std::vector<vpHomogeneousMatrix> &rMe,
                       vpHomogeneousMatrix &eMc);

private:
  static void calibrationVerifrMo(const std::vector<vpHomogeneousMatrix> &cMo,
                                  const std::vector<vpHomogeneousMatrix> &rMe, const vpHomogeneousMatrix &eMc);
  static int calibrationRotationTsai(const std::vector<vpHomogeneousMatrix> &cMo,
                                     const std::vector<vpHomogeneousMatrix> &rMe, vpRotationMatrix &eRc);
  static int calibrationRotationTsaiOld(const std::vector<vpHomogeneousMatrix> &cMo,
                                        const std::vector<vpHomogeneousMatrix> &rMe, vpRotationMatrix &eRc);
  static int calibrationRotationProcrustes(const std::vector<vpHomogeneousMatrix> &cMo,
                                           const std::vector<vpHomogeneousMatrix> &rMe, vpRotationMatrix &eRc);
  static int calibrationTranslation(const std::vector<vpHomogeneousMatrix> &cMo,
                                    const std::vector<vpHomogeneousMatrix> &rMe, vpRotationMatrix &eRc,
                                    vpTranslationVector &eTc);
  static int calibrationTranslationOld(const std::vector<vpHomogeneousMatrix> &cMo,
                                       const std::vector<vpHomogeneousMatrix> &rMe, vpRotationMatrix &eRc,
                                       vpTranslationVector &eTc);
  static double calibrationErrVVS(const std::vector<vpHomogeneousMatrix> &cMo,
                                  const std::vector<vpHomogeneousMatrix> &rMe, const vpHomogeneousMatrix &eMc,
                                  vpColVector &errVVS);
  static int calibrationVVS(const std::vector<vpHomogeneousMatrix> &cMo, const std::vector<vpHomogeneousMatrix> &rMe,
                            vpHomogeneousMatrix &eMc);
};
END_VISP_NAMESPACE
#endif
