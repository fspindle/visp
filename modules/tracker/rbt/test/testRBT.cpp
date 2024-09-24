/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * Test vpMbGenericTracker JSON parse / save.
 */

/*!
  \file testMbtJsonSettings.cpp

  Test test saving and parsing JSON configuration for vpMbGenericTracker
*/

#include <visp3/core/vpIoTools.h>
#include <visp3/rbt/vpRBTracker.h>

#include <visp3/rbt/vpRBSilhouetteMeTracker.h>
#include <visp3/rbt/vpRBSilhouetteCCDTracker.h>
#include <visp3/rbt/vpRBKltTracker.h>
#include <visp3/rbt/vpRBDenseDepthTracker.h>



#if defined(VISP_HAVE_CATCH2)

#if defined(VISP_HAVE_NLOHMANN_JSON)
#include <nlohmann/json.hpp>
#endif

#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

SCENARIO("Instanciating a silhouette me tracker", "[rbt]")
{
  GIVEN("A base me tracker")
  {
    vpRBSilhouetteMeTracker tracker;
    WHEN("Changing mask parameters")
    {
      THEN("Enabling mask is seen")
      {
        bool useMaskDefault = tracker.shouldUseMask();
        tracker.setShouldUseMask(!useMaskDefault);
        REQUIRE(useMaskDefault != tracker.shouldUseMask());
      }
      THEN("Changing mask min confidence with a correct value is Ok")
      {
        tracker.setMinimumMaskConfidence(0.0);
        REQUIRE(tracker.getMinimumMaskConfidence() == 0.0);
        tracker.setMinimumMaskConfidence(1.0);
        REQUIRE(tracker.getMinimumMaskConfidence() == 1.0);
        tracker.setMinimumMaskConfidence(0.5);
        REQUIRE(tracker.getMinimumMaskConfidence() == 0.5);
      }
      THEN("Setting incorrect mask confidence value fails")
      {
        REQUIRE_THROWS(tracker.setMinimumMaskConfidence(-1.0));
      }
    }
    WHEN("Changing robust threshold")
    {
      THEN("Setting correct value works")
      {
        tracker.setMinRobustThreshold(0.5);
        REQUIRE(tracker.getMinRobustThreshold() == 0.5);
      }
      THEN("Setting negative value throws")
      {
        REQUIRE_THROWS(tracker.setMinRobustThreshold(-0.5));
      }
    }
    WHEN("Changing number of candidates")
    {
      THEN("Setting correct value works")
      {
        tracker.setNumCandidates(3);
        REQUIRE(tracker.getNumCandidates() == 3);
      }
      THEN("Setting incorrect value throws")
      {
        REQUIRE_THROWS(tracker.setNumCandidates(0));
      }
    }
    WHEN("Changing convergence settings")
    {
      THEN("Setting correct single point value works")
      {
        tracker.setSinglePointConvergenceThreshold(1.0);
        REQUIRE(tracker.getSinglePointConvergenceThreshold() == 1.0);
      }
      THEN("Setting incorrect single point value throws")
      {
        REQUIRE_THROWS(tracker.setSinglePointConvergenceThreshold(-1.0));
      }
      THEN("Setting correct global value works")
      {
        tracker.setGlobalConvergenceMinimumRatio(0.0);
        REQUIRE(tracker.getGlobalConvergenceMinimumRatio() == 0.0);
        tracker.setGlobalConvergenceMinimumRatio(1.0);
        REQUIRE(tracker.getGlobalConvergenceMinimumRatio() == 1.0);
        tracker.setGlobalConvergenceMinimumRatio(0.5);
        REQUIRE(tracker.getGlobalConvergenceMinimumRatio() == 0.5);
      }
    }
#if defined(VISP_HAVE_NLOHMANN_JSON)
    WHEN("defining JSON parameters")
    {

      nlohmann::json j = {
        {"type", "silhouetteMe"},
        { "numCandidates", 1 },
        { "weight", 0.5 },
        { "convergencePixelThreshold", 0.5 },
        { "convergenceRatio", 0.99},
        { "useMask", true},
        { "minMaskConfidence", 0.5},
        { "movingEdge", {
          {"maskSign", 0},
          {"maskSize" , 5},
          {"minSampleStep" , 4.0},
          {"mu" , {0.5, 0.5}},
          {"nMask" , 90},
          {"ntotalSample" , 0},
          {"pointsToTrack" , 200},
          {"range" , 5},
          {"sampleStep" , 4.0},
          {"strip" , 2},
          {"thresholdType" , "normalized"},
          {"threshold" , 20.0}
        }}
      };
      THEN("Loading correct settings works")
      {
        tracker.loadJsonConfiguration(j);
        REQUIRE(tracker.getNumCandidates() == 1);
        REQUIRE(tracker.shouldUseMask() == true);
        REQUIRE(tracker.getMinimumMaskConfidence() == 0.5);
        REQUIRE(tracker.getMe().getMaskNumber() == 90);
        REQUIRE(tracker.getMe().getThreshold() == 20.0);
      }
      THEN("Setting incorrect candidate number throws")
      {
        j["numCandidates"] = 0;
        REQUIRE_THROWS(tracker.loadJsonConfiguration(j));
      }
      THEN("Setting incorrect mask confidence throws")
      {
        j["minMaskConfidence"] = 5.0;
        REQUIRE_THROWS(tracker.loadJsonConfiguration(j));
      }
      THEN("Setting incorrect single point convergence vlaue confidence throws")
      {
        j["convergencePixelThreshold"] = -1.0;
        REQUIRE_THROWS(tracker.loadJsonConfiguration(j));
      }
      THEN("Setting incorrect global convergence vlaue confidence throws")
      {
        j["convergenceRatio"] = 2.0;
        REQUIRE_THROWS(tracker.loadJsonConfiguration(j));
      }
    }
  }
#endif
}

SCENARIO("Instanciating a silhouette CCD tracker", "[rbt]")
{
  vpRBSilhouetteCCDTracker tracker;
  WHEN("Setting smoothing factor")
  {
    THEN("Setting value above 0 works")
    {
      tracker.setTemporalSmoothingFactor(0.5);
      REQUIRE(tracker.getTemporalSmoothingFactor() == 0.5);
    }
    THEN("Setting value below 0 throws")
    {
      REQUIRE_THROWS(tracker.setTemporalSmoothingFactor(-2.0));
    }
  }
  WHEN("Updating CCD parameters")
  {
    vpCCDParameters ccd = tracker.getCCDParameters();
    ccd.h += 4;
    ccd.delta_h += 2;
    tracker.setCCDParameters(ccd);
    THEN("Changes are propagated to tracker")
    {
      REQUIRE(tracker.getCCDParameters().h == ccd.h);
      REQUIRE(tracker.getCCDParameters().delta_h == ccd.delta_h);
    }

  }

#if defined(VISP_HAVE_NLOHMANN_JSON)
  WHEN("Defining associated json")
  {
    nlohmann::json j = {
      {"type", "silhouetteCCD"},
      {"weight", 0.01},
      {"temporalSmoothing", 0.1},
      {"convergenceThreshold", 0.1},
      {"ccd", {
        {"h", 64},
        {"delta_h", 16},
        {"gamma", { 0.1, 0.2, 0.3, 0.4 } }
      }}
    };
    THEN("Loading correct json works")
    {
      tracker.loadJsonConfiguration(j);
      REQUIRE(tracker.getTemporalSmoothingFactor() == 0.1);
      vpCCDParameters ccd = tracker.getCCDParameters();
      REQUIRE(ccd.h == 64);
      REQUIRE(ccd.delta_h == 16);
      REQUIRE((ccd.gamma_1 == 0.1 && ccd.gamma_2 == 0.2 && ccd.gamma_3 == 0.3 && ccd.gamma_4 == 0.4));
    }
    THEN("Loading invalid temporal smoothing factor throws")
    {
      j["temporalSmoothing"] = -3.14;
      REQUIRE_THROWS(tracker.loadJsonConfiguration(j));
    }
    THEN("Loading invalid ccd gamma throws")
    {
      j["ccd"]["gamma"] = -3.14;
      REQUIRE_THROWS(tracker.loadJsonConfiguration(j));
    }
  }
#endif
}

#if defined(VP_HAVE_RB_KLT_TRACKER)
SCENARIO("Instanciating KLT tracker")
{
  vpRBKltTracker tracker;
  WHEN("Modifying basic settings")
  {
    tracker.setFilteringBorderSize(2);
    tracker.setFilteringMaxReprojectionError(0.024);
    tracker.setMinimumDistanceNewPoints(0.005);
    tracker.setMinimumNumberOfPoints(20);
    tracker.setShouldUseMask(true);
    tracker.setMinimumMaskConfidence(0.5);
    THEN("Every change is visible")
    {
      REQUIRE(tracker.getFilteringBorderSize() == 2);
      REQUIRE(tracker.getFilteringMaxReprojectionError() == 0.024);
      REQUIRE(tracker.getMinimumDistanceNewPoints() == 0.005);
      REQUIRE(tracker.getMinimumNumberOfPoints() == 20);
      REQUIRE(tracker.shouldUseMask());
      REQUIRE(tracker.getMinimumMaskConfidence() == 0.5);
    }
    THEN("Setting incorrect Mask confidence throws")
    {
      REQUIRE_THROWS(tracker.setMinimumMaskConfidence(-1.0));
    }
  }

#if defined(VISP_HAVE_NLOHMANN_JSON)
  WHEN("Defining associated json")
  {

    nlohmann::json j = {
      {"type", "klt"},
      {"weight", 0.01},
      {"minimumNumPoints", 25},
      {"newPointsMinPixelDistance", 5},
      {"maxReprojectionErrorPixels", 0.01},
      {"useMask", true},
      {"minMaskConfidence", 0.1},
      { "windowSize", 7 },
      { "quality", 0.01 },
      { "maxFeatures", 500 }
    };
    THEN("Loading correct json works")
    {
      tracker.loadJsonConfiguration(j);
      REQUIRE(tracker.getMinimumNumberOfPoints() == 25);
      REQUIRE(tracker.getMinimumDistanceNewPoints() == 5);
      REQUIRE(tracker.getFilteringMaxReprojectionError() == 0.01);
      REQUIRE(tracker.shouldUseMask() == true);
      REQUIRE(tracker.getMinimumMaskConfidence() == 0.1f);
      REQUIRE(tracker.getKltTracker().getWindowSize() == 7);
      REQUIRE(tracker.getKltTracker().getQuality() == 0.01);
      REQUIRE(tracker.getKltTracker().getMaxFeatures() == 500);
    }
    THEN("Loading invalid mask confidence throws")
    {
      j["minMaskConfidence"] = -3.14;
      REQUIRE_THROWS(tracker.loadJsonConfiguration(j));
    }
  }
#endif
}
#endif

SCENARIO("Instanciating depth tracker", "[rbt]")
{
  vpRBDenseDepthTracker tracker;
  WHEN("Setting steps")
  {
    THEN("Setting positive value works")
    {
      tracker.setStep(4);
      REQUIRE(tracker.getStep() == 4);
    }
    THEN("Setting 0 step is invalid")
    {
      REQUIRE_THROWS(tracker.setStep(0));
    }
  }
  WHEN("Setting confidence")
  {
    THEN("Setting incorrect mask confidence value")
    {
      REQUIRE_THROWS(tracker.setMinimumMaskConfidence(-1.0));
    }
    THEN("Setting correct mask confidence value")
    {
      tracker.setMinimumMaskConfidence(0.8);
      REQUIRE(tracker.getMinimumMaskConfidence() == 0.8f);
    }
    THEN("Toggling mask works")
    {
      tracker.setShouldUseMask(true);
      REQUIRE(tracker.shouldUseMask());
    }
  }
#if defined(VISP_HAVE_NLOHMANN_JSON)
  WHEN("Defining associated json")
  {

    nlohmann::json j = {
      {"type", "klt"},
      {"weight", 0.01},
      {"step", 16},
      {"useMask", true},
      {"minMaskConfidence", 0.1}
    };
    THEN("Loading correct json works")
    {
      tracker.loadJsonConfiguration(j);
      REQUIRE(tracker.getStep() == 16);
      REQUIRE(tracker.shouldUseMask());
      REQUIRE(tracker.getMinimumMaskConfidence() == 0.1f);
    }
    THEN("Loading invalid mask confidence throws")
    {
      j["minMaskConfidence"] = -3.14;
      REQUIRE_THROWS(tracker.loadJsonConfiguration(j));
    }
    THEN("Loading invalid step throws")
    {
      j["step"] = 0;
      REQUIRE_THROWS(tracker.loadJsonConfiguration(j));
    }
  }
#endif
}

SCENARIO("Instanciating a render-based tracker", "[rbt]")
{
  vpRBTracker tracker;

  WHEN("Setting optimization parameters")
  {
    THEN("Max num iter cannot be zero")
    {
      REQUIRE_THROWS(tracker.setMaxOptimizationIters(0));
    }
    THEN("Setting num iter is ok")
    {
      tracker.setMaxOptimizationIters(10);
      REQUIRE(tracker.getMaxOptimizationIters() == 10);
    }
    THEN("Gain cannot be negative")
    {
      REQUIRE_THROWS(tracker.setOptimizationGain(-0.5));
    }
    THEN("Positive gain is ok")
    {
      tracker.setOptimizationGain(0.5);
      REQUIRE(tracker.getOptimizationGain() == 0.5);
    }
    THEN("Initial mu cannot be negative")
    {
      REQUIRE_THROWS(tracker.setOptimizationInitialMu(-0.5));
    }
    THEN("Initial mu can be zero (gauss newton)")
    {
      tracker.setOptimizationInitialMu(0.0);
      REQUIRE(tracker.getOptimizationInitialMu() == 0.0);
    }
    THEN("Initial mu can be above zero")
    {
      tracker.setOptimizationInitialMu(0.1);
      REQUIRE(tracker.getOptimizationInitialMu() == 0.1);
    }

    THEN("Mu factor cannot be negative")
    {
      REQUIRE_THROWS(tracker.setOptimizationMuIterFactor(-0.5));
    }
    THEN("Mu factor can be zero")
    {
      tracker.setOptimizationMuIterFactor(0.0);
      REQUIRE(tracker.getOptimizationMuIterFactor() == 0.0);
    }
    THEN("Mu factor can be positive")
    {
      tracker.setOptimizationMuIterFactor(0.1);
      REQUIRE(tracker.getOptimizationMuIterFactor() == 0.1);
    }


  }

  WHEN("Setting camera parameters and resolution")
  {
    unsigned int h = 480, w = 640;
    vpCameraParameters cam(600, 600, 320, 240);
    THEN("Image height cannot be zero")
    {
      REQUIRE_THROWS(tracker.setCameraParameters(cam, 0, w));
    }
    THEN("Image width cannot be zero")
    {
      REQUIRE_THROWS(tracker.setCameraParameters(cam, h, 0));
    }
    THEN("Camera model cannot have distortion")
    {
      cam.initPersProjWithDistortion(600, 600, 320, 240, 0.01, 0.01);
      REQUIRE_THROWS(tracker.setCameraParameters(cam, h, w));
    }
    THEN("Loading with perspective model with no distortion and correct resolution is ok")
    {
      tracker.setCameraParameters(cam, h, w);
      REQUIRE(tracker.getCameraParameters() == cam);
      REQUIRE(tracker.getImageHeight() == h);
      REQUIRE(tracker.getImageWidth() == w);
    }
  }

#if defined(VISP_HAVE_NLOHMANN_JSON)
  WHEN("Loading JSON configuration")
  {
    const std::string jsonLiteral = R"JSON({
      "camera": {
        "intrinsics": {
          "model": "perspectiveWithoutDistortion",
          "px" : 302.573,
          "py" : 302.396,
          "u0" : 162.776,
          "v0" : 122.475
        },
        "height": 240,
        "width" : 320
      },
      "vvs": {
        "gain": 1.0,
        "maxIterations" : 10
      },
      "model" : "/home/sfelton/Downloads/sinatrack-data/data/cutting_guide/cutting_guide.obj",
      "silhouetteExtractionSettings" : {
        "threshold": {
          "type": "relative",
          "value" : 0.1
        },
        "sampling" : {
          "type": "fixed",
          "numPoints" : 400
        }
      },
      "features": {
        "silhouetteGeometry": {
          "weight" : 0.5,
          "numCandidates" : 3,
          "convergencePixelThreshold" : 3,
          "convergenceRatio" : 0.99,
          "movingEdge" : {
            "maskSign": 0,
            "maskSize" : 5,
            "minSampleStep" : 4.0,
            "mu" : [
              0.5,
                0.5
            ] ,
            "nMask" : 90,
            "ntotalSample" : 0,
            "pointsToTrack" : 200,
            "range" : 5,
            "sampleStep" : 4.0,
            "strip" : 2,
            "thresholdType" : "normalized",
            "threshold" : 20.0
          }
        },
        "silhouetteColor" : {
          "weight" : 0.5,
          "convergenceThreshold" : 0.1,
          "temporalSmoothing" : 0.1,
          "ccd" : {
          "h": 4,
          "delta_h" : 1
        }
      }
    }
    })JSON";
    nlohmann::json j = nlohmann::json::parse(jsonLiteral);
    THEN("Loading configuration with trackers and a 3D model works")
    {
      tracker.loadConfiguration(j);
      REQUIRE(...);
    }
    THEN("Loading configuration without model also works")
    {
      j.erase("model");
      tracker.loadConfiguration(j);
      REQUIRE(...);
    }
  }
#endif

}


int main(int argc, char *argv[])
{
  Catch::Session session; // There must be exactly one instance
  session.applyCommandLine(argc, argv);

  int numFailed = session.run();
  return numFailed;
}

#else

int main()
{
  return EXIT_SUCCESS;
}

#endif
