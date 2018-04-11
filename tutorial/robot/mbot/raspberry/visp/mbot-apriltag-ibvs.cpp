//! \example mbot-apriltag-ibvs.cpp.cpp
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/sensor/vpV4l2Grabber.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureDepth.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/robot/vpUnicycle.h>
#include <visp3/core/vpSerial.h>

int main(int argc, const char **argv)
{
#if defined(VISP_HAVE_APRILTAG) && (defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_OPENCV))
  int opt_device = 0;
  vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
  vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
  double tagSize = 0.053;
  float quad_decimate = 4.0;
  int nThreads = 2;
  std::string intrinsic_file = "";
  std::string camera_name = "";
  bool display_tag = false;
  bool display_on = false;
  bool serial_off = false;
  bool integrator = false;
  double integrator_mu = 0.4;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--pose_method" && i + 1 < argc) {
      poseEstimationMethod = (vpDetectorAprilTag::vpPoseEstimationMethod)atoi(argv[i + 1]);
    } else if (std::string(argv[i]) == "--tag_size" && i + 1 < argc) {
      tagSize = atof(argv[i + 1]);
    } else if (std::string(argv[i]) == "--input" && i + 1 < argc) {
      opt_device = atoi(argv[i + 1]);
    } else if (std::string(argv[i]) == "--quad_decimate" && i + 1 < argc) {
      quad_decimate = (float)atof(argv[i + 1]);
    } else if (std::string(argv[i]) == "--nthreads" && i + 1 < argc) {
      nThreads = atoi(argv[i + 1]);
    } else if (std::string(argv[i]) == "--intrinsic" && i + 1 < argc) {
      intrinsic_file = std::string(argv[i + 1]);
    } else if (std::string(argv[i]) == "--camera_name" && i + 1 < argc) {
      camera_name = std::string(argv[i + 1]);
    } else if (std::string(argv[i]) == "--display_tag") {
      display_tag = true;
#if !(defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
    } else if (std::string(argv[i]) == "--display_on") {
      display_on = true;
#endif
    } else if (std::string(argv[i]) == "--serial_off") {
      serial_off = true;
    } else if (std::string(argv[i]) == "--tag_family" && i + 1 < argc) {
      tagFamily = (vpDetectorAprilTag::vpAprilTagFamily)atoi(argv[i + 1]);
    } else if (std::string(argv[i]) == "--integrator") {
      integrator = true;
    } else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "Usage: " << argv[0]
                << " [--input <camera input>] [--tag_size <tag_size in m>]"
                   " [--quad_decimate <quad_decimate>] [--nthreads <nb>]"
                   " [--intrinsic <intrinsic file>] [--camera_name <camera name>]"
                   " [--pose_method <method> (0: HOMOGRAPHY_VIRTUAL_VS, 1: "
                   "DEMENTHON_VIRTUAL_VS,"
                   " 2: LAGRANGE_VIRTUAL_VS, 3: BEST_RESIDUAL_VIRTUAL_VS)]"
                   " [--tag_family <family> (0: TAG_36h11, 1: TAG_36h10, 2: "
                   "TAG_36ARTOOLKIT,"
                   " 3: TAG_25h9, 4: TAG_25h7, 5: TAG_16h5)]"
                   " [--display_tag]";
#if (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
      std::cout << " [--display_on]";
#endif
      std::cout << " [--serial_off] [--help]" << std::endl;
      return EXIT_SUCCESS;
    }
  }

  // if com ok: led 1 green
  // if exception: led 1 red
  // if tag detected: led 2 green, else led 2 red
  // if motor left: led 3 blue
  // if motor right: led 4 blue

  vpSerial *serial = NULL;
  if (! serial_off) {
    serial = new vpSerial("/dev/ttyAMA0", 115200);

    serial->write("LED_RING=0,0,0,0\n"); // Switch off all led
    serial->write("LED_RING=1,0,10,0\n"); // Switch on led 1 to green: serial ok
  }

  try {
    vpImage<unsigned char> I;

    //! [Construct grabber]
#if defined(VISP_HAVE_V4L2)
    vpV4l2Grabber g;
    std::ostringstream device;
    device << "/dev/video" << opt_device;
    g.setDevice(device.str());
    g.setScale(1);
    g.acquire(I);
#elif defined(VISP_HAVE_OPENCV)
    cv::VideoCapture cap(opt_device); // open the default camera
    if (!cap.isOpened()) {            // check if we succeeded
      std::cout << "Failed to open the camera" << std::endl;
      return EXIT_FAILURE;
    }
    cv::Mat frame;
    cap >> frame; // get a new frame from camera
    vpImageConvert::convert(frame, I);
#endif
    //! [Construct grabber]

    vpDisplay *d = NULL;
    if (display_on) {
#ifdef VISP_HAVE_X11
      d = new vpDisplayX(I);
#elif defined(VISP_HAVE_GDI)
      d = new vpDisplayGDI(I);
#elif defined(VISP_HAVE_OPENCV)
      d = new vpDisplayOpenCV(I);
#endif
    }

    vpCameraParameters cam;
    cam.initPersProjWithoutDistortion(615.1674805, 615.1675415, I.getWidth() / 2., I.getHeight() / 2.);
  #ifdef VISP_HAVE_XML2
    vpXmlParserCamera parser;
    if (!intrinsic_file.empty() && !camera_name.empty())
      parser.parse(cam, intrinsic_file, camera_name, vpCameraParameters::perspectiveProjWithoutDistortion);
  #endif
    std::cout << "cam:\n" << cam << std::endl;
    std::cout << "poseEstimationMethod: " << poseEstimationMethod << std::endl;
    std::cout << "tagFamily: " << tagFamily << std::endl;

    vpDetectorAprilTag detector(tagFamily);

    detector.setAprilTagQuadDecimate(quad_decimate);
    detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
    detector.setAprilTagNbThreads(nThreads);
    detector.setDisplayTag(display_tag);

    vpServo task;
    vpAdaptiveGain lambda;
    if (display_on)
      lambda.initStandard(2.5, 0.4, 30); // lambda(0)=2.5, lambda(oo)=0.4 and lambda'(0)=30
    else
      lambda.initStandard(4.5, 0.4, 30); // lambda(0)=4.5, lambda(oo)=0.4 and lambda'(0)=30

    vpUnicycle robot;
    task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
    task.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE);
    task.setLambda(lambda);
    vpRotationMatrix cRe;
    cRe[0][0] = 0; cRe[0][1] = -1; cRe[0][2] =  0;
    cRe[1][0] = 0; cRe[1][1] =  0; cRe[1][2] = -1;
    cRe[2][0] = 1; cRe[2][1] =  0; cRe[2][2] =  0;

    vpHomogeneousMatrix cMe(vpTranslationVector(), cRe);
    vpVelocityTwistMatrix cVe(cMe);
    task.set_cVe(cVe);

    vpMatrix eJe(6, 2, 0);
    eJe[0][0] = eJe[5][1] = 1.0;

    std::cout << "eJe: \n" << eJe << std::endl;

    // Current and desired visual feature associated to the x coordinate of
    // the point
    vpFeaturePoint s_x, s_xd;
    vpImagePoint cog;
    double Z, Zd;
    Z = Zd = 0.3;

    // Create the current x visual feature
    vpFeatureBuilder::create(s_x, cam, cog);

    // Create the desired x* visual feature
    s_xd.buildFrom(0, 0, Zd);

    // Add the feature
    task.addFeature(s_x, s_xd, vpFeaturePoint::selectX());

    // Create the current log(Z/Z*) visual feature
    vpFeatureDepth s_Z, s_Zd;

    std::cout << "Z " << Z << std::endl;
    s_Z.buildFrom(s_x.get_x(), s_x.get_y(), Z, 0); // log(Z/Z*) = 0 that's why the last parameter is 0
    s_Zd.buildFrom(0, 0, Zd, 0); // The value of s* is 0 with Z=1 meter

    // Add the feature
    task.addFeature(s_Z, s_Zd);

    vpColVector v; // vz, wx
    vpColVector sum_de_dt(2);

    std::vector<double> time_vec;
    for (;;) {
      //! [Acquisition]
#if defined(VISP_HAVE_V4L2)
      g.acquire(I);
#elif defined(VISP_HAVE_OPENCV)
      cap >> frame; // get a new frame from camera
      vpImageConvert::convert(frame, I);
#endif
      //! [Acquisition]

      vpDisplay::display(I);

      double t = vpTime::measureTimeMs();
      std::vector<vpHomogeneousMatrix> cMo_vec;
      detector.detect(I, tagSize, cam, cMo_vec);
      t = vpTime::measureTimeMs() - t;
      time_vec.push_back(t);

      std::stringstream ss;
      ss << "Detection time: " << t << " ms for " << detector.getNbObjects() << " tags";
      vpDisplay::displayText(I, 40, 20, ss.str(), vpColor::red);

      //! [Display camera pose for each tag]
      for (size_t i = 0; i < cMo_vec.size(); i++) {
        vpDisplay::displayFrame(I, cMo_vec[i], cam, tagSize / 2, vpColor::none, 3);
      }
      //! [Display camera pose for each tag]
      //!
      //!       vpFeatureBuilder::create(s_x, cam, dot);

      if (detector.getNbObjects() == 1) {
        if (! serial_off) {
//        serial->write("LED_RING=2,0,10,0\n"); // Switch on led 2 to green: tag detected
        }

        vpPolygon polygon(detector.getPolygon(0));
        double surface = polygon.getArea();
        std::cout << "Surface: " << surface << std::endl;

        // Compute the distance from target surface and 3D size
        Z = tagSize * cam.get_px() / sqrt(surface);

        vpFeatureBuilder::create(s_x, cam, detector.getCog(0));
        s_x.set_Z(Z);

        // Update log(Z/Z*) feature. Since the depth Z change, we need to update
        // the intection matrix
        s_Z.buildFrom(s_x.get_x(), s_x.get_y(), Z, log(Z / Zd));

        std::cout << "cog: " << detector.getCog(0) << " Z: " << Z << std::endl;

        task.set_cVe(cVe);
        task.set_eJe(eJe);

        // Compute the control law. Velocities are computed in the mobile robot reference frame
        v = task.computeControlLaw();

        if (integrator) {
          vpColVector error = task.getError();
          if (std::fabs(error[0]) < (20 / cam.get_px()) && std::fabs(error[1]) < 0.05) {
            sum_de_dt += error;
            v -= integrator_mu * task.getTaskJacobianPseudoInverse() * sum_de_dt;

            vpDisplay::displayText(I, 80, 20, "Use integrator", vpColor::red);
            std::cout << "Integrator, v: " << v.t() << std::endl;
          }
          else {
            std::cout << "Quit Integrator" << "\n";
            sum_de_dt = 0.0;
          }
        }

        std::cout << "Send velocity to the pionner: " << v[0] << " m/s " << vpMath::deg(v[1]) << " deg/s" << std::endl;

        task.print();
        double radius = 0.0325;
        double L = 0.0725;
        double motor_left  = (-v[0] - L * v[1]) / radius;
        double motor_right = ( v[0] - L * v[1]) / radius;
        std::cout << "motor left vel: " << motor_left << " motor right vel: " << motor_right << std::endl;
        if (! serial_off) {
//        serial->write("LED_RING=3,0,0,10\n"); // Switch on led 3 to blue: motor left servoed
//        serial->write("LED_RING=4,0,0,10\n"); // Switch on led 4 to blue: motor right servoed
        }
        std::stringstream ss;
        double rpm_left  = motor_left  * 30. / M_PI;
        double rpm_right = motor_right * 30. / M_PI;
        ss << "MOTOR_RPM=" << vpMath::round(rpm_left) << "," << vpMath::round(rpm_right) << "\n";
        std::cout << "Send: " << ss.str() << std::endl;
        if (! serial_off) {
          serial->write(ss.str());
        }
      }
      else {
        // stop the robot
        if (! serial_off) {
//        serial->write("LED_RING=2,10,0,0\n"); // Switch on led 2 to red: tag not detected
//        serial->write("LED_RING=3,0,0,0\n"); // Switch on led 3 to blue: motor left not servoed
//        serial->write("LED_RING=4,0,0,0\n"); // Switch on led 4 to blue: motor right not servoed
//        serial->write("MOTOR_RPM=0,-0\n"); // Stop the robot
        }
      }

      vpDisplay::displayText(I, 20, 20, "Click to quit.", vpColor::red);
      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false))
        break;
    }

    if (! serial_off) {
      serial->write("LED_RING=0,0,0,0\n"); // Switch off all led
    }

    std::cout << "Benchmark computation time" << std::endl;
    std::cout << "Mean / Median / Std: " << vpMath::getMean(time_vec) << " ms"
              << " ; " << vpMath::getMedian(time_vec) << " ms"
              << " ; " << vpMath::getStdev(time_vec) << " ms" << std::endl;

    if (display_on)
      delete d;
    if (! serial_off) {
      delete serial;
    }
  } catch (const vpException &e) {
    std::cerr << "Catch an exception: " << e.getMessage() << std::endl;
    if (! serial_off) {
      serial->write("LED_RING=1,10,0,0\n"); // Switch on led 1 to red
    }
  }

  return EXIT_SUCCESS;
#else
  (void)argc;
  (void)argv;
#ifndef VISP_HAVE_APRILTAG
  std::cout << "ViSP is not build with Apriltag support" << std::endl;
#endif
#if !(defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_OPENCV))
  std::cout << "ViSP is not build with v4l2 or OpenCV support" << std::endl;
#endif
  std::cout << "Install missing 3rd parties, configure and build ViSP to run this tutorial" << std::endl;
  return EXIT_SUCCESS;
#endif
}
