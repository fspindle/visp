//! \example mbot-apriltag-ibvs.cpp.cpp
#include <visp3/core/vpSerial.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/core/vpMomentObject.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpMomentGravityCenter.h>
#include <visp3/core/vpMomentDatabase.h>
#include <visp3/core/vpMomentCentered.h>
#include <visp3/core/vpMomentAreaNormalized.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpV4l2Grabber.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/visual_features/vpFeatureMomentAreaNormalized.h>
#include <visp3/vs/vpServo.h>
#include <visp3/robot/vpUnicycle.h>

int main(int argc, const char **argv)
{
#if defined(VISP_HAVE_APRILTAG) //&& defined(VISP_HAVE_V4L2)
  int device = 0;
  vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
  double tagSize = 0.065;
  float quad_decimate = 4.0;
  int nThreads = 2;
  std::string intrinsic_file = "";
  std::string camera_name = "";
  bool display_tag = false;
  bool display_on = false;
  bool serial_off = false;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--tag_size" && i + 1 < argc) {
      tagSize = std::atof(argv[i + 1]);
    } else if (std::string(argv[i]) == "--input" && i + 1 < argc) {
      device = std::atoi(argv[i + 1]);
    } else if (std::string(argv[i]) == "--quad_decimate" && i + 1 < argc) {
      quad_decimate = (float)atof(argv[i + 1]);
    } else if (std::string(argv[i]) == "--nthreads" && i + 1 < argc) {
      nThreads = std::atoi(argv[i + 1]);
    } else if (std::string(argv[i]) == "--intrinsic" && i + 1 < argc) {
      intrinsic_file = std::string(argv[i + 1]);
    } else if (std::string(argv[i]) == "--camera_name" && i + 1 < argc) {
      camera_name = std::string(argv[i + 1]);
    } else if (std::string(argv[i]) == "--display_tag") {
      display_tag = true;
#if !(defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI))
    } else if (std::string(argv[i]) == "--display_on") {
      display_on = true;
#endif
    } else if (std::string(argv[i]) == "--serial_off") {
      serial_off = true;
    } else if (std::string(argv[i]) == "--tag_family" && i + 1 < argc) {
      tagFamily = (vpDetectorAprilTag::vpAprilTagFamily)std::atoi(argv[i + 1]);
    } else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "Usage: " << argv[0]
                << " [--input <camera input>] [--tag_size <tag_size in m>]"
                   " [--quad_decimate <quad_decimate>] [--nthreads <nb>]"
                   " [--intrinsic <intrinsic file>] [--camera_name <camera name>]"
                   " [--tag_family <family> (0: TAG_36h11, 1: TAG_36h10, 2: "
                   "TAG_36ARTOOLKIT,"
                   " 3: TAG_25h9, 4: TAG_25h7, 5: TAG_16h5)]"
                   " [--display_tag]";
#if (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI))
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

    vpV4l2Grabber grabber;
    std::ostringstream device_name;
    device_name << "/dev/video" << device;
    grabber.setDevice(device_name.str());
    grabber.setScale(1);
    grabber.acquire(I);

    vpDisplay *d = NULL;
    if (display_on) {
#ifdef VISP_HAVE_X11
      d = new vpDisplayX(I);
#elif defined(VISP_HAVE_GDI)
      d = new vpDisplayGDI(I);
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
    std::cout << "tagFamily: " << tagFamily << std::endl;
    std::cout << "tagSize: " << tagSize << std::endl;

    vpDetectorAprilTag detector(tagFamily);

    detector.setAprilTagQuadDecimate(quad_decimate);
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
    task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE);
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

    // Current and desired visual feature associated to the x coordinate of the point
    vpFeaturePoint s_x, s_xd;
    vpImagePoint cog;
    double Zd = 0.3;

    // Create the current x visual feature
    vpFeatureBuilder::create(s_x, cam, cog);

    // Create the desired x* visual feature
    s_xd.buildFrom(0, 0, Zd);

    // Add the point feature
    task.addFeature(s_x, s_xd, vpFeaturePoint::selectX());

    double X[4] = {-tagSize/2.,  tagSize/2., tagSize/2., -tagSize/2.};
    double Y[4] = {-tagSize/2., -tagSize/2., tagSize/2.,  tagSize/2.};
    std::vector<vpPoint> vec_P, vec_Pd;
    double m_ad = 0;
    for (int i = 0; i < 4; i++) {
      vpPoint Pd(X[i], Y[i], 0);
      vpHomogeneousMatrix cdMo(0, 0, Zd, 0, 0, 0);
      Pd.track(cdMo); //
      vec_Pd.push_back(Pd);

      // Compute moment area a at desired position: m_ad
      m_ad += vpMath::sqr(Pd.get_x()) + vpMath::sqr(Pd.get_y());
    }

    vpMomentObject m_obj(3), m_obj_d(3);
    vpMomentDatabase mdb, mdb_d;
    vpMomentGravityCenter g, g_d;
    vpMomentCentered mc, mc_d;
    vpMomentAreaNormalized an(m_ad, Zd), an_d(m_ad, Zd); //declare normalized surface with

    // Desired moments
    m_obj_d.setType(vpMomentObject::DISCRETE); // Discrete mode for object
    m_obj_d.fromVector(vec_Pd); // initialize the object with the points coordinates

    g_d.linkTo(mdb_d);        // Add gravity center to database
    mc_d.linkTo(mdb_d);       // Add centered moments to database
    an_d.linkTo(mdb_d);       // Add area normalized to database
    mdb_d.updateAll(m_obj_d); // All of the moments must be updated, not just an_d
    g_d.compute();            // Compute gravity center moment
    mc_d.compute();           // Compute centered moments AFTER gravity center
    an_d.compute();           // Compute area centered moment AFTER centered moments

    // Desired plane
    double A = 0.0;
    double B = 0.0;
    double C = 1.0 / Zd;

    // Construct area normalized features
    vpFeatureMomentAreaNormalized s_an(mdb, A, B, C), s_an_d(mdb_d, A, B, C);

    // Add the feature
    task.addFeature(s_an, s_an_d);

    // Update desired area normalized feature
    s_an_d.update(A, B, C);
    s_an_d.compute_interaction();

    vpColVector v; // vz, wx

    std::vector<double> time_vec;
    for (;;) {
      //! [Acquisition]
      grabber.acquire(I);
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

      if (detector.getNbObjects() == 1) {
        if (! serial_off) {
//        serial->write("LED_RING=2,0,10,0\n"); // Switch on led 2 to green: tag detected
        }

        vpFeatureBuilder::create(s_x, cam, detector.getCog(0));
        s_x.set_Z(Zd);

        // Update points
        std::vector< vpImagePoint > vec_ip = detector.getPolygon(0);
        vec_P.clear();
        for (size_t i = 0; i < vec_ip.size(); i++) { // size = 4
          double x = 0, y = 0;
          vpPixelMeterConversion::convertPoint(cam, vec_ip[i], x, y);
          vpPoint P;
          P.set_x(x);
          P.set_y(y);
          vec_P.push_back(P);
        }

        // Current moments
        m_obj.setType(vpMomentObject::DISCRETE); // Discrete mode for object
        m_obj.fromVector(vec_P); // initialize the object with the points coordinates

        g.linkTo(mdb);        // Add gravity center to database
        mc.linkTo(mdb);       // Add centered moments to database
        an.linkTo(mdb);       // Add area normalized to database
        mdb.updateAll(m_obj); // All of the moments must be updated, not just an_d
        g.compute();            // Compute gravity center moment
        mc.compute();           // Compute centered moments AFTER gravity center
        an.compute();           // Compute area centered moment AFTER centered moments

        s_an.update(A, B, C);
        s_an.compute_interaction();

        task.set_cVe(cVe);
        task.set_eJe(eJe);

        // Compute the control law. Velocities are computed in the mobile robot reference frame
        v = task.computeControlLaw();

        std::cout << "Send velocity to the mbot: " << v[0] << " m/s " << vpMath::deg(v[1]) << " deg/s" << std::endl;

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
#ifndef VISP_HAVE_V4L2
  std::cout << "ViSP is not build with v4l2 support" << std::endl;
#endif
  std::cout << "Install missing 3rd parties, configure and build ViSP to run this tutorial" << std::endl;
  return EXIT_SUCCESS;
#endif
}
