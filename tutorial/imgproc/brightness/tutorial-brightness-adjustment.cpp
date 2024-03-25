//! \example tutorial-brightness-adjustment.cpp

#include <cstdlib>
#include <iostream>
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>

#if defined(VISP_HAVE_MODULE_IMGPROC)
//! [Include]
#include <visp3/imgproc/vpImgproc.h>
//! [Include]
#endif

int main(int argc, const char **argv)
{
//! [Macro defined]
#if defined(VISP_HAVE_MODULE_IMGPROC) &&                                                                               \
    (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV)) &&                                 \
    (defined(VISP_HAVE_PNG) || defined(VISP_HAVE_OPENCV))
  //! [Macro defined]
  //!
  std::string input_filename = "Sample_low_brightness.png";
  double alpha = 10.0, beta = 50.0;
  double gamma = 3.5;
  vp::vpGammaMethod method = vp::GAMMA_MANUAL;
  vp::vpGammaColorHandling colorHandling = vp::GAMMA_HSV;
  int scale = 240, scaleDiv = 3, level = 0, kernelSize = -1;
  double dynamic = 3.0;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--input" && i + 1 < argc) {
      input_filename = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--alpha" && i + 1 < argc) {
      alpha = atof(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--beta" && i + 1 < argc) {
      beta = atof(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--gamma" && i + 1 < argc) {
      gamma = atof(argv[i + 1]);
    }
    else if ((std::string(argv[i]) == "--gamma-color-handling") && ((i + 1) < argc)) {
      ++i;
      colorHandling = vp::vpGammaColorHandlingFromString(argv[i]);
    }
    else if ((std::string(argv[i]) == "--gamma-method") && ((i + 1) < argc)) {
      ++i;
      method = vp::vpGammaMethodFromString(argv[i]);
    }
    else if (std::string(argv[i]) == "--scale" && i + 1 < argc) {
      scale = atoi(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--scaleDiv" && i + 1 < argc) {
      scaleDiv = atoi(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--level" && i + 1 < argc) {
      level = atoi(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--kernelSize" && i + 1 < argc) {
      kernelSize = atoi(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--dynamic" && i + 1 < argc) {
      dynamic = atof(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "Usage: " << argv[0]
        << " [--input <input image>]"
        " [--alpha <alpha for vp::adjust()>] [--beta <beta for "
        "vp::adjust()>]"
        " [--gamma <gamma for vp::gammaCorrection()>]"
        " [--gamma-color-handling " << vp::vpGammaColorHandlingList() << "]"
        " [--gamma-method " << vp::vpGammaMethodList() << "]"
        " [--scale <scale for vp::retinex()> [--scaleDiv for "
        "vp::retinex()]"
        " [--level <level for vp::retinex()> [--kernelSize "
        "<kernelSize for vp::retinex()>]"
        " [--dynamic <dynamic for vp::retinex()>] [--help]"
        << std::endl;
      return EXIT_SUCCESS;
    }
  }

  vpImage<vpRGBa> I_color;
  vpImageIo::read(I_color, input_filename);
  vpImage<unsigned char> I_gray;
  vpImageConvert::convert(I_color, I_gray);

  vpImage<vpRGBa> I_color_res(I_color.getHeight(), 2 * I_color.getWidth());
  I_color_res.insert(I_color, vpImagePoint());
  vpImage<unsigned char> I_gray_res(I_gray.getHeight(), 2 * I_gray.getWidth());
  I_gray_res.insert(I_gray, vpImagePoint());
#ifdef VISP_HAVE_X11
  vpDisplayX d_gray(I_gray_res);
  vpDisplayX d(I_color_res);
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI d_gray(I_gray_res);
  vpDisplayGDI d(I_color_res);
#elif defined(HAVE_OPENCV_HIGHGUI)
  vpDisplayOpenCV d_gray(I_gray_res);
  vpDisplayOpenCV d(I_color_res);
#endif

  //! [Brightness contrast adjustment]
  vpImage<vpRGBa> I_color_adjust;
  vp::adjust(I_color, I_color_adjust, alpha, beta);
  //! [Brightness contrast adjustment]
  I_color_res.insert(I_color_adjust, vpImagePoint(0, I_color.getWidth()));
  std::stringstream ss;
  ss << "Sample_low_brightness_alpha=" << alpha << "_beta=" << beta << ".png";
  vpImageIo::write(I_color_res, ss.str());

  vpDisplay::display(I_color_res);
  vpDisplay::displayText(I_color_res, 20, 20, "Brightness and contrast adjustment. Click to continue.", vpColor::red);
  vpDisplay::flush(I_color_res);
  vpDisplay::getClick(I_color_res);

  //! [Gamma correction]
  if (method != vp::GAMMA_MANUAL) {
    // If the user wants to use an automatic method, the gamma factor must be negative.
    gamma = -1.;
  }

  if (gamma > 0.) {
    // If the user wants to set a constant user-defined gamma factor, the method must be set to manual.
    method = vp::GAMMA_MANUAL;
  }
  vpImage<unsigned char> I_gray_gamma_correction;
  vp::gammaCorrection(I_gray, I_gray_gamma_correction, static_cast<float>(gamma), method);
  vpImage<vpRGBa> I_color_gamma_correction;
  vp::gammaCorrection(I_color, I_color_gamma_correction, static_cast<float>(gamma), colorHandling, method);
  //! [Gamma correction]
  I_gray_res.insert(I_gray_gamma_correction, vpImagePoint(0, I_gray.getWidth()));
  ss.str("");
  ss << "Sample_low_brightness_gray.png";
  vpImageIo::write(I_gray_res, ss.str());

  vpDisplay::display(I_gray_res);
  vpDisplay::displayText(I_gray_res, 20, 20, "Gamma correction on gray image. Click to continue.", vpColor::red);
  vpDisplay::flush(I_gray_res);
  vpDisplay::getClick(I_gray_res);

  I_color_res.insert(I_color_gamma_correction, vpImagePoint(0, I_color.getWidth()));
  ss.str("");
  ss << "Sample_low_brightness_gamma=" << gamma << ".png";
  vpImageIo::write(I_color_res, ss.str());

  vpDisplay::display(I_color_res);
  vpDisplay::displayText(I_color_res, 20, 20, "Gamma correction. Click to continue.", vpColor::red);
  vpDisplay::flush(I_color_res);
  vpDisplay::getClick(I_color_res);

  //! [Histogram equalization]
  vpImage<vpRGBa> I_color_equalize_histogram;
  vp::equalizeHistogram(I_color, I_color_equalize_histogram);
  //! [Histogram equalization]
  I_color_res.insert(I_color_equalize_histogram, vpImagePoint(0, I_color.getWidth()));
  ss.str("");
  ss << "Sample_low_brightness_eqHist.png";
  vpImageIo::write(I_color_res, ss.str());

  vpDisplay::display(I_color_res);
  vpDisplay::displayText(I_color_res, 20, 20, "Histogram equalization. Click to continue.", vpColor::red);
  vpDisplay::flush(I_color_res);
  vpDisplay::getClick(I_color_res);

  //! [Retinex]
  vpImage<vpRGBa> I_color_retinex;
  vp::retinex(I_color, I_color_retinex, scale, scaleDiv, level, dynamic, kernelSize);
  //! [Retinex]
  I_color_res.insert(I_color_retinex, vpImagePoint(0, I_color.getWidth()));

  ss.str("");
  ss << "Sample_low_brightness_scale=" << scale << "_scaleDiv=" << scaleDiv << "_level=" << level
    << "_dynamic=" << dynamic << "_kernelSize=" << kernelSize << ".png";
  vpImageIo::write(I_color_res, ss.str());

  vpDisplay::display(I_color_res);
  vpDisplay::displayText(I_color_res, 20, 20, "Retinex. Click to quit.", vpColor::red);
  vpDisplay::flush(I_color_res);
  vpDisplay::getClick(I_color_res);
#else
  (void)argc;
  (void)argv;
#endif
  return EXIT_SUCCESS;
}
