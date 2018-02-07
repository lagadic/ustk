//! \example tutorial-elastography-2D-basic.cpp

#include <iostream>
#include <visp3/ustk_core/usConfig.h>

#if defined(USTK_HAVE_FFTW)

#include <visp3/ustk_elastography/usElastography.h>

#include <visp3/io/vpImageIo.h>
#include <visp3/ustk_io/usImageIo.h>

int main()
{

  // prepare image;
  usImageRF2D<short int> preComp;
  usImageRF2D<short int> postComp;

  std::string image1 = us::getDataSetPath() + std::string("/RFElasto/image00012.mhd");
  std::string image2 = us::getDataSetPath() + std::string("/RFElasto/image00015.mhd");

  usImageIo::read(preComp, image1.c_str());
  usImageIo::read(postComp, image2.c_str());

  usElastography elastography(preComp, postComp);
  elastography.setROI(40, 2500, 50, 500);

  // computate elasto
  vpImage<unsigned char> strainImage;
  strainImage = elastography.run();

  vpImageIo::write(strainImage, "outputElasto.png");

  return 0;
}

#else
int main()
{
  std::cout << "You should intall FFTW to run this tutorial" << std::endl;
  return 0;
}

#endif
