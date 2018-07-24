//! \example tutorial-elastography-3D-basic.cpp

#include <iostream>
#include <visp3/ustk_core/usConfig.h>

#if defined(USTK_HAVE_FFTW)

#include <visp3/ustk_core/usImageIo.h>
#include <visp3/ustk_elastography/usElastography3D.h>
#include <visp3/ustk_grabber/usNetworkGrabberRF3D.h>

int main(int argc, char **argv)
{
  // input volumes
  usImageRF3D<short int> preComp, postComp;

  usImageIo::read(preComp, us::getDataSetPath() + std::string("volumePreComp.mhd"));
  usImageIo::read(postComp, us::getDataSetPath() + std::string("volumePostComp.mhd"));

  // elasto process
  usElastography3D elsastoProcess;

  elsastoProcess.setPreCompression(preComp);
  elsastoProcess.setPostCompression(postComp);

  usImagePreScan3D<unsigned char> elastResult;
  elastResult.setData(elsastoProcess.run());

  usImageIo::write(elastResult, us::getDataSetPath() + std::string("volumePostComp.mhd"));

  return 0;
}
#else
int main()
{
  std::cout << "You should intall FFTW to run this tutorial" << std::endl;
  return 0;
}

#endif
