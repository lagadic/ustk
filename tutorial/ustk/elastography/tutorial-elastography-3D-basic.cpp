//! \example tutorial-elastography-3D-basic.cpp

#include <iostream>
#include <visp3/ustk_core/usConfig.h>

#if defined(USTK_HAVE_FFTW)

#include <visp3/ustk_core/usImageIo.h>
#include <visp3/ustk_elastography/usElastography3D.h>

int main()
{
  // input volumes
  usImageRF3D<short int> preComp, postComp;

  if (us::getDataSetPath().empty()) {
    std::cout << "You should set USTK_DATASET_PATH environment var to access to ustk dataset containing RF volumes for "
                 "elastography"
              << std::endl;
    return 0;
  }

  usImageIo::read(preComp, us::getDataSetPath() + std::string("/RFElasto3D/preCompressed/image00000.mhd"));
  usImageIo::read(postComp, us::getDataSetPath() + std::string("/RFElasto3D/postCompressed/image00000.mhd"));

  // elasto process
  usElastography3D elsastoProcess;
  elsastoProcess.setPreCompression(preComp);
  elsastoProcess.setPostCompression(postComp);
  elsastoProcess.setROI(40, 2500, 4, 50, 500, 5);

  usImagePreScan3D<unsigned char> elastResult;
  elastResult.setImagePreScanSettings(preComp);

  elastResult.setData(elsastoProcess.run());

  usImageIo::write(elastResult, std::string("elastResult.mhd"));

  std::cout << "output written in elastResult.mhd" << std::endl;

  return 0;
}
#else
int main()
{
  std::cout << "You should intall FFTW to run this tutorial" << std::endl;
  return 0;
}

#endif
