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
  usImageRF2D<short int> preComp2D, postComp2D;

  usImageIo::read(preComp2D, us::getDataSetPath() + std::string("/RFElasto/image00012.mhd"));
  usImageIo::read(postComp2D, us::getDataSetPath() + std::string("/RFElasto/image00015.mhd"));

  preComp.resize(preComp2D.getHeight(), preComp2D.getWidth(), 5);
  postComp.resize(postComp2D.getHeight(), postComp2D.getWidth(), 5);

  preComp.setImagePreScanSettings(preComp2D);
  postComp.setImagePreScanSettings(postComp2D);

  preComp.setMotorRadius(0.02725);
  postComp.setMotorRadius(0.02725);
  preComp.setMotorType(usMotorSettings::TiltingMotor);
  postComp.setMotorType(usMotorSettings::TiltingMotor);
  preComp.setFramePitch(0.0255342);
  postComp.setFramePitch(0.0255342);

  for (unsigned int i = 0; i < 5; i++) {
    preComp.insertFrame(preComp2D, i);
    postComp.insertFrame(postComp2D, i);
  }

  // elasto process
  usElastography3D elsastoProcess;

  elsastoProcess.setPreCompression(preComp);
  elsastoProcess.setPostCompression(postComp);
  elsastoProcess.setROI(40, 2500, 1, 50, 500, 2);

  usImagePreScan3D<unsigned char> elastResult;
  elastResult.setImagePreScanSettings(preComp);

  elastResult.setData(elsastoProcess.run());

  usImageIo::write(elastResult, us::getDataSetPath() + std::string("/volumePostComp.mhd"));

  std::cout << "output written in " << us::getDataSetPath() + std::string("/volumePostComp.mhd") << std::endl;

  return 0;
}
#else
int main()
{
  std::cout << "You should intall FFTW to run this tutorial" << std::endl;
  return 0;
}

#endif
