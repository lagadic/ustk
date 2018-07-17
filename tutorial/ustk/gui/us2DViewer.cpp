#include <visp3/ustk_core/usConfig.h>

#ifdef USTK_HAVE_VTK_QT

#include <visp3/ustk_core/us.h>
#include <visp3/ustk_core/usImageIo.h>
#include <visp3/ustk_core/usImagePostScan3D.h>
#include <visp3/ustk_gui/us2DSceneWidget.h>
#include <visp3/ustk_gui/usVTKConverter.h>
#include <vtkMetaImageReader.h>

int main(int argc, char **argv)
{
  std::string mhd_filename;

  for (int i = 0; i < argc; i++) {
    if (std::string(argv[i]) == "--input")
      mhd_filename = std::string(argv[i + 1]);
    else if (std::string(argv[i]) == "--help") {
      std::cout << "\nUsage: " << argv[0] << " [--input <preScan3D.xml>] [--help]\n" << std::endl;
      return 0;
    }
  }

  // Get the ustk-dataset package path or USTK_DATASET_PATH environment variable value
  if (mhd_filename.empty()) {
    std::string env_ipath = us::getDataSetPath();
    if (!env_ipath.empty())
      mhd_filename = env_ipath + "/post-scan/3D_mhd/volume.mhd";
    else {
      std::cout << "You should set USTK_DATASET_PATH environment var to access to ustk dataset" << std::endl;
      return 0;
    }
  }
  // QT application
  QApplication app(argc, argv);

  // read us image
  usImagePostScan3D<unsigned char> postScan3D;
  usImageIo::read(postScan3D, mhd_filename);
  // conversion to vtk format
  vtkSmartPointer<vtkImageData> vtkImage = vtkSmartPointer<vtkImageData>::New();
  usVTKConverter::convert(postScan3D, vtkImage);

  int dims[3];
  vtkImage->GetDimensions(dims);
  double spacing[3];
  vtkImage->GetSpacing(spacing);

  // set reslice matrix
  vpHomogeneousMatrix mat;
  mat.eye();
  mat.buildFrom(0, 0, 0.05, 0, 0, 0);

  vtkMatrix4x4 *vtkMat = vtkMatrix4x4::New();
  usVTKConverter::convert(mat, vtkMat);

  // setup view widget
  us2DSceneWidget *scene = new us2DSceneWidget();
  scene->setImageData(vtkImage);
  scene->setResliceMatrix(vtkMat);
  scene->init();
  scene->show();

  // extract 2D slice
  usImagePostScan2D<unsigned char> image2D;
  scene->getCurrentSlice(image2D); // We get only the pixel values and pixel size. Transducer settings are not set when
                                   // we extract a post-scan 2D slice.

  // We write it
  usImageIo::write(image2D, "slice.xml");

  // wait until user closes the window
  return app.exec();
}
#else
#include <iostream>

int main() { std::cout << "Install vtk with qt4 or qt5 support to run this tutorial." << std::endl; }

#endif
