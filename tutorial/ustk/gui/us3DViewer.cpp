#include <visp3/ustk_gui/usGuiConfig.h>

#ifdef USTK_HAVE_VTK_QT

#include <visp3/ustk_core/us.h>
#include <visp3/ustk_core/usImagePostScan3D.h>
#include <visp3/ustk_io/usImageIo.h>
#include <visp3/ustk_gui/us3DSceneWidget.h>
#include <visp3/ustk_gui/usVTKConverter.h>
#include <vtkMetaImageReader.h>

int main(int argc, char** argv)
{
  std::string mhd_filename;

  for (int i=0; i<argc; i++) {
    if (std::string(argv[i]) == "--input")
      mhd_filename = std::string(argv[i+1]);
    else if (std::string(argv[i]) == "--help") {
      std::cout << "\nUsage: " << argv[0] << " [--input <preScan3D.xml>] [--help]\n" << std::endl;
      return 0;
    }
  }

  // Get the ustk-dataset package path or USTK_DATASET_PATH environment variable value
  if (mhd_filename.empty()) {
    std::string env_ipath = us::getDataSetPath();
    if (! env_ipath.empty())
      mhd_filename = env_ipath + "/pre-scan/3D_mhd/volume.mhd";
    else {
      std::cout << "You should set USTK_DATASET_PATH environment var to access to ustk dataset" << std::endl;
      return 0;
    }
  }

  vtkSmartPointer< vtkMetaImageReader > reader =
      vtkSmartPointer< vtkMetaImageReader >::New();
  reader->SetFileName(mhd_filename.c_str());
  reader->Update();

  std::cout << "reader output" << std::endl;

  reader->GetOutput()->Print(std::cout);

  // QT application
  QApplication app( argc, argv );


  //read us image
  usImagePostScan3D<unsigned char> postScan3D;
  usImageIo::read(postScan3D, mhd_filename);
  //conversion to vtk format
  vtkSmartPointer<vtkImageData> vtkImage = vtkSmartPointer<vtkImageData>::New();

  vtkSmartPointer<vtkImageImport> importer = vtkSmartPointer<vtkImageImport>::New();
  importer->SetDataScalarTypeToUnsignedChar();
  importer->SetWholeExtent(0,postScan3D.getDimX()-1,0, postScan3D.getDimY()-1, 0, postScan3D.getDimZ()-1);
  importer->SetDataExtentToWholeExtent();
  importer->SetNumberOfScalarComponents(1);
  usVTKConverter::convert(postScan3D,vtkImage,importer);

  vtkImage->Print(std::cout);

  //setup view widget
  us3DSceneWidget scene;
  scene.setImageData(vtkImage);

  double spacing[3];
  vtkImage->GetSpacing(spacing);

  vtkSmartPointer<vtkPlane> planeX = vtkSmartPointer<vtkPlane>::New();
  planeX->SetNormal(1,0,0);
  planeX->SetOrigin(postScan3D.getDimX()*spacing[0]/2.0,0,0);
  vtkSmartPointer<vtkPlane> planeY = vtkSmartPointer<vtkPlane>::New();
  planeY->SetNormal(0,1,0);
  planeY->SetOrigin(0,postScan3D.getDimY()*spacing[1]/2.0,0);
  vtkSmartPointer<vtkPlane> planeZ = vtkSmartPointer<vtkPlane>::New();
  planeZ->SetNormal(0,0,1);
  planeZ->SetOrigin(0,0,postScan3D.getDimZ()*spacing[2]/2.0);
  scene.setPlanes(planeX,planeY,planeZ);

  std::cout << "planeZ origin Z" << planeZ->GetOrigin()[2] << std::endl;

  scene.init();
  app.setActiveWindow(&scene);
  scene.show();

  return app.exec();
}
#else
#include <iostream>

int main()
{
  std::cout << "Install vtk with qt4 or qt5 support to run this tutorial." << std::endl;
}

#endif
