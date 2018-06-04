#include <visp3/ustk_core/usConfig.h>

#ifdef USTK_HAVE_VTK_QT
#include "usMeshDeformation.h"

#include <visp3/ustk_core/us.h>

#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkUnstructuredGridReader.h>

int main(int argc, char **argv)
{
  // QT application
  QApplication app(argc, argv);

  // setup view widget
  usMeshDeformation scene;

  // Create the mesh to display
  vtkSmartPointer<vtkUnstructuredGridReader> reader = vtkSmartPointer<vtkUnstructuredGridReader>::New();

  std::string filename = us::getDataSetPath() + "/mesh/liver.vtk";
  reader->SetFileName(filename.c_str());
  reader->Update();

  scene.setMeshInScene(reader->GetOutput());

  app.setActiveWindow(&scene);
  scene.showMaximized();

  return app.exec();
}
#else
#include <iostream>

int main() { std::cout << "Install vtk with qt4 or qt5 support to run this tutorial." << std::endl; }

#endif
