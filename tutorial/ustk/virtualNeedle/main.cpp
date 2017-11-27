#include <visp3/ustk_core/usConfig.h>

#ifdef USTK_HAVE_VTK_QT
#include "usVirtualNeedle.h"
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>

int main(int argc, char **argv)
{
  // QT application
  QApplication app(argc, argv);

  // setup view widget
  usVirtualNeedle scene;

  // Create the mesh to display
  vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->SetCenter(0, 0.01, 0.01);
  sphereSource->SetRadius(0.01);
  sphereSource->Update();

  scene.setMeshInScene(sphereSource->GetOutput());

  app.setActiveWindow(&scene);
  scene.show();
  scene.showMaximized();

  return app.exec();
}
#else
#include <iostream>

int main() { std::cout << "Install vtk with qt4 or qt5 support to run this tutorial." << std::endl; }

#endif
