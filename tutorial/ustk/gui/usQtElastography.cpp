#include <visp3/ustk_core/usConfig.h>

#if defined(USTK_HAVE_VTK_QT) && defined(USTK_HAVE_ELASTOGRAPHY) && defined(USTK_HAVE_GRABBER)

#include <QApplication>
#include <visp3/ustk_gui/usElastographyMainWindow.h>

int main(int argc, char **argv)
{
  // QT application
  std::cout << "A\n";
  QApplication app(argc, argv);
  std::cout << "B\n";

  usElastographyMainWindow *window = new usElastographyMainWindow;
  app.setActiveWindow(window);
  return app.exec();
}
#else
#include <iostream>

int main() { std::cout << "Install vtk with qt4 or qt5 support to run this tutorial." << std::endl; }

#endif
