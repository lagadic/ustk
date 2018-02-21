//! \example tutorial-elastography-2D-basic.cpp

#include <iostream>
#include <visp3/ustk_core/usConfig.h>

#if defined(VISP_HAVE_MODULE_USTK_GUI) && defined(VISP_HAVE_MODULE_USTK_ELASTOGRAPHY)

#include <visp3/ustk_gui/usElastographyMainWindow.h>

#include <QApplication>

int main(int argc, char **argv)
{
  QApplication app(argc, argv);
  app.setApplicationName(QString("USTK elastography demo"));

  usElastographyMainWindow window;

  window.setElastoROI(QRect(40, 2500, 50, 500));
  window.show();

  return app.exec();
}

#else
int main()
{
  std::cout << "You should build ustk_gui and ustk_elastography to run this tutorial" << std::endl;
  return 0;
}

#endif
