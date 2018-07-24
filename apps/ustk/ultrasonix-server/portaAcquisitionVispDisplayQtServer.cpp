// std includes
#include <iostream>

// Visp includes
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpImage.h>

// Qt includes
#include <QtWidgets/QApplication>

// local includes
#include "usNetworkServer.h"

int main(int argc, char *argv[])
{
  QApplication a(argc, argv);
  usNetworkServer server;

  if (a.arguments().contains(QString("--probeSettingsFile"))) {
    std::cout << "using settings file : "
              << a.arguments().at(a.arguments().indexOf(QString("--probeSettingsFile")) + 1).toStdString() << std::endl;
    server.useProbeConfigFile(
        a.arguments().at(a.arguments().indexOf(QString("--probeSettingsFile")) + 1).toStdString());
  }

  return a.exec();
}
