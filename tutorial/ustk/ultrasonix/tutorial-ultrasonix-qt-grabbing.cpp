//! \example tutorial-ultrasonix-qt-grabbing.cpp

#include <visp3/ustk_grabber/usNetworkGrabber.h>

#if defined(USTK_HAVE_QT4) || defined(USTK_HAVE_QT5)

#include <QThread>
#include <QApplication>

int main(int argc, char** argv)
{
  // QT application
  QApplication app( argc, argv );


  usNetworkGrabber * qtGrabber = new usNetworkGrabber();



  //QThread * thread = new QThread();
  qtGrabber->setConnection(true);
  //qtGrabber->moveToThread(thread);

  usNetworkGrabber::usInitHeaderSent header;
  header.imagingMode = 0; //B-mode
  header.imageHeight = 224;
  header.frequency = 0; //not used for now

  qtGrabber->initAcquisition(header);

  app.exec();

  std::cout << "ending application" << std::endl;
  return 0;
}

#else
int main()
{
  std::cout << "You should intall Qt4 or Qt5 to run this tutorial" << std::endl;
}

#endif
