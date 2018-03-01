//! \example tutorial-ultrasonix-qt-grabbing-pre-scan3D-display.cpp

#include <iostream>
#include <visp3/ustk_core/usConfig.h>

#if defined(VISP_HAVE_MODULE_USTK_GUI) && defined(VISP_HAVE_MODULE_USTK_GRABBER)

#include <QApplication>
#include <QtCore/QThread>

#include <visp3/ustk_grabber/usNetworkGrabberPreScan3D.h>

#include <visp3/ustk_io/usImageIo.h>

#include <visp3/ustk_gui/us3DSceneWidget.h>
#include <visp3/ustk_gui/usVTKConverter.h>

int main(int argc, char **argv)
{
  // QT application
  QApplication app(argc, argv);

  ////////////  Init Network grabber ////////////
  QThread *grabbingThread = new QThread();

  usNetworkGrabberPreScan3D *qtGrabber = new usNetworkGrabberPreScan3D();
  qtGrabber->connectToServer();

  // setting acquisition parameters
  usNetworkGrabber::usInitHeaderSent header;
  header.probeId = 15;    // 4DC7 id = 15
  header.slotId = 0;      // top slot id = 0
  header.imagingMode = 0; // B-mode = 0

  // prepare image;
  usVolumeGrabbedInfo<usImagePreScan3D<unsigned char> > *grabbedFrame;

  bool captureRunning = true;

  // sending acquisition parameters
  qtGrabber->initAcquisition(header);

  qtGrabber->setStepsPerFrame(usAcquisitionParameters::US_ANGLE_PITCH_3);
  qtGrabber->setFramesPerVolume(25);
  qtGrabber->setMotorActivation(true);
  qtGrabber->sendAcquisitionParameters();

  ////////////  Init VTK scene ////////////
  // pre-compute the importer to save time during the data import in vtk
  // this importer will switch between the 2 previsous usImages at each iteration
  vtkSmartPointer<vtkImageData> vtkImage = vtkSmartPointer<vtkImageData>::New();

  // setup view widget
  us3DSceneWidget scene;

  // Send the command to run the acquisition
  qtGrabber->runAcquisition();

  // Move the grabber object to another thread, and run it
  qtGrabber->moveToThread(grabbingThread);
  grabbingThread->start();

  bool init = false;

  // our grabbing loop
  do {
    if (qtGrabber->isFirstFrameAvailable()) {
      grabbedFrame = qtGrabber->acquire();

      std::cout << "MAIN THREAD received volume No : " << grabbedFrame->getVolumeCount() << std::endl;

      if (!init) {

        usVTKConverter::convert(*grabbedFrame, vtkImage);

        scene.setImageData(vtkImage);

        double spacing[3];
        vtkImage->GetSpacing(spacing);

        vtkSmartPointer<vtkPlane> planeX = vtkSmartPointer<vtkPlane>::New();
        planeX->SetNormal(1, 0, 0);
        planeX->SetOrigin(grabbedFrame->getWidth() * spacing[0] / 2.0, 0, 0);
        vtkSmartPointer<vtkPlane> planeY = vtkSmartPointer<vtkPlane>::New();
        planeY->SetNormal(0, 1, 0);
        planeY->SetOrigin(0, grabbedFrame->getHeight() * spacing[1] / 2.0, 0);
        vtkSmartPointer<vtkPlane> planeZ = vtkSmartPointer<vtkPlane>::New();
        planeZ->SetNormal(0, 0, 1);
        planeZ->SetOrigin(0, 0, grabbedFrame->getNumberOfFrames() * spacing[2] / 2.0);
        scene.setPlanes(planeX, planeY, planeZ);

        scene.init();
        scene.show();

        init = true;
      } else {

        usVTKConverter::convert(*grabbedFrame, vtkImage);

        scene.setImageData(vtkImage);
      }
    } else {
      vpTime::wait(10);
    }
  } while (captureRunning);

  return app.exec();
}

#else
int main()
{
  std::cout << "You should build ustk_gui and ustk_grabber to run this tutorial" << std::endl;
  return 0;
}

#endif
