#include <QtGui/QMainWindow>
#include <QtGui/QApplication>

#include <vtkMetaImageWriter.h>

#include <visp3/ustk_io/usImageIo.h>
#include <visp3/ustk_gui/usVTKConverter.h>
#include <visp3/ustk_gui/usMedicalImageViewer.h>

int main( int argc, char** argv )
{
  //read the us data
  usImagePreScan3D<unsigned char> preScanImage;
  usImageIo::read(preScanImage,"/home/mpouliqu/Documents/ustk-dataset/3D/volumeTest.mhd");

  std::cout << preScanImage.getDimX() << std::endl;
  std::cout << preScanImage.getDimY() << std::endl;
  std::cout << preScanImage.getDimZ() << std::endl;

  //convert to vtkImageData to display the image
  vtkSmartPointer<vtkImageData> vtkImage;
  std::cout << "begin conversion" << std::endl;
  double t1 = vpTime::measureTimeMs();

  usVTKConverter converter;
  converter.convert(preScanImage,vtkImage);
  double t2 = vpTime::measureTimeMs();

  std::cout << "vtk convert time = " << t2-t1 << std::endl;

  std::cout << "vtk ptr = " << vtkImage->GetScalarPointer(0,0,0) << std::endl;

  unsigned char* ptr = (unsigned char*) vtkImage->GetScalarPointer(0,0,0);

  int i=0;
  while (i<26) {
    std::cout << "vtkImage [" << i << "] = " << (int) *ptr << std::endl;
    ptr++;
    i++;
  }

/*
  for (int i = 0; i < preScanImage.getDimX(); i++) {
    for (int j = 0; j < preScanImage.getDimY(); j++) {
      for (int k = 0; k < preScanImage.getDimZ(); k++) {
        //std::cout << "i = " << i << ", j = " << j << ", k = " << k << std::endl;
        std::cout << "imageData [" << i << "] [" << j << "] [" << k << "] = " << vtkImage->GetScalarComponentAsDouble(i,j,k,0) << std::endl;
      }
    }
  }
*/
  //write mhd using VTK
  vtkSmartPointer<vtkMetaImageWriter> writer = vtkSmartPointer<vtkMetaImageWriter>::New();
  writer->SetFileName("/home/mpouliqu/Documents/ustk-dataset/3D/volumeTestWritingVTK.mhd");
  writer->SetInputData(vtkImage);
  writer->Write();

/*
  // QT application
  QApplication app( argc, argv );

  //std::string fileName = "/home/mpouliqu/Documents/ustk-dataset/pre-scan/3D_mhd/volume.mhd";
  usMedicalImageViewer medicalImageViewer(vtkImage);
  medicalImageViewer.show();

  return app.exec();*/
  return 0;
}
