#include <visp3/ustk_gui/usGuiConfig.h>

#ifdef USTK_HAVE_VTK_QT

#include <vtkMetaImageWriter.h>
#include <vtkImageImport.h>
#include <visp3/ustk_io/usImageIo.h>
#include <visp3/ustk_gui/usVTKConverter.h>

int main()
{
  //read the us data
  usImagePostScan3D<unsigned char> postScanImage;
  //usImageIo::read(postScanImage,"/home/mpouliqu/Documents/ustk-dataset/3D/volumeTest.mhd");
  usImageIo::read(postScanImage,"/home/mpouliqu/Documents/usData/prescan/3D/USpreScan_volume-0000/volume.mhd");
  //usImageIo::read(postScanImage,"/udd/mpouliqu/soft/ustk/ustk-dataset/pre-scan/3D_mhd/volume.mhd");

  //convert to vtkImageData to display the image
  vtkSmartPointer<vtkImageData> vtkImage;

  //pre-compute the importer to save time during the data import in vtk
  vtkImageImport *importer = vtkImageImport::New();
  importer->SetDataScalarTypeToUnsignedChar();
  importer->SetImportVoidPointer((void *)postScanImage.getConstData());
  importer->SetWholeExtent(0,postScanImage.getDimX()-1,0, postScanImage.getDimY()-1, 0, postScanImage.getDimZ()-1);
  importer->SetDataExtentToWholeExtent();
  importer->SetNumberOfScalarComponents(1);

  double t1 = vpTime::measureTimeMs();

  usVTKConverter::convert(postScanImage,vtkImage,importer);
  double t2 = vpTime::measureTimeMs();

  std::cout << "vtk convert time = " << t2-t1 << std::endl;

  //picking random voxels to compare pointers
  /*std::cout << "us ptr = " << (void*)postScanImage.getConstData() << std::endl;
  std::cout << "vtk ptr = " << vtkImage->GetScalarPointer(0,0,0) << std::endl;

  unsigned char *ptr = postScanImage.getConstData();
  ptr += (postScanImage.getDimX() * postScanImage.getDimY()) * 10 + postScanImage.getDimX()*10 + 10;
  std::cout << "us ptr (10,10,10) = " << (void*)ptr << std::endl;
  std::cout << "vtk ptr (10,10,10) = " << vtkImage->GetScalarPointer(10,10,10) << std::endl;*/

  //write mhd using VTK
  vtkSmartPointer<vtkMetaImageWriter> writer = vtkSmartPointer<vtkMetaImageWriter>::New();
  writer->SetFileName("/home/mpouliqu/Documents/ustk-dataset/3D/volumeTestWritingVTK.mhd");
  writer->SetInputData(vtkImage);
  writer->Write();

  return 0;
}
#else
#include <iostream>

int main()
{
  std::cout << "Install vtk with qt4 or qt5 support to run this tutorial." << std::endl;
}

#endif
