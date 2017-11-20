#include <visp3/ustk_core/usConfig.h>

#ifdef USTK_HAVE_VTK_QT

#include <visp3/ustk_gui/usVTKConverter.h>
#include <visp3/ustk_io/usImageIo.h>
#include <vtkImageImport.h>
#include <vtkMetaImageWriter.h>

int main()
{
  std::string mhd_filename;
  // Get the ustk-dataset package path or USTK_DATASET_PATH environment variable value
  std::string env_ipath = us::getDataSetPath();
  if (!env_ipath.empty())
    mhd_filename = env_ipath + "/post-scan/3D_mhd/volume.mhd";
  else {
    std::cout << "You should set USTK_DATASET_PATH environment var to access to ustk dataset" << std::endl;
    return 1;
  }

  // read image
  usImagePostScan3D<unsigned char> postScanImage;
  usImageIo::read(postScanImage, mhd_filename);

  // Import image in vtk type
  vtkSmartPointer<vtkImageData> vtkImage = vtkSmartPointer<vtkImageData>::New();
  vtkImageImport *importer = vtkImageImport::New();
  importer->SetDataScalarTypeToUnsignedChar();
  importer->SetImportVoidPointer((void *)postScanImage.getConstData());
  importer->SetWholeExtent(0, postScanImage.getDimX() - 1, 0, postScanImage.getDimY() - 1, 0,
                           postScanImage.getDimZ() - 1);
  importer->SetDataExtentToWholeExtent();
  importer->SetNumberOfScalarComponents(1);

  usVTKConverter::convert(postScanImage, vtkImage, importer);

  // picking random voxels to compare their pointers to make sure the conversion was good

  // voxel 0,0,0
  if (vtkImage->GetScalarPointer(0, 0, 0) != postScanImage.getConstData())
    return 1;

  // voxel 10,10,10
  if (vtkImage->GetScalarPointer(10, 10, 10) != postScanImage.getData(10, 10, 10))
    return 1;

  // voxel of middle or image
  int middleX = postScanImage.getDimX() / 2;
  int middleY = postScanImage.getDimY() / 2;
  int middleZ = postScanImage.getDimZ() / 2;
  if (vtkImage->GetScalarPointer(middleX, middleY, middleZ) != postScanImage.getData(middleX, middleY, middleZ))
    return 1;

  return 0;
}
#else
#include <iostream>

int main() { std::cout << "Install vtk with qt4 or qt5 support to run this test." << std::endl; }

#endif
