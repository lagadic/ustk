#include <visp3/ustk_core/usScanConverter3D.h>
#include <visp3/ustk_io/usImageIo.h>
#include <visp3/ustk_io/usSequenceReader.h>
#include <visp3/core/vpTime.h>

int main(int argc, char** argv)
{
  std::string mhd_filename;

  for (int i=0; i<argc; i++) {
    if (std::string(argv[i]) == "--input")
      mhd_filename = std::string(argv[i+1]);
    else if (std::string(argv[i]) == "--help") {
      std::cout << "\nUsage: " << argv[0] << " [--input <preScan3D.xml>] [--help]\n" << std::endl;
      return 0;
    }
  }

  // Get the ustk-dataset package path or USTK_DATASET_PATH environment variable value
  if (mhd_filename.empty()) {
    std::string env_ipath = us::getDataSetPath();
    if (! env_ipath.empty())
      mhd_filename = env_ipath + "/pre-scan/3D_xml/sequencepreScan3D.xml";
    else {
      std::cout << "You should set USTK_DATASET_PATH environment var to access to ustk dataset" << std::endl;
      return 0;
    }
  }

/*  TEST : WRITE A SEQUENCE OF VOXEL IN A IMAGE
  usImagePreScan3D<unsigned char> prescanImage;
  prescanImage.resize(3,3,3);

  prescanImage(0,0,0,0);
  prescanImage(1,0,0,1);
  prescanImage(2,0,0,2);
  prescanImage(0,1,0,3);
  prescanImage(1,1,0,4);
  prescanImage(2,1,0,5);
  prescanImage(0,2,0,6);
  prescanImage(1,2,0,7);
  prescanImage(2,2,0,8);
  prescanImage(0,0,1,9);
  prescanImage(1,0,1,10);
  prescanImage(2,0,1,11);
  prescanImage(0,1,1,12);
  prescanImage(1,1,1,13);
  prescanImage(2,1,1,14);
  prescanImage(0,2,1,15);
  prescanImage(1,2,1,16);
  prescanImage(2,2,1,17);
  prescanImage(0,0,2,18);
  prescanImage(1,0,2,19);
  prescanImage(2,0,2,20);
  prescanImage(0,1,2,21);
  prescanImage(1,1,2,22);
  prescanImage(2,1,2,23);
  prescanImage(0,2,2,24);
  prescanImage(1,2,2,25);
  prescanImage(2,2,2,26);

  usImageIo::write(prescanImage,"volumeTest.mhd");
*/

  usImagePreScan3D<unsigned char> prescanImage;
  prescanImage.resize(128,480,16);
  usImagePostScan3D<unsigned char> postscanImage;

  usImageIo::read(prescanImage,mhd_filename);

  std::cout << "end reading" << std::endl;

  //scan-conversion
  usScanConverter3D converter;

  double startTime = vpTime::measureTimeMs();
  std::cout << "init converter..." << std::endl;

  converter.init(prescanImage);

  double endInitTime = vpTime::measureTimeMs();
  std::cout << "init time (sec) = " << (endInitTime - startTime) / 1000.0 << std::endl;

  std::cout << "converting..." << std::endl;
  converter.convert(postscanImage);

  double endConvertTime = vpTime::measureTimeMs();
  std::cout << "convert time (sec) = " << (endConvertTime - endInitTime) / 1000.0 << std::endl;

  std::cout << "writing post-scan..." << std::endl;
  std::string mhdFileName ="volume.mhd";
  usImageIo::write(postscanImage,mhdFileName);

  return 0;
}
