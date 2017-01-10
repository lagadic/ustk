#include <visp3/ustk_core/usScanConverter3D.h>
#include <visp3/ustk_io/usImageIo.h>
#include <visp3/ustk_io/usSequenceReader.h>
#include <visp3/core/vpTime.h>

int main()
{
  usImagePreScan2D<unsigned char> frame;
  usImagePreScan3D<unsigned char> prescanImage;
  prescanImage.resize(128,480,16);
  usImagePostScan3D<unsigned char> postscanImage;

  //get ultrasonix settings for scan conversion
  /*us::getUltrasonix4DC7MotorSettings(prescanImage);
  us::getUltrasonix4DC7PreScanSettings(prescanImage);

  //reading the set of images
  usSequenceReader<usImagePreScan2D<unsigned char> >reader;
  reader.setSequenceFileName("/home/mpouliqu/Documents/usData/prescan/3D/USpreScan_volume-0000/sequencepreScan2D.xml");
  std::cout << "reading images..." << std::endl;
  int i = 0;
  while(!reader.end()) {
    reader.acquire(frame);
    prescanImage.insertFrame(frame,i);
    i++;
  }*/

  usImageIo::read(prescanImage,"/home/mpouliqu/Documents/ustk-dataset/pre-scan/3D_xml/sequencepreScan3D.xml");
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
  std::string mhdFileName ="/home/mpouliqu/Documents/usData/prescan/3D/USpreScan_volume-0000/volume.mhd";
  usImageIo::write(postscanImage,mhdFileName);

  return 0;
}
