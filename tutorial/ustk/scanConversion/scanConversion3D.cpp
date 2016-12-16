#include <visp3/ustk_core/usScanConverter3D.h>
#include <visp3/ustk_io/usImageIo.h>
#include <visp3/ustk_io/usSequenceReader.h>

int main(int argc, const char *argv[])
{
  usImagePreScan2D<unsigned char> frame;
  usImagePreScan3D<unsigned char> prescanImage;
  prescanImage.resize(128,480,31);
  usImagePostScan3D<unsigned char> postscanImage;

  std::cout << "volume size : " << prescanImage.getSize();

  //reading the set of images
  usSequenceReader<usImagePreScan2D<unsigned char> >reader;
  reader.setSequenceFileName("/home/mpouliqu/Documents/usData/prescan/3D/USpreScan_volume-0000/sequencepreScan2D.xml");
  std::cout << "reading images..." << std::endl;
  int i = 0;
  while(!reader.end()) {
    std::cout << "reading image : " << i << std::endl;
    reader.acquire(frame);
    std::cout << "inserting image : " << i << std::endl;
    prescanImage.insertFrame(frame,i);
    i++;
  }
  std::cout << "end reading" << std::endl;
  //scan-conversion
  usScanConverter3D converter;
  std::cout << "init converter" << std::endl;
  converter.init(128,480,31);
  std::cout << "setting volume in converter" << std::endl;
  converter.setVolume(prescanImage);
  std::cout << "converting" << std::endl;
  converter.convert();
  std::cout << "writing post-scan" << std::endl;
  converter.getVolume(postscanImage);
  std::string mhdFileName ="/home/mpouliqu/Documents/usData/prescan/3D/USpreScan_volume-0000/volume.mhd";
  usImageIo::write(postscanImage,mhdFileName);

  return 0;
}
