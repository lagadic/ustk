#include <visp3/core/vpException.h>
#include <visp3/ustk_io/usRawFileParser.h>
#include <fstream>
#include <iostream>

void usRawFileParser::read(usImagePostScan3D<unsigned char> &postScanImage3D, const std::string rawFilename)
{
  std::fstream fileStream(rawFilename.c_str(), std::ios::out | std::ios::binary);
  unsigned int i = 0;
  while (i<postScanImage3D.getSize()){
    fileStream.write ((char*)&postScanImage3D[i], sizeof (unsigned char));
    i++;
  }
    fileStream.close();
}

void usRawFileParser::write(const usImagePostScan3D<unsigned char> &postScanImage3D, const std::string rawFilename)
{
  std::ofstream fileStream(rawFilename.c_str());
  unsigned int i = 0;
  std::cout << "toto" << std::endl;
  while (i<postScanImage3D.getSize()){
    fileStream << postScanImage3D[i];
    std::cout << i << std::endl;
    i++;
  }
  fileStream.close();
}
