#include <visp3/core/vpException.h>
#include <visp3/ustk_io/usRawFileParser.h>
#include <fstream>
#include <iostream>

void usRawFileParser::read(usImage3D<unsigned char> &image3D, const std::string rawFilename)
{
  std::fstream fileStream(rawFilename.c_str(), std::ios::out | std::ios::binary);
  unsigned int i = 0;
  while (i<image3D.getSize()){
    fileStream.write ((char*)&image3D[i], sizeof (unsigned char));
    i++;
  }
  fileStream.close();
}

void usRawFileParser::write(const usImage3D<unsigned char> &image3D, const std::string rawFilename)
{
  std::ofstream fileStream(rawFilename.c_str());
  unsigned int i = 0;
  while (i<image3D.getSize()){
    fileStream << image3D[i];
    i++;
  }
  fileStream.close();
}

void usRawFileParser::read(vpImage<unsigned char> &image2D, const std::string rawFilename)
{
  std::fstream fileStream(rawFilename.c_str(), std::ios::out | std::ios::binary);
  unsigned int i = 0;
  unsigned int j = 0;
  while (i<image2D.getSize()) {
    if (j==image2D.getHeight())
      j = 0;
    fileStream.write((char*)&image2D[i][j], sizeof(unsigned char));
    i++;
    j++;
  }
  fileStream.close();
}

void usRawFileParser::write(const vpImage<unsigned char> &image2D, const std::string rawFileName)
{
  std::ofstream fileStream(rawFileName.c_str());
  unsigned int i = 0;
  unsigned int j = 0;
  while (i<image2D.getSize()) {
    if (j == image2D.getHeight())
      j = 0;
    fileStream << image2D[i][j];
    i++;
    j++;
  }
  fileStream.close();
}
