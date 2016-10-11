#include <visp3/core/vpException.h>
#include <visp3/ustk_io/usRawFileParser.h>
#include <fstream>
#include <iostream>

void usRawFileParser::read(usImage3D<unsigned char> &image3D, const std::string rawFilename)
{
  std::fstream fileStream(rawFilename.c_str(), std::ios::in | std::ios::binary);
  unsigned int i = 0;
  while (i<image3D.getSize()){
    fileStream >> image3D[i];
    i++;
  }
  fileStream.close();
}

void usRawFileParser::write(const usImage3D<unsigned char> &image3D, const std::string rawFilename)
{
  std::fstream fileStream(rawFilename.c_str(), std::ios::out | std::ios::binary);
  unsigned int i = 0;
  while (i<image3D.getSize()){
    fileStream << image3D[i];
    i++;
  }
  fileStream.close();
}

void usRawFileParser::read(vpImage<unsigned char> &image2D, const std::string rawFilename)
{
  std::fstream fileStream(rawFilename.c_str(), std::ios::in | std::ios::binary);
  for (unsigned int i = 0; i < image2D.getWidth(); i++) {
    for (unsigned int j = 0; j < image2D.getHeight(); j++) {
      fileStream >> image2D[j][i];
    }
  }
  fileStream.close();
}

void usRawFileParser::write(const vpImage<unsigned char> &image2D, const std::string rawFilename)
{
  std::fstream fileStream(rawFilename.c_str(), std::ios::out | std::ios::binary);
  for (unsigned int i = 0; i < image2D.getWidth(); i++) {
    for (unsigned int j = 0; j < image2D.getHeight(); j++) {
      fileStream << image2D[j][i];
    }
  }
  fileStream.close();
}
