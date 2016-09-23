#ifndef US_META_HEADER_PARSER_H
#define US_META_HEADER_PARSER_H

#include<string>
#include<map>
#include <cstdlib>
#include <string>
#include <ios>
#include <iostream>
#include <fstream>
#include <sstream>

#include <visp3/core/vpConfig.h>
#include <visp3/ustk_core/usImagePostScan2D.h>
#include <visp3/ustk_core/usImagePostScan3D.h>
#include <visp3/ustk_core/usImagePreScan2D.h>
#include <visp3/ustk_core/usImagePreScan3D.h>
#include <visp3/ustk_core/usImageRF2D.h>
#include <visp3/ustk_core/usImageRF3D.h>

class VISP_EXPORT usMetaHeaderParser {
private:


  std::map<std::string, int> elementTypeMap;

  typedef enum {
    MET_UNKNOWN = -1,
    MET_UCHAR,
    MET_SHORT,
    MET_DOUBLE,
  }ElementType;

  std::map<std::string, int> imageTypeMap;

  typedef enum {
    UNKNOWN = -1,
    RF_2D,
    RF_3D,
    PRESCAN_2D,
    PRESCAN_3D,
    POSTSCAN_2D,
    POSTSCAN_3D,
  }ImageType;

public:

  struct MHDHeader
  {
    std::string mhdFileName;
    std::string rawFileName;
    unsigned int  numberOfDimensions;
    int  numberOfChannels;
    ElementType elementType;
    unsigned int dim[4];
    float elementSpacing[4];
    float position[4];
    int headerSize;
    bool msb;
    ImageType imageType;
  };
  //Constructor
  usMetaHeaderParser();
  usMetaHeaderParser(std::string mhdFilename);

  //Desctructor
  virtual ~usMetaHeaderParser();

  MHDHeader readMHDHeader(const char * fileName);

/*  usImageSettings getImageSettings(MHDHeader mhdHeader);
  usImageSettings3D getImageSettings3D(MHDHeader mhdHeader);

  usImagePostScan2D getImagePostScan2D();
  usImagePostScan3D getImagePostScan3D();
  usImagePreScan2D getImagePreScan2D();
  usImagePreScan3D getImagePreScan3D();
  usImageRF2D getImageRF2D();
  usImageRF3D getImageRF3D();*/
};
#endif //US_META_HEADER_PARSER_H
