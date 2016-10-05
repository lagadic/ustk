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

public:

  typedef enum {
    MET_UNKNOWN = -1,
    MET_UCHAR,
    MET_SHORT,
    MET_DOUBLE,
  }ElementType;

  typedef enum {
    UNKNOWN = -1,
    RF_2D,
    RF_3D,
    PRESCAN_2D,
    PRESCAN_3D,
    POSTSCAN_2D,
    POSTSCAN_3D,
  }ImageType;

  struct MHDHeader
  {
    std::string mhdFileName;
    std::string rawFileName;
    unsigned int  numberOfDimensions;
    int  numberOfChannels;
    ElementType elementType;
    int dim[4];
    double elementSpacing[4];
    double position[4];
    int headerSize;
    bool msb;
    ImageType imageType;
    bool isProbeConvex;
    bool isMotorConvex;
    double probeRadius;
    double scanLinePitch;
    double motorRadius;
    double framePitch;
  };

  //Constructor
  usMetaHeaderParser();
  usMetaHeaderParser(std::string mhdFilename);
  //Desctructor
  virtual ~usMetaHeaderParser();

  // Data accessors.
  double getAxialResolution() const { return m_axialResolution; }
  double getHeightResolution() const { return m_heightResolution; }
  usImageSettings getImageSettings() const {return m_imageSettings;}
  usImage3DSettings getImageSettings3D() const {return m_imageSettings3D;}
  ImageType getImageType() const { return mhdHeader.imageType; }
  std::string getRawFileName() const {return mhdHeader.rawFileName;}
  double getWidthResolution() const { return m_widthResolution; }
  ElementType getElementType() const { return mhdHeader.elementType; }
  unsigned int getImageSizeX() const { return mhdHeader.dim[0]; }
  unsigned int getImageSizeY() const { return mhdHeader.dim[1]; }
  unsigned int getImageSizeZ() const { return mhdHeader.dim[2]; }
  MHDHeader getMhdHeader() const { return mhdHeader; }

  //comparison
  bool operator ==(usMetaHeaderParser const& other);

  void parse();
  //Read/write operations
  void read(const std::string& filename);
  void readMHDHeader(const char * fileName);
  void readMHDHeader(const std::string fileName);

  //Data setters
  void setImageSettings(const usImageSettings imageSettings);
  void setImageSettings3D(const usImage3DSettings imageSettings3D);
  void setImageFileName(const std::string imageFileName);
  void setAxialResolution(const double axialresolution);
  void setHeightResolution(const double heightResolution);
  void setWidthResolution(const double widthResolution);
  void setRawFileName(const std::string rawFileName);
  void setMhdHeader(MHDHeader header);

  void writeMHDHeader(const char * fileName);
  void writeMHDHeader(const std::string fileName);

private :
  usImageSettings m_imageSettings;
  usImage3DSettings m_imageSettings3D;
  double m_axialResolution;
  double m_heightResolution;
  double m_widthResolution;

  MHDHeader mhdHeader;
  std::map<std::string, int> imageTypeMap;
  std::map<std::string, int> elementTypeMap;
  std::map<int ,std::string> imageTypeReverseMap;
  std::map<int ,std::string> elementTypeReverseMap;
};
#endif //US_META_HEADER_PARSER_H
