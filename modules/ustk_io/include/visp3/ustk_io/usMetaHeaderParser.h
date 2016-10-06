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
    NOT_SET,
    RF_2D,
    RF_3D,
    PRESCAN_2D,
    PRESCAN_3D,
    POSTSCAN_2D,
    POSTSCAN_3D,
  }ImageType;

  struct MHDHeader
  {
    std::string MHDFileName;
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
    bool isTransducerConvex;
    bool isMotorRotating;
    double probeRadius;
    double scanLinePitch;
    double motorRadius;
    double framePitch;
  };

  //Constructor
  usMetaHeaderParser();
  usMetaHeaderParser(std::string MHDFilename);
  //Desctructor
  virtual ~usMetaHeaderParser();

  // Data accessors.
  double getAxialResolution() const { return m_axialResolution; }
  double getHeightResolution() const { return m_heightResolution; }
  usTransducerSettings getTransducerSettings() const {return m_transducerSettings;}
  usMotorSettings getMotorSettings() const {return m_motorSettings;}
  ImageType getImageType() const { return header.imageType; }
  std::string getRawFileName() const {return header.rawFileName;}
  double getWidthResolution() const { return m_widthResolution; }
  ElementType getElementType() const { return header.elementType; }
  unsigned int getImageSizeX() const { return header.dim[0]; }
  unsigned int getImageSizeY() const { return header.dim[1]; }
  unsigned int getImageSizeZ() const { return header.dim[2]; }
  MHDHeader getMHDHeader() const { return header; }

  //comparison
  bool operator ==(usMetaHeaderParser const& other);

  void parse();
  //Read/write operations
  void read(const std::string& filename);
  void readMHDHeader(const std::string fileName);

  //Data setters
  void setTransducerSettings(const usTransducerSettings transducerSettings);
  void setMotorSettings(const usMotorSettings motorSettings);
  void setImageFileName(const std::string imageFileName);
  void setAxialResolution(const double axialresolution);
  void setHeightResolution(const double heightResolution);
  void setWidthResolution(const double widthResolution);
  void setRawFileName(const std::string rawFileName);
  void setMHDHeader(MHDHeader header);

private :
  usTransducerSettings m_transducerSettings;
  usMotorSettings m_motorSettings;
  double m_axialResolution;
  double m_heightResolution;
  double m_widthResolution;

  MHDHeader header;
  std::map<std::string, int> imageTypeMap;
  std::map<std::string, int> elementTypeMap;
  std::map<int ,std::string> imageTypeReverseMap;
  std::map<int ,std::string> elementTypeReverseMap;
};
#endif //US_META_HEADER_PARSER_H
