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
    unsigned int dim[4];
    double elementSpacing[4];
    double position[4];
    int headerSize;
    bool msb;
    ImageType imageType;
    bool isImageConvex;
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

  //comparaison
  bool operator ==(usMetaHeaderParser const& other);

  //Read/write operations
  MHDHeader readMHDHeader(const char * fileName);

  void readMHDHeader(const std::string fileName);

  void parse(const std::string& filename);

  void read(const std::string& filename);


  // Data accessors.
  usImageSettings getImageSettings() const {return m_imageSettings;}
  usImageSettings3D getImageSettings3D() const {return m_imageSettings3D;}
  std::string getImageFileName() const {return m_imageFileName;}
  double getAxialResolution() const { return m_axialResolution; }
  double getHeightResolution() const { return m_heightResolution; }
  double getWidthResolution() const { return m_widthResolution; }

  //Data setters
  void setImageSettings(const usImageSettings imageSettings);
  void setImageSettings3D(const usImageSettings3D imageSettings3D);
  void setImageFileName(const std::string imageFileName);
  void setAxialResolution(const double axialresolution);
  void setHeightResolution(const double heightResolution);
  void setWidthResolution(const double widthResolution);

private :
  usImageSettings m_imageSettings;
  usImageSettings3D m_imageSettings3D;
  std::string m_imageFileName;

  double m_axialResolution;
  double m_heightResolution;
  double m_widthResolution;

  MHDHeader mhdHeader;
  std::map<std::string, int> imageTypeMap;
  std::map<std::string, int> elementTypeMap;
};
#endif //US_META_HEADER_PARSER_H
