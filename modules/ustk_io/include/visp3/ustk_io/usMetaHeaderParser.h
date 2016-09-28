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

  //comparaison
  bool operator ==(usImageSettingsXmlParser const& other);

  //Read/write operations
  MHDHeader readMHDHeader(const char * fileName);

  void parse(const std::string& filename);

  void read(const std::string& filename);


  // Data accessors.
  usImageSettings getImageSettings() const {return m_imageSettings;}
  usImageSettings3D getImageSettings() const {return m_imageSettings3D;}
  std::string getImageFileName() const {return m_imageFileName;}
  float getAxialResolution() const { return m_axialResolution; }
  float getHeightResolution() const { return m_heightResolution; }
  float getWidthResolution() const { return m_widthResolution; }
  bool isImagePreScan() const { return m_is_prescan; }

  //Data setters
  void setImagePreScanSettings(usImagePreScan2D<unsigned char> imagePrescan2D);
  void setImagePostScanSettings(usImagePostScan2D imagePostcan2D);
  void setImageSettings(float probeRadius, float scanLinePitch, bool isImageConvex, float axialResolution);
  void setImageSettings(float probeRadius, float scanLinePitch, bool isImageConvex, float widthResolution, float heightResolution);
  void setImageSettings3D(float probeRadius, float scanLinePitch, bool isImageConvex, float framePitch, float motorRadiurs, bool isMotorConvex, float axialResolution);
  void setImageSettings3D(float probeRadius, float scanLinePitch, bool isImageConvex, float framePitch, float motorRadiurs, bool isMotorConvex, float widthResolution, float heightResolution);
  void setImageFileName(std::string imageFileName);
  void setImagePreScan(bool is_prescan) { m_is_prescan = is_prescan; }
/*  usImageSettings getImageSettings(MHDHeader mhdHeader);
  usImageSettings3D getImageSettings3D(MHDHeader mhdHeader);

  usImagePostScan2D getImagePostScan2D();
  usImagePostScan3D getImagePostScan3D();
  usImagePreScan2D getImagePreScan2D();
  usImagePreScan3D getImagePreScan3D();
  usImageRF2D getImageRF2D();
  usImageRF3D getImageRF3D();*/

private :
  bool m_is_prescan;
  float m_axialResolution;
  float m_heightResolution;
  float m_widthResolution;


  MHDHeader mhdHeader;
  std::map<std::string, int> imageTypeMap;
  std::map<std::string, int> elementTypeMap;
};
#endif //US_META_HEADER_PARSER_H
