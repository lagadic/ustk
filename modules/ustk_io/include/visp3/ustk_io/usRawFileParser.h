#ifndef US_RAW_FILE_PARSER_H
#define US_RAW_FILE_PARSER_H

#include<string>
#include<map>
#include <cstdlib>
#include <string>
#include <ios>
#include <iostream>
#include <fstream>
#include <sstream>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/ustk_core/usImage3D.h>

/**
 * @class usRawFileParser
 * @brief Raw data parser.
 * @ingroup module_ustk_io
 */
class VISP_EXPORT usRawFileParser {

public:
  void read(usImage3D<unsigned char> &image3D, const std::string mhdFileName);
  void write(const usImage3D<unsigned char> &image3D, const std::string rawFileName);
  void read(vpImage<unsigned char> &image2D, const std::string mhdFileName);
  void write(const vpImage<unsigned char> &image2D, const std::string rawFileName);

};
#endif //US_RAW_FILE_PARSER_H
