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
#include <visp3/ustk_core/usImagePostScan2D.h>
#include <visp3/ustk_core/usImagePostScan3D.h>
#include <visp3/ustk_core/usImagePreScan2D.h>
#include <visp3/ustk_core/usImagePreScan3D.h>
#include <visp3/ustk_core/usImageRF2D.h>
#include <visp3/ustk_core/usImageRF3D.h>

class VISP_EXPORT usRawFileParser {

public:
  void read(usImagePostScan3D<unsigned char> &postScanImage3D, const std::string mhdFileName);
  void write(const usImagePostScan3D<unsigned char> &postScanImage3D, const std::string rawFileName);

};
#endif //US_RAW_FILE_PARSER_H
