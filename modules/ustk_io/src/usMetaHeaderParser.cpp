#include<visp3/core/vpException.h>
#include <visp3/ustk_io/usMetaHeaderParser.h>
#include <algorithm>


usMetaHeaderParser::usMetaHeaderParser()
{
  elementTypeMap["MET_UCHAR"] = MET_UCHAR;
  elementTypeMap["MET_SHORT"] = MET_SHORT;
  elementTypeMap["MET_DOUBLE"] = MET_DOUBLE;

  imageTypeMap["RF_2D"] = RF_2D;
  imageTypeMap["RF_3D"] = RF_3D;
  imageTypeMap["PRESCAN_2D"] = PRESCAN_2D;
  imageTypeMap["PRESCAN_3D"] = PRESCAN_3D;
  imageTypeMap["POSTSCAN_2D"] = POSTSCAN_2D;
  imageTypeMap["POSTSCAN_3D"] = POSTSCAN_3D;
}

usMetaHeaderParser::~usMetaHeaderParser()
{

}


usMetaHeaderParser::MHDHeader  usMetaHeaderParser::readMHDHeader(const char * fileName)
{
  std::string pathPrefix;
  std::string tmp;

  //splitPathPrefixAndFileName(std::string(fileName), pathPrefix, tmp);

  std::string keyword, keyval;

  MHDHeader header;
  header.numberOfDimensions = 0;
  header.mhdFileName = fileName;
  header.rawFileName = "";
  header.numberOfChannels = 1;
  header.dim[0] = 1;
  header.dim[1] = 1;
  header.dim[2] = 1;
  header.dim[3] = 1;
  header.elementSpacing[0] = 1.0f;
  header.elementSpacing[1] = 1.0f;
  header.elementSpacing[2] = 1.0f;
  header.elementSpacing[3] = 1.0f;
  header.position[0] = 0.0f;
  header.position[1] = 0.0f;
  header.position[2] = 0.0f;
  header.position[3] = 0.0f;
  header.msb = false;
  header.headerSize = 0;

  std::ifstream file;
  file.open(fileName, std::ifstream::in);
  if (!file.good()) {
    throw vpException(vpException::fatalError, std::string("Error opening .mhd file."));
  }
  std::cout << "mhd file opened !" << std::endl;
  std::string::iterator it;
  while (file.good())
  {
    std::getline(file, keyword, '=');
    it=keyword.end();
    keyword.erase(std::remove(keyword.begin(),keyword.end(),' '),it);
    if (keyword == "NDims")
    {
      file >> header.numberOfDimensions;
      std::getline(file, keyval, '\n');
    }
    else if (keyword == "DimSize")
    {
      for (unsigned int i = 0; i < header.numberOfDimensions; i++)
        file >> header.dim[i];
      std::getline(file, keyval, '\n');
    }
    else if (keyword == "ElementSpacing")
    {
      for (unsigned int i = 0; i < header.numberOfDimensions; i++)
        file >> header.elementSpacing[i];
      std::getline(file, keyval, '\n');
    }
    else if (keyword == "Position")
    {
      for (unsigned int i = 0; i < header.numberOfDimensions; i++)
        file >> header.position[i];
      std::getline(file, keyval, '\n');
    }
    else if (keyword == "ImagePositionPatient")
    {
      std::getline(file, keyval, '\n');
    }
    else if (keyword == "ElementByteOrderMSB")
    {
      std::getline(file, keyval, '\n');
      it=keyval.end();
      keyval.erase(std::remove(keyval.begin(),keyval.end(),' '),it);
      it=keyval.end();
      keyval.erase(std::remove(keyval.begin(),keyval.end(),'\r'),it);
      header.msb = ((keyval == "True") || (keyval == "1"));
    }
    else if (keyword == "BinaryDataByteOrderMSB")
    {
      std::getline(file, keyval, '\n');
      it=keyval.end();
      keyval.erase(std::remove(keyval.begin(),keyval.end(),' '),it);
      it=keyval.end();
      keyval.erase(std::remove(keyval.begin(),keyval.end(),'\r'),it);
      header.msb = ((keyval == "True") || (keyval == "1"));
    }
    else if (keyword == "ElementNumberOfChannels")
    {
      file >> header.numberOfChannels;
      std::getline(file, keyval, '\n');
    }
    else if (keyword == "ElementType")
    {
      std::getline(file, keyval, '\n');
      it=keyval.end();
      keyval.erase(std::remove(keyval.begin(),keyval.end(),' '),it);
      it=keyval.end();
      keyval.erase(std::remove(keyval.begin(), keyval.end(), '\r'), it);
      std::map<std::string, int>::iterator mapIt = elementTypeMap.find(keyval);
      header.elementType = ((mapIt != elementTypeMap.end()) ? (ElementType)mapIt->second : MET_UNKNOWN);
    }
    else if (keyword == "HeaderSize")
    {
      file >> header.headerSize;
      std::getline(file, keyval, '\n');
      if ((header.headerSize) && (header.headerSize != -1)) {
        std::cout <<   "Warning: " << header.headerSize << " bytes header" << std::endl;
      }
    }
    else if (keyword == "ElementDataFile")
    {
      std::getline(file, keyval, '\n');
      it=keyval.end();
      keyval.erase(std::remove(keyval.begin(),keyval.end(),' '),it);
      it=keyval.end();
      keyval.erase(std::remove(keyval.begin(),keyval.end(),'\r'),it);
      header.rawFileName = keyval;
    }
    else if (keyword == "UltrasoundImageType")
    {
      std::getline(file, keyval, '\n');
      it=keyval.end();
      keyval.erase(std::remove(keyval.begin(),keyval.end(),' '),it);
      it=keyval.end();
      keyval.erase(std::remove(keyval.begin(),keyval.end(),'\r'),it);
      std::map<std::string, int>::iterator mapIt = imageTypeMap.find(keyval);
      header.imageType = ((mapIt != imageTypeMap.end()) ? (ImageType)mapIt->second : UNKNOWN );
    }
    else
    {
      if (keyword != "")
      {
        std::getline(file, keyval, '\n');
      }
    }
  }

  file.close();

  return header;
}
