#include<visp3/core/vpException.h>
#include <visp3/ustk_io/usMetaHeaderParser.h>
#include <algorithm>
#include <string>

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


void usMetaHeaderParser::readMHDHeader(const std::string fileName)
{
  readMHDHeader(fileName.c_str());
}

void  usMetaHeaderParser::readMHDHeader(const char * fileName)
{
  std::string pathPrefix;
  std::string tmp;

  //splitPathPrefixAndFileName(std::string(fileName), pathPrefix, tmp);

  std::string keyword, keyval;

  this->mhdHeader.numberOfDimensions = 0;
  this->mhdHeader.mhdFileName = fileName;
  this->mhdHeader.rawFileName = "";
  this->mhdHeader.numberOfChannels = 1;
  this->mhdHeader.dim[0] = 1;
  this->mhdHeader.dim[1] = 1;
  this->mhdHeader.dim[2] = 1;
  this->mhdHeader.dim[3] = 1;
  this->mhdHeader.elementSpacing[0] = 1.0f;
  this->mhdHeader.elementSpacing[1] = 1.0f;
  this->mhdHeader.elementSpacing[2] = 1.0f;
  this->mhdHeader.elementSpacing[3] = 1.0f;
  this->mhdHeader.position[0] = 0.0f;
  this->mhdHeader.position[1] = 0.0f;
  this->mhdHeader.position[2] = 0.0f;
  this->mhdHeader.position[3] = 0.0f;
  this->mhdHeader.msb = false;
  this->mhdHeader.headerSize = 0;
  this->mhdHeader.imageType = UNKNOWN;
  this->mhdHeader.isImageConvex = false;
  this->mhdHeader.isMotorConvex = false;
  this->mhdHeader.probeRadius = 0.0f;
  this->mhdHeader.scanLinePitch = 0.0f;
  this->mhdHeader.motorRadius = 0.0f;
  this->mhdHeader.framePitch = 0.0f;

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
      file >> this->mhdHeader.numberOfDimensions;
      std::getline(file, keyval, '\n');
    }
    else if (keyword == "DimSize")
    {
      for (unsigned int i = 0; i < this->mhdHeader.numberOfDimensions; i++)
        file >> this->mhdHeader.dim[i];
      std::getline(file, keyval, '\n');
    }
    else if (keyword == "ElementSpacing")
    {
      for (unsigned int i = 0; i < this->mhdHeader.numberOfDimensions; i++)
        file >> this->mhdHeader.elementSpacing[i];
      std::getline(file, keyval, '\n');
    }
    else if (keyword == "Position")
    {
      for (unsigned int i = 0; i < this->mhdHeader.numberOfDimensions; i++)
        file >> this->mhdHeader.position[i];
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
      this->mhdHeader.msb = ((keyval == "True") || (keyval == "1"));
    }
    else if (keyword == "BinaryDataByteOrderMSB")
    {
      std::getline(file, keyval, '\n');
      it=keyval.end();
      keyval.erase(std::remove(keyval.begin(),keyval.end(),' '),it);
      it=keyval.end();
      keyval.erase(std::remove(keyval.begin(),keyval.end(),'\r'),it);
      this->mhdHeader.msb = ((keyval == "True") || (keyval == "1"));
    }
    else if (keyword == "ElementNumberOfChannels")
    {
      file >> this->mhdHeader.numberOfChannels;
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
      this->mhdHeader.elementType = ((mapIt != elementTypeMap.end()) ? (ElementType)mapIt->second : MET_UNKNOWN);
    }
    else if (keyword == "HeaderSize")
    {
      file >> this->mhdHeader.headerSize;
      std::getline(file, keyval, '\n');
      if ((this->mhdHeader.headerSize) && (this->mhdHeader.headerSize != -1)) {
        std::cout <<   "Warning: " << this->mhdHeader.headerSize << " bytes this->mhdHeader" << std::endl;
      }
    }
    else if (keyword == "ElementDataFile")
    {
      std::getline(file, keyval, '\n');
      it=keyval.end();
      keyval.erase(std::remove(keyval.begin(),keyval.end(),' '),it);
      it=keyval.end();
      keyval.erase(std::remove(keyval.begin(),keyval.end(),'\r'),it);
      this->mhdHeader.rawFileName = keyval;
    }
    else if (keyword == "UltrasoundImageType")
    {
      std::getline(file, keyval, '\n');
      it=keyval.end();
      keyval.erase(std::remove(keyval.begin(),keyval.end(),' '),it);
      it=keyval.end();
      keyval.erase(std::remove(keyval.begin(),keyval.end(),'\r'),it);
      std::map<std::string, int>::iterator mapIt = imageTypeMap.find(keyval);
      this->mhdHeader.imageType = ((mapIt != imageTypeMap.end()) ? (ImageType)mapIt->second : UNKNOWN );
    }
    else if (keyword == "ScanLinePitch")
    {
      file >> this->mhdHeader.scanLinePitch;
      std::getline(file, keyval, '\n');
    }
    else if (keyword == "ProbeRadius")
    {
      file >> this->mhdHeader.probeRadius;
      std::getline(file, keyval, '\n');
    }
    else if (keyword == "IsImageConvex")
    {
      std::getline(file, keyval, '\n');
      it=keyval.end();
      keyval.erase(std::remove(keyval.begin(),keyval.end(),' '),it);
      it=keyval.end();
      keyval.erase(std::remove(keyval.begin(),keyval.end(),'\r'),it);
      this->mhdHeader.isImageConvex = ((keyval == "True") || (keyval == "1"));
    }
    else if (keyword == "FramePitch")
    {
      file >> this->mhdHeader.framePitch;
      std::getline(file, keyval, '\n');
    }
    else if (keyword == "MotorRadius")
    {
      file >> this->mhdHeader.motorRadius;
      std::getline(file, keyval, '\n');
    }
    else if (keyword == "IsMotorConvex")
    {
      std::getline(file, keyval, '\n');
      it=keyval.end();
      keyval.erase(std::remove(keyval.begin(),keyval.end(),' '),it);
      it=keyval.end();
      keyval.erase(std::remove(keyval.begin(),keyval.end(),'\r'),it);
      this->mhdHeader.isMotorConvex = ((keyval == "True") || (keyval == "1"));
    }
    else if (keyword == "AxialResolution")
    {
      if(this->mhdHeader.imageType == POSTSCAN_2D || this->mhdHeader.imageType == POSTSCAN_3D) {
        throw(vpException(vpException::badValue, "bad header file : trying to assign an axial resolution to a postscan image"));
      }
      else {
        file >> this->m_axialResolution;
        std::getline(file, keyval, '\n');
      }
    }
    else if (keyword == "HeightResolution")
    {
      if(this->mhdHeader.imageType == PRESCAN_2D || this->mhdHeader.imageType == PRESCAN_3D || this->mhdHeader.imageType == RF_2D || this->mhdHeader.imageType == RF_3D) {
        throw(vpException(vpException::badValue, "bad header file : trying to assign a height resolution to a prescan image"));
      }
      else {
        file >> this->m_heightResolution;
        std::getline(file, keyval, '\n');
      }
    }
    else if (keyword == "WidthResolution")
    {
      if(this->mhdHeader.imageType == PRESCAN_2D || this->mhdHeader.imageType == PRESCAN_3D || this->mhdHeader.imageType == RF_2D || this->mhdHeader.imageType == RF_3D) {
        throw(vpException(vpException::badValue, "bad header file : trying to assign a width resolution to a prescan image"));
      }
      else {
        file >> this->m_widthResolution;
        std::getline(file, keyval, '\n');
      }
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
}

void usMetaHeaderParser::parse()
{
  // write header
  std::ofstream mhdfile;
  mhdfile.open( mhdHeader.mhdFileName.c_str() );

  mhdfile << "NDims = " << mhdHeader.numberOfDimensions << "\n";;

  if(mhdHeader.numberOfDimensions == 2) {
    char str[100];
    sprintf(str, "DimSize = %d %d \n", mhdHeader.dim[0],mhdHeader.dim[1]);
    mhdfile << str;
    char str2[100];
    sprintf(str2, "ElementSpacing = %f %f \n", mhdHeader.elementSpacing[0], mhdHeader.elementSpacing[1]);
    mhdfile << str2;
  }
  if(mhdHeader.numberOfDimensions == 3) {
    char str[100];
    sprintf(str, "DimSize = %d %d %d \n", mhdHeader.dim[0],mhdHeader.dim[1],mhdHeader.dim[2]);
    mhdfile << str;
    char str2[100];
    sprintf(str2, "ElementSpacing = %f %f %f \n", mhdHeader.elementSpacing[0], mhdHeader.elementSpacing[1],mhdHeader.elementSpacing[2]);
    mhdfile << str2;
  }

  if(mhdHeader.elementType== MET_SHORT)
    mhdfile << "ElementType = " <<  "MET_SHORT" << "\n";
  else if(mhdHeader.elementType == MET_DOUBLE)
    mhdfile << "ElementType = " <<  "MET_DOUBLE" << "\n";
  else if(mhdHeader.elementType == MET_UCHAR)
    mhdfile << "ElementType = " <<  "MET_UCHAR" << "\n";
  else
    mhdfile << "ElementType = " <<  "MET_UNKNOWN" << "\n";

  mhdfile << "ElementByteOrderMSB = " << mhdHeader.msb <<"\n";

  mhdfile << "ElementDataFile = " << mhdHeader.rawFileName << "\n";

  if(mhdHeader.imageType== RF_2D) {
    mhdfile << "UltrasoundImageType = " <<  "RF_2D" << "\n";
    mhdfile << "AxialResolution = " << this->m_axialResolution << "\n";
  }
  else if(mhdHeader.imageType == RF_3D) {
    mhdfile << "UltrasoundImageType = " <<  "RF_3D" << "\n";
    mhdfile << "AxialResolution = " << this->m_axialResolution << "\n";
  }
  else if(mhdHeader.imageType == PRESCAN_2D) {
    mhdfile << "UltrasoundImageType = " <<  "PRESCAN_2D" << "\n";
    mhdfile << "AxialResolution = " << this->m_axialResolution << "\n";
  }
  else if(mhdHeader.imageType == PRESCAN_3D) {
    mhdfile << "UltrasoundImageType = " <<  "PRESCAN_3D" << "\n";
    mhdfile << "AxialResolution = " << this->m_axialResolution << "\n";
  }
  else if(mhdHeader.imageType == POSTSCAN_2D) {
    mhdfile << "UltrasoundImageType = " <<  "POSTSCAN_2D" << "\n";
    mhdfile << "HeightResolution = " << this->m_heightResolution << "\n";
    mhdfile << "WidthResolution = " << this->m_widthResolution << "\n";
  }
  else if(mhdHeader.imageType == POSTSCAN_3D) {
    mhdfile << "UltrasoundImageType = " <<  "POSTSCAN_3D" << "\n";
    mhdfile << "HeightResolution = " << this->m_heightResolution << "\n";
    mhdfile << "WidthResolution = " << this->m_widthResolution << "\n";
  }
  else
    mhdfile << "UltrasoundImageType = " <<  "MET_UNKNOWN" << "\n";

  mhdfile << "IsImageConvex = " << mhdHeader.isImageConvex << "\n";

  mhdfile << "IsMotorConvex = " << mhdHeader.isMotorConvex << "\n";

  mhdfile << "ProbeRadius = " << mhdHeader.probeRadius << "\n";

  mhdfile << "ScanLinePitch = " << mhdHeader.scanLinePitch << "\n";

  mhdfile << "MotorRadius = " << mhdHeader.motorRadius << "\n";

  mhdfile << "FramePitch = " << mhdHeader.framePitch << "\n";

  mhdfile.close();
}

void usMetaHeaderParser::read(const std::string& filename)
{
  readMHDHeader(filename);

  //basic common settings
  this->m_imageSettings.setProbeRadius(mhdHeader.probeRadius);
  this->m_imageSettings.setScanLinePitch(mhdHeader.scanLinePitch);
  this->m_imageSettings.setImageConvex(mhdHeader.isImageConvex);

  if(this->mhdHeader.imageType == RF_3D || this->mhdHeader.imageType == PRESCAN_3D || this->mhdHeader.imageType == POSTSCAN_3D) {
    this->m_imageSettings3D.setProbeRadius(mhdHeader.probeRadius);
    this->m_imageSettings3D.setScanLinePitch(mhdHeader.scanLinePitch);
    this->m_imageSettings3D.setImageConvex(mhdHeader.isImageConvex);
    this->m_imageSettings3D.setMotorRadius(mhdHeader.motorRadius);
    this->m_imageSettings3D.setFramePitch(mhdHeader.framePitch);
    this->m_imageSettings3D.setMotorConvex(mhdHeader.isMotorConvex);
  }
}

//Data setters
void usMetaHeaderParser::setImageSettings(const usImageSettings imageSettings)
{
  m_imageSettings = imageSettings;
}

void usMetaHeaderParser::setImageSettings3D(const usImageSettings3D imageSettings3D)
{
  m_imageSettings3D = imageSettings3D;
}

void usMetaHeaderParser::setRawFileName(const std::string imageFileName)
{
  mhdHeader.rawFileName = imageFileName;
}

void usMetaHeaderParser::setAxialResolution(const double axialresolution)
{
  m_axialResolution = axialresolution;
}

void usMetaHeaderParser::setHeightResolution(const double heightResolution)
{
  m_heightResolution = heightResolution;
}

void usMetaHeaderParser::setWidthResolution(const double widthResolution)
{
  m_widthResolution = widthResolution;
}

void usMetaHeaderParser::setMhdHeader(const MHDHeader header)
{
  mhdHeader = header;
}
