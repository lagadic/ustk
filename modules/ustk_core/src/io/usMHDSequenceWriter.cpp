#include <visp3/ustk_core/usMHDSequenceWriter.h>

/**
* Constructor, initializes the member attribues.
*/
usMHDSequenceWriter::usMHDSequenceWriter() : m_sequenceDirectory(), m_sequenceImageType(us::NOT_SET), m_imageCounter(0)
{
}

/**
* Destructor.
*/
usMHDSequenceWriter::~usMHDSequenceWriter() {}

/**
* Setter for the directory where to write the mhd sequence. To call before calling write !
* @param sequenceDirectory The directory path.
*/
void usMHDSequenceWriter::setSequenceDirectory(const std::string sequenceDirectory)
{

  if (!vpIoTools::checkDirectory(sequenceDirectory))
    throw(vpException(vpException::badValue, "usMHDSequenceWriter : directory name incorrect !"));

  m_sequenceDirectory = sequenceDirectory;
  m_sequenceImageType = us::NOT_SET;
  m_imageCounter = 0;
}

/**
* Writing method for usImageRF2D images in a sequence.
* @param image The usImageRF2D image to write.
* @param timestamp The timestamp of the image.
*/
void usMHDSequenceWriter::write(const usImageRF2D<short int> &image, const uint64_t timestamp)
{

  if (m_sequenceImageType == us::NOT_SET) // first image written
    m_sequenceImageType = us::RF_2D;

  if (m_sequenceImageType != us::RF_2D)
    throw(vpException(vpException::badValue,
                      "usMHDSequenceWriter : trying to write a 2D RF image in a sequence of another type of image !"));

  std::string mhdImageFileName = m_sequenceDirectory + vpIoTools::path("/") + std::string("image%05d.mhd");
  std::string rawImageFileName = m_sequenceDirectory + vpIoTools::path("/") + std::string("image%05d.raw");

  char mhdFileNamebuffer[FILENAME_MAX];
  sprintf(mhdFileNamebuffer, mhdImageFileName.c_str(), m_imageCounter);

  char rawFileNamebuffer[FILENAME_MAX];
  sprintf(rawFileNamebuffer, rawImageFileName.c_str(), m_imageCounter);

  char rawFileNamebufferMin[FILENAME_MAX]; // filename without path
  sprintf(rawFileNamebufferMin, "image%05d.raw", m_imageCounter);

  usMetaHeaderParser::MHDHeader header;
  header.dim[0] = image.getWidth();
  header.dim[1] = image.getHeight();
  header.elementSpacing[0] = 1.0;
  header.elementSpacing[1] = 1.0;
  header.elementType = usMetaHeaderParser::MET_SHORT;
  header.imageType = us::RF_2D;
  header.isTransducerConvex = image.isTransducerConvex();
  header.MHDFileName = std::string(mhdFileNamebuffer);
  header.numberOfDimensions = 2;
  header.rawFileName = std::string(rawFileNamebufferMin);
  header.samplingFrequency = image.getSamplingFrequency();
  header.scanLineNumber = image.getScanLineNumber();
  header.scanLinePitch = image.getScanLinePitch();
  header.timestamp.push_back(timestamp);
  header.transducerRadius = image.getTransducerRadius();
  header.transmitFrequency = image.getTransmitFrequency();

  usMetaHeaderParser mhdParser;
  mhdParser.setMHDHeader(header);
  mhdParser.setAxialResolution(image.getAxialResolution());
  mhdParser.parse();

  usRawFileParser rawParser;
  rawParser.write(image, std::string(rawFileNamebuffer));

  m_imageCounter++;
}

/**
* Writing method for usImagePreScan2D images in a sequence.
* @param image The usImagePreScan2D image to write.
* @param timestamp The timestamp of the image.
*/
void usMHDSequenceWriter::write(const usImagePreScan2D<unsigned char> &image, const uint64_t timestamp)
{

  if (m_sequenceImageType == us::NOT_SET) // first image written
    m_sequenceImageType = us::PRESCAN_2D;

  if (m_sequenceImageType != us::PRESCAN_2D)
    throw(vpException(
        vpException::badValue,
        "usMHDSequenceWriter : trying to write a 2D pre-scan image in a sequence of another type of image !"));

  std::string mhdImageFileName = m_sequenceDirectory + vpIoTools::path("/") + std::string("image%05d.mhd");
  std::string rawImageFileName = m_sequenceDirectory + vpIoTools::path("/") + std::string("image%05d.raw");

  char mhdFileNamebuffer[FILENAME_MAX];
  sprintf(mhdFileNamebuffer, mhdImageFileName.c_str(), m_imageCounter);

  char rawFileNamebuffer[FILENAME_MAX];
  sprintf(rawFileNamebuffer, rawImageFileName.c_str(), m_imageCounter);

  char rawFileNamebufferMin[FILENAME_MAX]; // filename without path
  sprintf(rawFileNamebufferMin, "image%05d.raw", m_imageCounter);

  usMetaHeaderParser::MHDHeader header;
  header.dim[0] = image.getWidth();
  header.dim[1] = image.getHeight();
  header.elementSpacing[0] = 1.0;
  header.elementSpacing[1] = 1.0;
  header.elementType = usMetaHeaderParser::MET_UCHAR;
  header.imageType = us::PRESCAN_2D;
  header.isTransducerConvex = image.isTransducerConvex();
  header.MHDFileName = std::string(mhdFileNamebuffer);
  header.numberOfDimensions = 2;
  header.rawFileName = std::string(rawFileNamebufferMin);
  header.samplingFrequency = image.getSamplingFrequency();
  header.scanLineNumber = image.getScanLineNumber();
  header.scanLinePitch = image.getScanLinePitch();
  header.timestamp.push_back(timestamp);
  header.transducerRadius = image.getTransducerRadius();
  header.transmitFrequency = image.getTransmitFrequency();

  usMetaHeaderParser mhdParser;
  mhdParser.setMHDHeader(header);
  mhdParser.setAxialResolution(image.getAxialResolution());
  mhdParser.parse();

  usRawFileParser rawParser;
  rawParser.write(image, std::string(rawFileNamebuffer));

  m_imageCounter++;
}

/**
* Writing method for usImagePostScan2D images in a sequence.
* @param image The usImagePostScan2D image to write.
* @param timestamp The timestamp of the image.
*/
void usMHDSequenceWriter::write(const usImagePostScan2D<unsigned char> &image, const uint64_t timestamp)
{

  if (m_sequenceImageType == us::NOT_SET) // first image written
    m_sequenceImageType = us::POSTSCAN_2D;

  if (m_sequenceImageType != us::POSTSCAN_2D)
    throw(vpException(vpException::badValue,
                      "usMHDSequenceWriter : trying to write a 2D RF image in a sequence of another type of image !"));

  std::string mhdImageFileName = m_sequenceDirectory + vpIoTools::path("/") + std::string("image%05d.mhd");
  std::string rawImageFileName = m_sequenceDirectory + vpIoTools::path("/") + std::string("image%05d.raw");

  char mhdFileNamebuffer[FILENAME_MAX];
  sprintf(mhdFileNamebuffer, mhdImageFileName.c_str(), m_imageCounter);

  char rawFileNamebuffer[FILENAME_MAX];
  sprintf(rawFileNamebuffer, rawImageFileName.c_str(), m_imageCounter);

  char rawFileNamebufferMin[FILENAME_MAX]; // filename without path
  sprintf(rawFileNamebufferMin, "image%05d.raw", m_imageCounter);

  usMetaHeaderParser::MHDHeader header;
  header.dim[0] = image.getWidth();
  header.dim[1] = image.getHeight();
  header.elementSpacing[0] = image.getWidthResolution();
  header.elementSpacing[1] = image.getHeightResolution();
  ;
  header.elementType = usMetaHeaderParser::MET_UCHAR;
  header.imageType = us::POSTSCAN_2D;
  header.isTransducerConvex = image.isTransducerConvex();
  header.MHDFileName = std::string(mhdFileNamebuffer);
  header.numberOfDimensions = 2;
  header.rawFileName = std::string(rawFileNamebufferMin);
  header.samplingFrequency = image.getSamplingFrequency();
  header.scanLineNumber = image.getScanLineNumber();
  header.scanLinePitch = image.getScanLinePitch();
  header.timestamp.push_back(timestamp);
  header.transducerRadius = image.getTransducerRadius();
  header.transmitFrequency = image.getTransmitFrequency();

  usMetaHeaderParser mhdParser;
  mhdParser.setMHDHeader(header);
  mhdParser.setHeightResolution(image.getHeightResolution());
  mhdParser.setWidthResolution(image.getWidthResolution());
  mhdParser.parse();

  usRawFileParser rawParser;
  rawParser.write(image, std::string(rawFileNamebuffer));

  m_imageCounter++;
}

/**
* Writing method for usImageRF3D images in a sequence.
* @param image The usImageRF3D image to write.
* @param timestamp The timestamps of every frame of the volume  (inverted in case of odd volume in sequence !).
*/
void usMHDSequenceWriter::write(const usImageRF3D<short int> &image, const std::vector<uint64_t> timestamp)
{

  if (m_sequenceImageType == us::NOT_SET) // first image written
    m_sequenceImageType = us::RF_3D;

  if (m_sequenceImageType != us::RF_3D)
    throw(vpException(vpException::badValue,
                      "usMHDSequenceWriter : trying to write a 3D RF image in a sequence of another type of image !"));

  std::string mhdImageFileName = m_sequenceDirectory + vpIoTools::path("/") + std::string("image%05d.mhd");
  std::string rawImageFileName = m_sequenceDirectory + vpIoTools::path("/") + std::string("image%05d.raw");

  char mhdFileNamebuffer[FILENAME_MAX];
  sprintf(mhdFileNamebuffer, mhdImageFileName.c_str(), m_imageCounter);

  char rawFileNamebuffer[FILENAME_MAX];
  sprintf(rawFileNamebuffer, rawImageFileName.c_str(), m_imageCounter);

  char rawFileNamebufferMin[FILENAME_MAX]; // filename without path
  sprintf(rawFileNamebufferMin, "image%05d.raw", m_imageCounter);

  usMetaHeaderParser::MHDHeader header;
  header.dim[0] = image.getWidth();
  header.dim[1] = image.getHeight();
  header.dim[2] = image.getNumberOfFrames();
  header.elementSpacing[0] = 1.0;
  header.elementSpacing[1] = 1.0;
  header.elementSpacing[2] = 1.0;
  header.elementType = usMetaHeaderParser::MET_SHORT;
  header.frameNumber = image.getFrameNumber();
  header.framePitch = image.getFramePitch();
  header.imageType = us::RF_3D;
  header.isTransducerConvex = image.isTransducerConvex();
  header.MHDFileName = std::string(mhdFileNamebuffer);
  header.motorRadius = image.getMotorRadius();
  header.motorType = image.getMotorType();
  header.numberOfDimensions = 3;
  header.rawFileName = std::string(rawFileNamebufferMin);
  header.samplingFrequency = image.getSamplingFrequency();
  header.scanLineNumber = image.getScanLineNumber();
  header.scanLinePitch = image.getScanLinePitch();
  header.timestamp = timestamp;
  header.transducerRadius = image.getTransducerRadius();
  header.transmitFrequency = image.getTransmitFrequency();

  usMetaHeaderParser mhdParser;
  mhdParser.setMHDHeader(header);
  mhdParser.setAxialResolution(image.getAxialResolution());
  mhdParser.parse();

  usRawFileParser rawParser;
  rawParser.write(image, std::string(rawFileNamebuffer));

  m_imageCounter++;
}

/**
* Writing method for usImagePreScan3D images in a sequence.
* @param image The usImagePreScan3D image to write.
* @param timestamp The timestamps of every frame of the volume (inverted in case of odd volume in sequence !).
*/
void usMHDSequenceWriter::write(const usImagePreScan3D<unsigned char> &image, const std::vector<uint64_t> timestamp)
{

  if (m_sequenceImageType == us::NOT_SET) // first image written
    m_sequenceImageType = us::PRESCAN_3D;

  if (m_sequenceImageType != us::PRESCAN_3D)
    throw(vpException(
        vpException::badValue,
        "usMHDSequenceWriter : trying to write a 3D pre-scan image in a sequence of another type of image !"));

  std::string mhdImageFileName = m_sequenceDirectory + vpIoTools::path("/") + std::string("image%05d.mhd");
  std::string rawImageFileName = m_sequenceDirectory + vpIoTools::path("/") + std::string("image%05d.raw");

  char mhdFileNamebuffer[FILENAME_MAX];
  sprintf(mhdFileNamebuffer, mhdImageFileName.c_str(), m_imageCounter);

  char rawFileNamebuffer[FILENAME_MAX];
  sprintf(rawFileNamebuffer, rawImageFileName.c_str(), m_imageCounter);

  char rawFileNamebufferMin[FILENAME_MAX]; // filename without path
  sprintf(rawFileNamebufferMin, "image%05d.raw", m_imageCounter);

  usMetaHeaderParser::MHDHeader header;
  header.dim[0] = image.getWidth();
  header.dim[1] = image.getHeight();
  header.dim[2] = image.getNumberOfFrames();
  header.elementSpacing[0] = 1.0;
  header.elementSpacing[1] = 1.0;
  header.elementSpacing[2] = 1.0;
  header.elementType = usMetaHeaderParser::MET_UCHAR;
  header.frameNumber = image.getFrameNumber();
  header.framePitch = image.getFramePitch();
  header.imageType = us::PRESCAN_3D;
  header.isTransducerConvex = image.isTransducerConvex();
  header.MHDFileName = std::string(mhdFileNamebuffer);
  header.motorRadius = image.getMotorRadius();
  header.motorType = image.getMotorType();
  header.numberOfDimensions = 3;
  header.rawFileName = std::string(rawFileNamebufferMin);
  header.samplingFrequency = image.getSamplingFrequency();
  header.scanLineNumber = image.getScanLineNumber();
  header.scanLinePitch = image.getScanLinePitch();
  header.timestamp = timestamp;
  header.transducerRadius = image.getTransducerRadius();
  header.transmitFrequency = image.getTransmitFrequency();

  usMetaHeaderParser mhdParser;
  mhdParser.setMHDHeader(header);
  mhdParser.setAxialResolution(image.getAxialResolution());
  mhdParser.parse();

  usRawFileParser rawParser;
  rawParser.write(image, std::string(rawFileNamebuffer));

  m_imageCounter++;
}

/**
* Writing method for usImagePostScan3D images in a sequence.
* @param image The usImagePostScan3D image to write.
* @param timestamp The timestamp of the volume.
*/
void usMHDSequenceWriter::write(const usImagePostScan3D<unsigned char> &image, const uint64_t timestamp)
{

  if (m_sequenceImageType == us::NOT_SET) // first image written
    m_sequenceImageType = us::POSTSCAN_3D;

  if (m_sequenceImageType != us::POSTSCAN_3D)
    throw(vpException(
        vpException::badValue,
        "usMHDSequenceWriter : trying to write a 3D post-scan image in a sequence of another type of image !"));

  std::string mhdImageFileName = m_sequenceDirectory + vpIoTools::path("/") + std::string("image%05d.mhd");
  std::string rawImageFileName = m_sequenceDirectory + vpIoTools::path("/") + std::string("image%05d.raw");

  char mhdFileNamebuffer[FILENAME_MAX];
  sprintf(mhdFileNamebuffer, mhdImageFileName.c_str(), m_imageCounter);

  char rawFileNamebuffer[FILENAME_MAX];
  sprintf(rawFileNamebuffer, rawImageFileName.c_str(), m_imageCounter);

  char rawFileNamebufferMin[FILENAME_MAX]; // filename without path
  sprintf(rawFileNamebufferMin, "image%05d.raw", m_imageCounter);

  usMetaHeaderParser::MHDHeader header;
  header.dim[0] = image.getWidth();
  header.dim[1] = image.getHeight();
  header.dim[2] = image.getNumberOfFrames();
  header.elementSpacing[0] = 1.0;
  header.elementSpacing[1] = 1.0;
  header.elementSpacing[2] = 1.0;
  header.elementType = usMetaHeaderParser::MET_UCHAR;
  header.frameNumber = image.getFrameNumber();
  header.framePitch = image.getFramePitch();
  header.imageType = us::POSTSCAN_3D;
  header.isTransducerConvex = image.isTransducerConvex();
  header.MHDFileName = std::string(mhdFileNamebuffer);
  header.motorRadius = image.getMotorRadius();
  header.motorType = image.getMotorType();
  header.numberOfDimensions = 3;
  header.rawFileName = std::string(rawFileNamebufferMin);
  header.samplingFrequency = image.getSamplingFrequency();
  header.scanLineNumber = image.getScanLineNumber();
  header.scanLinePitch = image.getScanLinePitch();
  header.timestamp.push_back(timestamp);
  header.transducerRadius = image.getTransducerRadius();
  header.transmitFrequency = image.getTransmitFrequency();

  usMetaHeaderParser mhdParser;
  mhdParser.setMHDHeader(header);
  mhdParser.parse();

  usRawFileParser rawParser;
  rawParser.write(image, std::string(rawFileNamebuffer));

  m_imageCounter++;
}
