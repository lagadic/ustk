#include <visp3/ustk_io/usMHDSequenceReader.h>

/**
* Constructor, initializes the member attribues.
*/
usMHDSequenceReader::usMHDSequenceReader() : m_sequenceDirectory(), m_sequenceImageType(us::NOT_SET),
  m_sequenceFiles(), m_totalImageNumber(0), m_imageCounter(0)
{

}

/**
* Destructor.
*/
usMHDSequenceReader::~usMHDSequenceReader() {

}

/**
* Setter for the directory containing the mhd sequence to read. To call before calling acquire !
* @param sequenceDirectory The directory path.
*/
void usMHDSequenceReader::setSequenceDirectory(const std::string sequenceDirectory) {
  m_sequenceFiles = vpIoTools::getDirFiles(sequenceDirectory);
  m_sequenceDirectory = sequenceDirectory;
  m_totalImageNumber = m_sequenceFiles.size() / 2; // we have mhd and raw in the directory (2 * m_totalImageNumber)
  m_imageCounter = 0;
}

/**
* Acquisition method for usImageRF2D : fills the output image with the next image in the sequence.
* @param [out] image The usImageRF2D image acquired.
* @param [out] timestamp The timestamp of the image (0 if not present in the sequence parameters).
*/
void usMHDSequenceReader::acquire(usImageRF2D<short int> & image, uint64_t & timestamp) {

  if(m_imageCounter > m_totalImageNumber)
    throw(vpException(vpException::fatalError, "usMHDSequenceReader : end of sequence reached !"));

  if(usImageIo::getHeaderFormat(m_sequenceFiles.at(2*m_imageCounter)) != usImageIo::FORMAT_MHD) //check file extention
    throw(vpException(vpException::fatalError, "usMHDSequenceReader trying to open a non-mhd file !"));

  usMetaHeaderParser mhdParser;
  mhdParser.read(m_sequenceDirectory + vpIoTools::path("/") + m_sequenceFiles.at(2*m_imageCounter)); // we skip raw files
  m_sequenceImageType = mhdParser.getImageType();
  if (m_sequenceImageType != us::RF_2D && m_sequenceImageType != us::NOT_SET) {
    throw(vpException(vpException::badValue, "Reading a non rf 2D image!"));
  }
  if (mhdParser.getElementType() != usMetaHeaderParser::MET_SHORT) {
    throw(vpException(vpException::badValue, "Reading a non short image!"));
  }

  usMetaHeaderParser::MHDHeader mhdHeader = mhdParser.getMHDHeader();
  timestamp = mhdHeader.timestamp.at(0);

  usImagePreScanSettings settings;
  settings.setTransducerRadius(mhdHeader.transducerRadius);
  settings.setScanLinePitch(mhdHeader.scanLinePitch);
  settings.setTransducerConvexity(mhdHeader.isTransducerConvex);
  settings.setAxialResolution(mhdParser.getAxialResolution());
  settings.setDepth(settings.getAxialResolution()*mhdHeader.dim[1]);
  settings.setSamplingFrequency(mhdHeader.samplingFrequency);
  settings.setTransmitFrequency(mhdHeader.transmitFrequency);
  image.setImagePreScanSettings(settings);

  //resizing image in memory
  image.resize(mhdHeader.dim[1], mhdHeader.dim[0]);

  //data parsing
  usRawFileParser rawParser;
  rawParser.read(image, mhdParser.getRawFileName());

  m_imageCounter ++;
}

/**
* Acquisition method for usImagePreScan2D : fills the output image with the next image in the sequence.
* @param [out] image The usImagePreScan2D image acquired.
* @param [out] timestamp The timestamp of the image (0 if not present in the sequence parameters).
*/
void usMHDSequenceReader::acquire(usImagePreScan2D<unsigned char> & image, uint64_t & timestamp) {

  if(m_imageCounter > m_totalImageNumber)
    throw(vpException(vpException::fatalError, "usMHDSequenceReader : end of sequence reached !"));

  if(usImageIo::getHeaderFormat(m_sequenceFiles.at(2*m_imageCounter)) != usImageIo::FORMAT_MHD) //check file extention
    throw(vpException(vpException::fatalError, "usMHDSequenceReader trying to open a non-mhd file !"));

  usMetaHeaderParser mhdParser;
  mhdParser.read(m_sequenceDirectory + vpIoTools::path("/") + m_sequenceFiles.at(2*m_imageCounter)); // we skip raw files
  m_sequenceImageType = mhdParser.getImageType();
  if (m_sequenceImageType != us::PRESCAN_2D && m_sequenceImageType != us::NOT_SET) {
    throw(vpException(vpException::badValue, "Reading a non pre-scan 2D image!"));
  }
  if (mhdParser.getElementType() != usMetaHeaderParser::MET_UCHAR) {
    throw(vpException(vpException::badValue, "Reading a non unsigned char image!"));
  }

  usMetaHeaderParser::MHDHeader mhdHeader = mhdParser.getMHDHeader();
  timestamp = mhdHeader.timestamp.at(0);

  usImagePreScanSettings settings;
  settings.setTransducerRadius(mhdHeader.transducerRadius);
  settings.setScanLinePitch(mhdHeader.scanLinePitch);
  settings.setTransducerConvexity(mhdHeader.isTransducerConvex);
  settings.setAxialResolution(mhdParser.getAxialResolution());
  settings.setDepth(settings.getAxialResolution()*mhdHeader.dim[1]);
  settings.setSamplingFrequency(mhdHeader.samplingFrequency);
  settings.setTransmitFrequency(mhdHeader.transmitFrequency);
  image.setImagePreScanSettings(settings);

  //resizing image in memory
  image.resize(mhdHeader.dim[1], mhdHeader.dim[0]);

  //data parsing
  usRawFileParser rawParser;
  rawParser.read(image, mhdParser.getRawFileName());

  m_imageCounter ++;
}

/**
* Acquisition method for usImagePostScan2D : fills the output image with the next image in the sequence.
* @param [out] image The usImagePostScan2D image acquired.
* @param [out] timestamp The timestamp of the image (0 if not present in the sequence parameters).
*/
void usMHDSequenceReader::acquire(usImagePostScan2D<unsigned char> & image, uint64_t & timestamp) {

  if(m_imageCounter > m_totalImageNumber)
    throw(vpException(vpException::fatalError, "usMHDSequenceReader : end of sequence reached !"));

  if(usImageIo::getHeaderFormat(m_sequenceFiles.at(2*m_imageCounter)) != usImageIo::FORMAT_MHD) //check file extention
    throw(vpException(vpException::fatalError, "usMHDSequenceReader trying to open a non-mhd file !"));

  usMetaHeaderParser mhdParser;
  mhdParser.read(m_sequenceDirectory + vpIoTools::path("/") + m_sequenceFiles.at(2*m_imageCounter)); // we skip raw files
  m_sequenceImageType = mhdParser.getImageType();
  if (m_sequenceImageType != us::POSTSCAN_2D && m_sequenceImageType != us::NOT_SET) {
    throw(vpException(vpException::badValue, "Reading a non p-scan 2D image!"));
  }
  if (mhdParser.getElementType() != usMetaHeaderParser::MET_UCHAR) {
    throw(vpException(vpException::badValue, "Reading a non unsigned char image!"));
  }

  usMetaHeaderParser::MHDHeader mhdHeader = mhdParser.getMHDHeader();
  timestamp = mhdHeader.timestamp.at(0);

  usImagePreScanSettings settings;
  image.setTransducerRadius(mhdHeader.transducerRadius);
  image.setScanLinePitch(mhdHeader.scanLinePitch);
  image.setTransducerConvexity(mhdHeader.isTransducerConvex);;
  image.setDepth(settings.getAxialResolution()*mhdHeader.dim[1]);
  image.setWidthResolution(mhdHeader.dim[0]);
  image.setWidthResolution(mhdHeader.dim[1]);
  image.setSamplingFrequency(mhdHeader.samplingFrequency);
  image.setTransmitFrequency(mhdHeader.transmitFrequency);

  //resizing image in memory
  image.resize(mhdHeader.dim[1], mhdHeader.dim[0]);

  //data parsing
  usRawFileParser rawParser;
  rawParser.read(image, mhdParser.getRawFileName());

  m_imageCounter ++;
}

/**
* Acquisition method for usImageRF3D : fills the output image with the next volume in the sequence.
* @param [out] image The usImageRF3D image acquired.
* @param [out] timestamp The timestamps of the image (0 if not present in the sequence parameters). Every frame of the volume contains an associated timesamp.
*/
void usMHDSequenceReader::acquire(usImageRF3D<short int> & image, std::vector<uint64_t> & timestamp) {

  if(m_imageCounter > m_totalImageNumber)
    throw(vpException(vpException::fatalError, "usMHDSequenceReader : end of sequence reached !"));

  if(usImageIo::getHeaderFormat(m_sequenceFiles.at(2*m_imageCounter)) != usImageIo::FORMAT_MHD) //check file extention
    throw(vpException(vpException::fatalError, "usMHDSequenceReader trying to open a non-mhd file !"));

  usMetaHeaderParser mhdParser;
  mhdParser.read(m_sequenceDirectory + vpIoTools::path("/") + m_sequenceFiles.at(2*m_imageCounter)); // we skip raw files
  m_sequenceImageType = mhdParser.getImageType();
  if (m_sequenceImageType != us::RF_3D && m_sequenceImageType != us::NOT_SET) {
    throw(vpException(vpException::badValue, "Reading a non rf 3D image!"));
  }
  if (mhdParser.getElementType() != usMetaHeaderParser::MET_SHORT) {
    throw(vpException(vpException::badValue, "Reading a non short image!"));
  }

  usMetaHeaderParser::MHDHeader mhdHeader = mhdParser.getMHDHeader();
  timestamp = mhdHeader.timestamp;

  usImagePreScanSettings settings;
  settings.setTransducerRadius(mhdHeader.transducerRadius);
  settings.setScanLinePitch(mhdHeader.scanLinePitch);
  settings.setTransducerConvexity(mhdHeader.isTransducerConvex);
  settings.setAxialResolution(mhdParser.getAxialResolution());
  settings.setDepth(settings.getAxialResolution()*mhdHeader.dim[1]);
  settings.setSamplingFrequency(mhdHeader.samplingFrequency);
  settings.setTransmitFrequency(mhdHeader.transmitFrequency);
  image.setImagePreScanSettings(settings);

  usMotorSettings motorSettings;
  motorSettings.setMotorRadius(mhdHeader.motorRadius);
  motorSettings.setFramePitch(mhdHeader.framePitch);
  motorSettings.setMotorType(mhdHeader.motorType);
  motorSettings.setFrameNumber(mhdHeader.dim[2]);
  image.setMotorSettings(motorSettings);

  //resizing image in memory
  image.resize(mhdHeader.dim[0], mhdHeader.dim[1],mhdHeader.dim[2]);

  //data parsing
  usRawFileParser rawParser;
  rawParser.read(image, mhdParser.getRawFileName());

  m_imageCounter ++;
}

/**
* Acquisition method for usImagePreScan3D : fills the output image with the next volume in the sequence.
* @param [out] image The usImagePreScan3D image acquired.
* @param [out] timestamp The timestamps of the image (0 if not present in the sequence parameters). Every frame of the volume contains an associated timesamp.
*/
void usMHDSequenceReader::acquire(usImagePreScan3D<unsigned char> & image, std::vector<uint64_t> & timestamp) {

  if(m_imageCounter > m_totalImageNumber)
    throw(vpException(vpException::fatalError, "usMHDSequenceReader : end of sequence reached !"));

  if(usImageIo::getHeaderFormat(m_sequenceFiles.at(2*m_imageCounter)) != usImageIo::FORMAT_MHD) //check file extention
    throw(vpException(vpException::fatalError, "usMHDSequenceReader trying to open a non-mhd file !"));

  usMetaHeaderParser mhdParser;
  std::cout << "opening mhd file : " << m_sequenceDirectory + vpIoTools::path("/") + m_sequenceFiles.at(2*m_imageCounter) << std::endl;
  mhdParser.read(m_sequenceDirectory + vpIoTools::path("/") + m_sequenceFiles.at(2*m_imageCounter)); // we skip raw files
  m_sequenceImageType = mhdParser.getImageType();
  if (m_sequenceImageType != us::PRESCAN_3D && m_sequenceImageType != us::NOT_SET) {
    throw(vpException(vpException::badValue, "Reading a non pre-scan 3D image!"));
  }
  if (mhdParser.getElementType() != usMetaHeaderParser::MET_UCHAR) {
    throw(vpException(vpException::badValue, "Reading a non unsigned char image!"));
  }

  usMetaHeaderParser::MHDHeader mhdHeader = mhdParser.getMHDHeader();
  timestamp = mhdHeader.timestamp;

  usImagePreScanSettings settings;
  settings.setTransducerRadius(mhdHeader.transducerRadius);
  settings.setScanLinePitch(mhdHeader.scanLinePitch);
  settings.setTransducerConvexity(mhdHeader.isTransducerConvex);
  settings.setAxialResolution(mhdParser.getAxialResolution());
  settings.setDepth(settings.getAxialResolution()*mhdHeader.dim[1]);
  settings.setSamplingFrequency(mhdHeader.samplingFrequency);
  settings.setTransmitFrequency(mhdHeader.transmitFrequency);
  image.setImagePreScanSettings(settings);

  usMotorSettings motorSettings;
  motorSettings.setMotorRadius(mhdHeader.motorRadius);
  motorSettings.setFramePitch(mhdHeader.framePitch);
  motorSettings.setMotorType(mhdHeader.motorType);
  motorSettings.setFrameNumber(mhdHeader.dim[2]);
  image.setMotorSettings(motorSettings);

  //resizing image in memory
  image.resize(mhdHeader.dim[0], mhdHeader.dim[1],mhdHeader.dim[2]);

  //data parsing
  usRawFileParser rawParser;
  rawParser.read(image, mhdParser.getRawFileName());

  m_imageCounter ++;
}

/**
* Acquisition method for usImagePostScan3D : fills the output image with the next volume in the sequence.
* @param [out] image The usImagePostScan3D image acquired.
* @param [out] timestamp The timestamps of the image (0 if not present in the sequence parameters). Every frame of the volume contains an associated timesamp.
*/
void usMHDSequenceReader::acquire(usImagePostScan3D<unsigned char> & image, std::vector<uint64_t> & timestamp) {

  if(m_imageCounter > m_totalImageNumber)
    throw(vpException(vpException::fatalError, "usMHDSequenceReader : end of sequence reached !"));

  if(usImageIo::getHeaderFormat(m_sequenceFiles.at(2*m_imageCounter)) != usImageIo::FORMAT_MHD) //check file extention
    throw(vpException(vpException::fatalError, "usMHDSequenceReader trying to open a non-mhd file !"));

  usMetaHeaderParser mhdParser;
  mhdParser.read(m_sequenceDirectory + vpIoTools::path("/") + m_sequenceFiles.at(2*m_imageCounter)); // we skip raw files
  m_sequenceImageType = mhdParser.getImageType();
  if (m_sequenceImageType != us::POSTSCAN_3D && m_sequenceImageType != us::NOT_SET) {
    throw(vpException(vpException::badValue, "Reading a non pre-scan 3D image!"));
  }
  if (mhdParser.getElementType() != usMetaHeaderParser::MET_UCHAR) {
    throw(vpException(vpException::badValue, "Reading a non unsigned char image!"));
  }

  usMetaHeaderParser::MHDHeader mhdHeader = mhdParser.getMHDHeader();
  timestamp = mhdHeader.timestamp;

  //resizing image in memory
  image.resize(mhdHeader.dim[0], mhdHeader.dim[1],mhdHeader.dim[2]);

  image.setTransducerRadius(mhdHeader.transducerRadius);
  image.setScanLinePitch(mhdHeader.scanLinePitch);
  image.setTransducerConvexity(mhdHeader.isTransducerConvex);
  image.setScanLineNumber(mhdHeader.scanLineNumber);
  image.setElementSpacingX(mhdHeader.elementSpacing[0]);
  image.setElementSpacingY(mhdHeader.elementSpacing[1]);
  image.setElementSpacingZ(mhdHeader.elementSpacing[2]);
  image.setMotorRadius(mhdHeader.motorRadius);
  image.setFramePitch(mhdHeader.framePitch);
  image.setFrameNumber(mhdHeader.frameNumber);
  image.setMotorType(mhdHeader.motorType);
  image.setSamplingFrequency(mhdHeader.samplingFrequency);
  image.setTransmitFrequency(mhdHeader.transmitFrequency);

  //data parsing
  usRawFileParser rawParser;
  std::string fullImageFileName = mhdParser.getRawFileName();
  rawParser.read(image,fullImageFileName);

  m_imageCounter ++;
}

/**
* Tells the used if the end of the sequence is reached.
* @return True if the end of the sequence is reached.
*/
bool usMHDSequenceReader::end() {
  return m_imageCounter >= m_totalImageNumber;
}

/**
* Returns the current type of image acquired.
* @return The image type of the sequence currently read.
*/
us::ImageType usMHDSequenceReader::getImageType() const {
  return m_sequenceImageType;
}
