#include <visp3/ustk_io/usMHDSequenceReader.h>

/**
* Constructor, initializes the member attribues.
*/
usMHDSequenceReader::usMHDSequenceReader()
  : m_sequenceDirectory(), m_sequenceImageType(us::NOT_SET), m_sequenceFiles(), m_totalImageNumber(0), m_imageCounter(0)
{
}

/**
* Destructor.
*/
usMHDSequenceReader::~usMHDSequenceReader() {}

/**
* Setter for the directory containing the mhd sequence to read. To call before calling acquire !
* @param sequenceDirectory The directory path.
*/
void usMHDSequenceReader::setSequenceDirectory(const std::string sequenceDirectory)
{
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
void usMHDSequenceReader::acquire(usImageRF2D<short int> &image, uint64_t &timestamp)
{

  if (m_imageCounter > m_totalImageNumber)
    throw(vpException(vpException::fatalError, "usMHDSequenceReader : end of sequence reached !"));

  if (usImageIo::getHeaderFormat(m_sequenceFiles.at(2 * m_imageCounter)) !=
      usImageIo::FORMAT_MHD) // check file extention
    throw(vpException(vpException::fatalError, "usMHDSequenceReader trying to open a non-mhd file !"));

  usMetaHeaderParser mhdParser;
  mhdParser.read(m_sequenceDirectory + vpIoTools::path("/") +
                 m_sequenceFiles.at(2 * m_imageCounter)); // we skip raw files
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
  settings.setDepth(settings.getAxialResolution() * mhdHeader.dim[1]);
  settings.setSamplingFrequency(mhdHeader.samplingFrequency);
  settings.setTransmitFrequency(mhdHeader.transmitFrequency);
  image.setImagePreScanSettings(settings);

  // resizing image in memory
  image.resize(mhdHeader.dim[1], mhdHeader.dim[0]);

  // data parsing
  usRawFileParser rawParser;
  rawParser.read(image, m_sequenceDirectory + vpIoTools::path("/") + m_sequenceFiles.at(2 * m_imageCounter + 1));

  m_imageCounter++;
}

/**
* Acquisition method for usImagePreScan2D : fills the output image with the next image in the sequence.
* @param [out] image The usImagePreScan2D image acquired.
* @param [out] timestamp The timestamp of the image (0 if not present in the sequence parameters).
*/
void usMHDSequenceReader::acquire(usImagePreScan2D<unsigned char> &image, uint64_t &timestamp)
{
  if (m_imageCounter > m_totalImageNumber)
    throw(vpException(vpException::fatalError, "usMHDSequenceReader : end of sequence reached !"));

  if (usImageIo::getHeaderFormat(m_sequenceFiles.at(2 * m_imageCounter)) !=
      usImageIo::FORMAT_MHD) // check file extention
    throw(vpException(vpException::fatalError, "usMHDSequenceReader trying to open a non-mhd file !"));

  usMetaHeaderParser mhdParser;
  mhdParser.read(m_sequenceDirectory + vpIoTools::path("/") +
                 m_sequenceFiles.at(2 * m_imageCounter)); // we skip raw files
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
  settings.setDepth(settings.getAxialResolution() * mhdHeader.dim[1]);
  settings.setSamplingFrequency(mhdHeader.samplingFrequency);
  settings.setTransmitFrequency(mhdHeader.transmitFrequency);
  image.setImagePreScanSettings(settings);

  // resizing image in memory
  image.resize(mhdHeader.dim[1], mhdHeader.dim[0]);

  // data parsing
  usRawFileParser rawParser;
  rawParser.read(image, m_sequenceDirectory + vpIoTools::path("/") + m_sequenceFiles.at(2 * m_imageCounter + 1));

  m_imageCounter++;
}

/**
* Acquisition method for usImagePostScan2D : fills the output image with the next image in the sequence.
* @param [out] image The usImagePostScan2D image acquired.
* @param [out] timestamp The timestamp of the image (0 if not present in the sequence parameters).
*/
void usMHDSequenceReader::acquire(usImagePostScan2D<unsigned char> &image, uint64_t &timestamp)
{
  if (m_imageCounter > m_totalImageNumber)
    throw(vpException(vpException::fatalError, "usMHDSequenceReader : end of sequence reached !"));

  if (usImageIo::getHeaderFormat(m_sequenceFiles.at(2 * m_imageCounter)) !=
      usImageIo::FORMAT_MHD) // check file extention
    throw(vpException(vpException::fatalError, "usMHDSequenceReader trying to open a non-mhd file !"));

  usMetaHeaderParser mhdParser;
  mhdParser.read(m_sequenceDirectory + vpIoTools::path("/") +
                 m_sequenceFiles.at(2 * m_imageCounter)); // we skip raw files
  m_sequenceImageType = mhdParser.getImageType();
  if (m_sequenceImageType != us::POSTSCAN_2D && m_sequenceImageType != us::NOT_SET) {
    throw(vpException(vpException::badValue, "Reading a non post-scan 2D image!"));
  }
  if (mhdParser.getElementType() != usMetaHeaderParser::MET_UCHAR) {
    throw(vpException(vpException::badValue, "Reading a non unsigned char image!"));
  }

  usMetaHeaderParser::MHDHeader mhdHeader = mhdParser.getMHDHeader();
  timestamp = mhdHeader.timestamp.at(0);

  usImagePreScanSettings settings;
  image.setTransducerRadius(mhdHeader.transducerRadius);
  image.setScanLinePitch(mhdHeader.scanLinePitch);
  image.setTransducerConvexity(mhdHeader.isTransducerConvex);

  // computing image depth from the pixel size and the transducer settings
  if (mhdHeader.isTransducerConvex) {
    // distance
    double deltaDepthPostScan2D =
        mhdHeader.transducerRadius *
        (1 - std::cos((double)((mhdHeader.scanLineNumber - 1) * mhdHeader.scanLinePitch / 2.0)));
    image.setDepth(mhdHeader.elementSpacing[1] * mhdHeader.dim[1] - deltaDepthPostScan2D);
  } else // linear transducer
    image.setDepth(mhdHeader.elementSpacing[1] * mhdHeader.dim[1]);

  image.setWidthResolution(mhdHeader.dim[0]);
  image.setWidthResolution(mhdHeader.dim[1]);
  image.setSamplingFrequency(mhdHeader.samplingFrequency);
  image.setTransmitFrequency(mhdHeader.transmitFrequency);

  // resizing image in memory
  image.resize(mhdHeader.dim[1], mhdHeader.dim[0]);

  // data parsing
  usRawFileParser rawParser;
  rawParser.read(image, m_sequenceDirectory + vpIoTools::path("/") + m_sequenceFiles.at(2 * m_imageCounter + 1));

  m_imageCounter++;
}

/**
* Acquisition method for usImageRF3D : fills the output image with the next volume in the sequence.
* @param [out] image The usImageRF3D image acquired.
* @param [out] timestamp The timestamps of the image (0 if not present in the sequence parameters). Every frame of the
* volume contains an associated timesamp.
* If the volume number in the sequence is odd, the timestamp vector is reversed : to fit the real conditions of the
* sweeping motor of a 3D probe (along + / - Z axis every new volume).
* First volume of a sequence is considered going along Z axis, and the second along -Z, etc...
*/
void usMHDSequenceReader::acquire(usImageRF3D<short int> &image, std::vector<uint64_t> &timestamp)
{
  if (m_imageCounter > m_totalImageNumber)
    throw(vpException(vpException::fatalError, "usMHDSequenceReader : end of sequence reached !"));

  if (usImageIo::getHeaderFormat(m_sequenceFiles.at(2 * m_imageCounter)) !=
      usImageIo::FORMAT_MHD) // check file extention
    throw(vpException(vpException::fatalError, "usMHDSequenceReader trying to open a non-mhd file !"));

  usMetaHeaderParser mhdParser;
  mhdParser.read(m_sequenceDirectory + vpIoTools::path("/") +
                 m_sequenceFiles.at(2 * m_imageCounter)); // we skip raw files
  m_sequenceImageType = mhdParser.getImageType();
  if (m_sequenceImageType != us::RF_3D && m_sequenceImageType != us::NOT_SET) {
    throw(vpException(vpException::badValue, "Reading a non rf 3D image!"));
  }
  if (mhdParser.getElementType() != usMetaHeaderParser::MET_SHORT) {
    throw(vpException(vpException::badValue, "Reading a non short image!"));
  }

  usMetaHeaderParser::MHDHeader mhdHeader = mhdParser.getMHDHeader();
  timestamp = mhdHeader.timestamp;
  if (m_imageCounter % 2 == 1) // odd volume: we reverse it
    std::reverse(timestamp.begin(), timestamp.end());

  usImagePreScanSettings settings;
  settings.setTransducerRadius(mhdHeader.transducerRadius);
  settings.setScanLinePitch(mhdHeader.scanLinePitch);
  settings.setTransducerConvexity(mhdHeader.isTransducerConvex);
  settings.setAxialResolution(mhdParser.getAxialResolution());
  settings.setDepth(settings.getAxialResolution() * mhdHeader.dim[1]);
  settings.setSamplingFrequency(mhdHeader.samplingFrequency);
  settings.setTransmitFrequency(mhdHeader.transmitFrequency);
  image.setImagePreScanSettings(settings);

  usMotorSettings motorSettings;
  motorSettings.setMotorRadius(mhdHeader.motorRadius);
  motorSettings.setFramePitch(mhdHeader.framePitch);
  motorSettings.setMotorType(mhdHeader.motorType);
  motorSettings.setFrameNumber(mhdHeader.dim[2]);
  image.setMotorSettings(motorSettings);

  // resizing image in memory
  image.resize(mhdHeader.dim[1], mhdHeader.dim[0], mhdHeader.dim[2]);

  // data parsing
  usRawFileParser rawParser;
  rawParser.read(image, m_sequenceDirectory + vpIoTools::path("/") + m_sequenceFiles.at(2 * m_imageCounter + 1));

  m_imageCounter++;
}

/**
* Acquisition method for usImagePreScan3D : fills the output image with the next volume in the sequence.
* @param [out] image The usImagePreScan3D image acquired.
* @param [out] timestamp The timestamps of the image (0 if not present in the sequence parameters). Every frame of the
* volume contains an associated timesamp.
* If the volume number in the sequence is odd, the timestamp vector is reversed : to fit the real conditions of the
* sweeping motor of a 3D probe (along + / - Z axis every new volume).
* First volume of a sequence is considered going along Z axis, and the second along -Z, etc...
*/
void usMHDSequenceReader::acquire(usImagePreScan3D<unsigned char> &image, std::vector<uint64_t> &timestamp)
{
  if (m_imageCounter > m_totalImageNumber)
    throw(vpException(vpException::fatalError, "usMHDSequenceReader : end of sequence reached !"));

  if (usImageIo::getHeaderFormat(m_sequenceFiles.at(2 * m_imageCounter)) !=
      usImageIo::FORMAT_MHD) // check file extention
    throw(vpException(vpException::fatalError, "usMHDSequenceReader trying to open a non-mhd file !"));

  usMetaHeaderParser mhdParser;
  mhdParser.read(m_sequenceDirectory + vpIoTools::path("/") +
                 m_sequenceFiles.at(2 * m_imageCounter)); // we skip raw files
  m_sequenceImageType = mhdParser.getImageType();
  if (m_sequenceImageType != us::PRESCAN_3D && m_sequenceImageType != us::NOT_SET) {
    throw(vpException(vpException::badValue, "Reading a non pre-scan 3D image!"));
  }
  if (mhdParser.getElementType() != usMetaHeaderParser::MET_UCHAR) {
    throw(vpException(vpException::badValue, "Reading a non unsigned char image!"));
  }

  usMetaHeaderParser::MHDHeader mhdHeader = mhdParser.getMHDHeader();
  timestamp = mhdHeader.timestamp;
  if (m_imageCounter % 2 == 1) // odd volume: we reverse it
    std::reverse(timestamp.begin(), timestamp.end());

  usImagePreScanSettings settings;
  settings.setTransducerRadius(mhdHeader.transducerRadius);
  settings.setScanLinePitch(mhdHeader.scanLinePitch);
  settings.setTransducerConvexity(mhdHeader.isTransducerConvex);
  settings.setAxialResolution(mhdParser.getAxialResolution());
  settings.setDepth(settings.getAxialResolution() * mhdHeader.dim[1]);
  settings.setSamplingFrequency(mhdHeader.samplingFrequency);
  settings.setTransmitFrequency(mhdHeader.transmitFrequency);
  image.setImagePreScanSettings(settings);

  usMotorSettings motorSettings;
  motorSettings.setMotorRadius(mhdHeader.motorRadius);
  motorSettings.setFramePitch(mhdHeader.framePitch);
  motorSettings.setMotorType(mhdHeader.motorType);
  motorSettings.setFrameNumber(mhdHeader.dim[2]);
  image.setMotorSettings(motorSettings);

  // resizing image in memory
  image.resize(mhdHeader.dim[1], mhdHeader.dim[0], mhdHeader.dim[2]);

  // data parsing
  usRawFileParser rawParser;
  rawParser.read(image, m_sequenceDirectory + vpIoTools::path("/") + m_sequenceFiles.at(2 * m_imageCounter + 1));

  m_imageCounter++;
}

/**
* Acquisition method for usImagePostScan3D : fills the output image with the next volume in the sequence.
* @param [out] image The usImagePostScan3D image acquired.
* @param [out] timestamp The usImagePostScan3D timestamp (0 if not present in mhd file).
*/
void usMHDSequenceReader::acquire(usImagePostScan3D<unsigned char> &image, uint64_t &timestamp)
{
  if (m_imageCounter > m_totalImageNumber)
    throw(vpException(vpException::fatalError, "usMHDSequenceReader : end of sequence reached !"));

  if (usImageIo::getHeaderFormat(m_sequenceFiles.at(2 * m_imageCounter)) !=
      usImageIo::FORMAT_MHD) // check file extention
    throw(vpException(vpException::fatalError, "usMHDSequenceReader trying to open a non-mhd file !"));

  usMetaHeaderParser mhdParser;
  mhdParser.read(m_sequenceDirectory + vpIoTools::path("/") +
                 m_sequenceFiles.at(2 * m_imageCounter)); // we skip raw files
  m_sequenceImageType = mhdParser.getImageType();
  if (m_sequenceImageType != us::POSTSCAN_3D && m_sequenceImageType != us::NOT_SET) {
    throw(vpException(vpException::badValue, "Reading a non post-scan 3D image!"));
  }
  if (mhdParser.getElementType() != usMetaHeaderParser::MET_UCHAR) {
    throw(vpException(vpException::badValue, "Reading a non unsigned char image!"));
  }

  usMetaHeaderParser::MHDHeader mhdHeader = mhdParser.getMHDHeader();
  timestamp = 0;
  if (mhdHeader.timestamp.size() > 0)
    timestamp = mhdHeader.timestamp.at(0);

  // resizing image in memory
  image.resize(mhdHeader.dim[1], mhdHeader.dim[0], mhdHeader.dim[2]);

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

  // data parsing
  usRawFileParser rawParser;
  rawParser.read(image, m_sequenceDirectory + vpIoTools::path("/") + m_sequenceFiles.at(2 * m_imageCounter + 1));

  m_imageCounter++;
}

/**
* Tells the used if the end of the sequence is reached.
* @return True if the end of the sequence is reached.
*/
bool usMHDSequenceReader::end() { return m_imageCounter >= m_totalImageNumber; }

/**
* Returns the current type of image acquired.
* @return The image type of the sequence currently read.
*/
us::ImageType usMHDSequenceReader::getImageType() const { return m_sequenceImageType; }

/**
* Returns the timestamp of next frame (use only for 2D sequence).
* @return The timestamp of next frame.
*/
uint64_t usMHDSequenceReader::getNextTimeStamp()
{
  usMetaHeaderParser mhdParser;
  mhdParser.read(m_sequenceDirectory + vpIoTools::path("/") +
                 m_sequenceFiles.at(2 * m_imageCounter)); // we skip raw files, imagecounter already incremented
  return mhdParser.getMHDHeader().timestamp.at(0);
}

/**
* Returns the timestamps of next volume.
* @return The timestamps of next volume.
*/
std::vector<uint64_t> usMHDSequenceReader::getNextTimeStamps()
{
  usMetaHeaderParser mhdParser;
  mhdParser.read(m_sequenceDirectory + vpIoTools::path("/") +
                 m_sequenceFiles.at(2 * m_imageCounter)); // we skip raw files, imagecounter already incremented
  std::vector<uint64_t> timestamps = mhdParser.getMHDHeader().timestamp;
  if (m_imageCounter % 2 == 0) // current volume is even => next volume is odd
    std::reverse(timestamps.begin(), timestamps.end());
  return timestamps;
}

/**
* Returns the current image number, of last image acquired.
* @return The image number (volume number for 3D sequences, frame number for 2D sequences).
*/
int usMHDSequenceReader::getImageNumber() const { return m_imageCounter; }

/**
* Returns the total image number in sequence.
* @return The total image number (total volume number for 3D sequences, total frame number for 2D sequences).
*/
int usMHDSequenceReader::getTotalImageNumber() const { return m_totalImageNumber; }

/**
* Acquisition method for specific image in the sequence for usImageRF2D : fills the output image with the next volume in
* the sequence.
* @param [in] imageNumber Image number in sequence to acquire (from 0 to total image number - 1)
* @param [out] image The usImageRF2D image acquired.
* @param [out] timestamp The usImageRF2D timestamp (0 if not present in mhd file).
*/
void usMHDSequenceReader::getImage(unsigned int imageNumber, usImageRF2D<short int> &image, uint64_t &timestamp)
{

  if (imageNumber > (unsigned int)m_totalImageNumber)
    throw(vpException(vpException::fatalError,
                      "usMHDSequenceReader : trying to acquire an image with an index out of sequence bounds !"));

  if (usImageIo::getHeaderFormat(m_sequenceFiles.at(2 * imageNumber)) != usImageIo::FORMAT_MHD) // check file extention
    throw(vpException(vpException::fatalError, "usMHDSequenceReader trying to open a non-mhd file !"));

  usMetaHeaderParser mhdParser;
  mhdParser.read(m_sequenceDirectory + vpIoTools::path("/") + m_sequenceFiles.at(2 * imageNumber)); // we skip raw files
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
  settings.setDepth(settings.getAxialResolution() * mhdHeader.dim[1]);
  settings.setSamplingFrequency(mhdHeader.samplingFrequency);
  settings.setTransmitFrequency(mhdHeader.transmitFrequency);
  image.setImagePreScanSettings(settings);

  // resizing image in memory
  image.resize(mhdHeader.dim[1], mhdHeader.dim[0]);

  // data parsing
  usRawFileParser rawParser;
  rawParser.read(image, m_sequenceDirectory + vpIoTools::path("/") + m_sequenceFiles.at(2 * imageNumber + 1));
}

/**
* Acquisition method for specific image in the sequence for usImagePreScan2D : fills the output image with the next
* volume in
* the sequence.
* @param [in] imageNumber Image number in sequence to acquire (from 0 to total image number - 1)
* @param [out] image The usImagePreScan2D image acquired.
* @param [out] timestamp The usImagePreScan2D timestamp (0 if not present in mhd file).
*/
void usMHDSequenceReader::getImage(unsigned int imageNumber, usImagePreScan2D<unsigned char> &image,
                                   uint64_t &timestamp)
{
  if (imageNumber > (unsigned int)m_totalImageNumber)
    throw(vpException(vpException::fatalError,
                      "usMHDSequenceReader : trying to acquire an image with an index out of sequence bounds !"));

  if (usImageIo::getHeaderFormat(m_sequenceFiles.at(2 * imageNumber)) != usImageIo::FORMAT_MHD) // check file extention
    throw(vpException(vpException::fatalError, "usMHDSequenceReader trying to open a non-mhd file !"));

  usMetaHeaderParser mhdParser;
  mhdParser.read(m_sequenceDirectory + vpIoTools::path("/") + m_sequenceFiles.at(2 * imageNumber)); // we skip raw files
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
  settings.setDepth(settings.getAxialResolution() * mhdHeader.dim[1]);
  settings.setSamplingFrequency(mhdHeader.samplingFrequency);
  settings.setTransmitFrequency(mhdHeader.transmitFrequency);
  image.setImagePreScanSettings(settings);

  // resizing image in memory
  image.resize(mhdHeader.dim[1], mhdHeader.dim[0]);

  // data parsing
  usRawFileParser rawParser;
  rawParser.read(image, m_sequenceDirectory + vpIoTools::path("/") + m_sequenceFiles.at(2 * imageNumber + 1));
}

/**
* Acquisition method for specific image in the sequence for usImagePostScan2D : fills the output image with the next
* volume in
* the sequence.
* @param [in] imageNumber Image number in sequence to acquire (from 0 to total image number - 1)
* @param [out] image The usImagePostScan2D image acquired.
* @param [out] timestamp The usImagePostScan2D timestamp (0 if not present in mhd file).
*/
void usMHDSequenceReader::getImage(unsigned int imageNumber, usImagePostScan2D<unsigned char> &image,
                                   uint64_t &timestamp)
{
  if (imageNumber > (unsigned int)m_totalImageNumber)
    throw(vpException(vpException::fatalError,
                      "usMHDSequenceReader : trying to acquire an image with an index out of sequence bounds !"));

  if (usImageIo::getHeaderFormat(m_sequenceFiles.at(2 * imageNumber)) != usImageIo::FORMAT_MHD) // check file extention
    throw(vpException(vpException::fatalError, "usMHDSequenceReader trying to open a non-mhd file !"));

  usMetaHeaderParser mhdParser;
  mhdParser.read(m_sequenceDirectory + vpIoTools::path("/") + m_sequenceFiles.at(2 * imageNumber)); // we skip raw files
  m_sequenceImageType = mhdParser.getImageType();
  if (m_sequenceImageType != us::POSTSCAN_2D && m_sequenceImageType != us::NOT_SET) {
    throw(vpException(vpException::badValue, "Reading a non post-scan 2D image!"));
  }
  if (mhdParser.getElementType() != usMetaHeaderParser::MET_UCHAR) {
    throw(vpException(vpException::badValue, "Reading a non unsigned char image!"));
  }

  usMetaHeaderParser::MHDHeader mhdHeader = mhdParser.getMHDHeader();
  timestamp = mhdHeader.timestamp.at(0);

  usImagePreScanSettings settings;
  image.setTransducerRadius(mhdHeader.transducerRadius);
  image.setScanLinePitch(mhdHeader.scanLinePitch);
  image.setTransducerConvexity(mhdHeader.isTransducerConvex);

  // computing image depth from the pixel size and the transducer settings
  if (mhdHeader.isTransducerConvex) {
    // distance
    double deltaDepthPostScan2D =
        mhdHeader.transducerRadius *
        (1 - std::cos((double)((mhdHeader.scanLineNumber - 1) * mhdHeader.scanLinePitch / 2.0)));
    image.setDepth(mhdHeader.elementSpacing[1] * mhdHeader.dim[1] - deltaDepthPostScan2D);
  } else // linear transducer
    image.setDepth(mhdHeader.elementSpacing[1] * mhdHeader.dim[1]);

  image.setWidthResolution(mhdHeader.dim[0]);
  image.setWidthResolution(mhdHeader.dim[1]);
  image.setSamplingFrequency(mhdHeader.samplingFrequency);
  image.setTransmitFrequency(mhdHeader.transmitFrequency);

  // resizing image in memory
  image.resize(mhdHeader.dim[1], mhdHeader.dim[0]);

  // data parsing
  usRawFileParser rawParser;
  rawParser.read(image, m_sequenceDirectory + vpIoTools::path("/") + m_sequenceFiles.at(2 * imageNumber + 1));
}

/**
* Acquisition method for specific image in the sequence for usImageRF3D : fills the output image with the next volume in
* the sequence.
* @param [in] imageNumber Image number in sequence to acquire (from 0 to total image number - 1)
* @param [out] image The usImageRF3D image acquired.
* @param [out] timestamp The usImageRF3D timestamp (0 if not present in mhd file).
*/
void usMHDSequenceReader::getImage(unsigned int imageNumber, usImageRF3D<short int> &image,
                                   std::vector<uint64_t> &timestamp)
{
  if (imageNumber > (unsigned int)m_totalImageNumber)
    throw(vpException(vpException::fatalError, "usMHDSequenceReader : end of sequence reached !"));

  if (usImageIo::getHeaderFormat(m_sequenceFiles.at(2 * imageNumber)) != usImageIo::FORMAT_MHD) // check file extention
    throw(vpException(vpException::fatalError, "usMHDSequenceReader trying to open a non-mhd file !"));

  usMetaHeaderParser mhdParser;
  mhdParser.read(m_sequenceDirectory + vpIoTools::path("/") + m_sequenceFiles.at(2 * imageNumber)); // we skip raw files
  m_sequenceImageType = mhdParser.getImageType();
  if (m_sequenceImageType != us::RF_3D && m_sequenceImageType != us::NOT_SET) {
    throw(vpException(vpException::badValue, "Reading a non rf 3D image!"));
  }
  if (mhdParser.getElementType() != usMetaHeaderParser::MET_SHORT) {
    throw(vpException(vpException::badValue, "Reading a non short image!"));
  }

  usMetaHeaderParser::MHDHeader mhdHeader = mhdParser.getMHDHeader();
  timestamp = mhdHeader.timestamp;
  if (imageNumber % 2 == 1) // odd volume: we reverse it
    std::reverse(timestamp.begin(), timestamp.end());

  usImagePreScanSettings settings;
  settings.setTransducerRadius(mhdHeader.transducerRadius);
  settings.setScanLinePitch(mhdHeader.scanLinePitch);
  settings.setTransducerConvexity(mhdHeader.isTransducerConvex);
  settings.setAxialResolution(mhdParser.getAxialResolution());
  settings.setDepth(settings.getAxialResolution() * mhdHeader.dim[1]);
  settings.setSamplingFrequency(mhdHeader.samplingFrequency);
  settings.setTransmitFrequency(mhdHeader.transmitFrequency);
  image.setImagePreScanSettings(settings);

  usMotorSettings motorSettings;
  motorSettings.setMotorRadius(mhdHeader.motorRadius);
  motorSettings.setFramePitch(mhdHeader.framePitch);
  motorSettings.setMotorType(mhdHeader.motorType);
  motorSettings.setFrameNumber(mhdHeader.dim[2]);
  image.setMotorSettings(motorSettings);

  // resizing image in memory
  image.resize(mhdHeader.dim[1], mhdHeader.dim[0], mhdHeader.dim[2]);

  // data parsing
  usRawFileParser rawParser;
  rawParser.read(image, m_sequenceDirectory + vpIoTools::path("/") + m_sequenceFiles.at(2 * imageNumber + 1));
}

/**
* Acquisition method for specific image in the sequence for usImagePreScan3D : fills the output image with the next
* volume in
* the sequence.
* @param [in] imageNumber Image number in sequence to acquire (from 0 to total image number - 1)
* @param [out] image The usImagePreScan3D image acquired.
* @param [out] timestamp The usImagePreScan3D timestamp (0 if not present in mhd file).
*/
void usMHDSequenceReader::getImage(unsigned int imageNumber, usImagePreScan3D<unsigned char> &image,
                                   std::vector<uint64_t> &timestamp)
{
  if (imageNumber > (unsigned int)m_totalImageNumber)
    throw(vpException(vpException::fatalError, "usMHDSequenceReader : end of sequence reached !"));

  if (usImageIo::getHeaderFormat(m_sequenceFiles.at(2 * imageNumber)) != usImageIo::FORMAT_MHD) // check file extention
    throw(vpException(vpException::fatalError, "usMHDSequenceReader trying to open a non-mhd file !"));

  usMetaHeaderParser mhdParser;
  mhdParser.read(m_sequenceDirectory + vpIoTools::path("/") + m_sequenceFiles.at(2 * imageNumber)); // we skip raw files
  m_sequenceImageType = mhdParser.getImageType();
  if (m_sequenceImageType != us::PRESCAN_3D && m_sequenceImageType != us::NOT_SET) {
    throw(vpException(vpException::badValue, "Reading a non pre-scan 3D image!"));
  }
  if (mhdParser.getElementType() != usMetaHeaderParser::MET_UCHAR) {
    throw(vpException(vpException::badValue, "Reading a non unsigned char image!"));
  }

  usMetaHeaderParser::MHDHeader mhdHeader = mhdParser.getMHDHeader();
  timestamp = mhdHeader.timestamp;
  if (imageNumber % 2 == 1) // odd volume: we reverse it
    std::reverse(timestamp.begin(), timestamp.end());

  usImagePreScanSettings settings;
  settings.setTransducerRadius(mhdHeader.transducerRadius);
  settings.setScanLinePitch(mhdHeader.scanLinePitch);
  settings.setTransducerConvexity(mhdHeader.isTransducerConvex);
  settings.setAxialResolution(mhdParser.getAxialResolution());
  settings.setDepth(settings.getAxialResolution() * mhdHeader.dim[1]);
  settings.setSamplingFrequency(mhdHeader.samplingFrequency);
  settings.setTransmitFrequency(mhdHeader.transmitFrequency);
  image.setImagePreScanSettings(settings);

  usMotorSettings motorSettings;
  motorSettings.setMotorRadius(mhdHeader.motorRadius);
  motorSettings.setFramePitch(mhdHeader.framePitch);
  motorSettings.setMotorType(mhdHeader.motorType);
  motorSettings.setFrameNumber(mhdHeader.dim[2]);
  image.setMotorSettings(motorSettings);

  // resizing image in memory
  image.resize(mhdHeader.dim[1], mhdHeader.dim[0], mhdHeader.dim[2]);

  // data parsing
  usRawFileParser rawParser;
  rawParser.read(image, m_sequenceDirectory + vpIoTools::path("/") + m_sequenceFiles.at(2 * imageNumber + 1));
}

/**
* Acquisition method for specific image in the sequence for usImagePostScan3D : fills the output image with the next
* volume in
* the sequence.
* @param [in] imageNumber Image number in sequence to acquire (from 0 to total image number - 1)
* @param [out] image The usImagePostScan3D image acquired.
* @param [out] timestamp The usImagePostScan3D timestamp (0 if not present in mhd file).
*/
void usMHDSequenceReader::getImage(unsigned int imageNumber, usImagePostScan3D<unsigned char> &image,
                                   uint64_t &timestamp)
{
  if (imageNumber > (unsigned int)m_totalImageNumber)
    throw(vpException(vpException::fatalError,
                      "usMHDSequenceReader : trying to acquire an image with an index out of sequence bounds !"));

  if (usImageIo::getHeaderFormat(m_sequenceFiles.at(2 * imageNumber)) != usImageIo::FORMAT_MHD) // check file extention
    throw(vpException(vpException::fatalError, "usMHDSequenceReader trying to open a non-mhd file !"));

  usMetaHeaderParser mhdParser;
  mhdParser.read(m_sequenceDirectory + vpIoTools::path("/") + m_sequenceFiles.at(2 * imageNumber)); // we skip raw files
  m_sequenceImageType = mhdParser.getImageType();
  if (m_sequenceImageType != us::POSTSCAN_3D && m_sequenceImageType != us::NOT_SET) {
    throw(vpException(vpException::badValue, "Reading a non post-scan 3D image!"));
  }
  if (mhdParser.getElementType() != usMetaHeaderParser::MET_UCHAR) {
    throw(vpException(vpException::badValue, "Reading a non unsigned char image!"));
  }

  usMetaHeaderParser::MHDHeader mhdHeader = mhdParser.getMHDHeader();
  timestamp = 0;
  if (mhdHeader.timestamp.size() > 0)
    timestamp = mhdHeader.timestamp.at(0);

  // resizing image in memory
  image.resize(mhdHeader.dim[1], mhdHeader.dim[0], mhdHeader.dim[2]);

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

  // data parsing
  usRawFileParser rawParser;
  rawParser.read(image, m_sequenceDirectory + vpIoTools::path("/") + m_sequenceFiles.at(2 * imageNumber + 1));
}
