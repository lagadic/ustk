#include "usVirtualServer.h"

/**
* Constructor, sets the sequence
* @param image The usImageRF2D image to write.
* @param timestamp The timestamp of the image.
*/
usVirtualServer::usVirtualServer(std::string sequencePath, QObject *parent) : QObject(parent), m_tcpServer(), m_serverIsSendingImages(false)
{
  imageHeader.frameCount = 0;
  //read sequence parameters
  setSequencePath(sequencePath); //opens first image of the sequence

  //init TCP server
  // set : acceptTheConnection() will be called whenever there is a new connection
  connect(&m_tcpServer, SIGNAL(newConnection()), this, SLOT(acceptTheConnection()));

  // loop sending image activation / desactivation
  connect(this, SIGNAL(runAcquisitionSignal(bool)), this, SLOT(runAcquisition(bool)));
  connect(this, SIGNAL(startSendingLoopSignal()), this, SLOT(startSendingLoop()));

  //Start listening on port 8080
  QString portNum = QString::number(8080);
  bool status = m_tcpServer.listen(QHostAddress::Any, portNum.toUShort() );

  // Check, if the server did start correctly or not
  if( status == true )
  {
    std::cout << "TCP server Started\nServer now listening on port# " << portNum.toStdString() << std::endl;
  }
  else
  {
    std::cout << "TCP server start failure" << m_tcpServer.errorString().toStdString() << std::endl;
  }
}

usVirtualServer::~usVirtualServer()
{
}

// This function is called whenever we have an incoming connection
void usVirtualServer::acceptTheConnection()
{

  connectionSoc = m_tcpServer.nextPendingConnection();

  connect(connectionSoc, SIGNAL(readyRead()), this, SLOT(readIncomingData()) );

  connect(connectionSoc, SIGNAL(disconnected()), this, SLOT(connectionAboutToClose()));
}

// Will be called whenever the data (coming from client) is available
void usVirtualServer::readIncomingData()
{
  //headers possible to be received
  usVirtualServer::usInitHeaderIncomming headerInit;

  //reading part

  //prepare reading in QDataStream
  QDataStream in;
  in.setDevice(connectionSoc);
  in.setVersion(QDataStream::Qt_5_0);

  //read header id
  int id;
  in >> id;

  if(id == 1) { // init header
    initWithoutUpdate = true;
    in >> headerInit.probeId;
    in >> headerInit.slotId;
    in >> headerInit.imagingMode;

    //values by default for virtual server
    confirmHeader.initOk = 1;
    confirmHeader.probeId = 0;

    //send back default params
    QByteArray block;
    QDataStream out(&block,QIODevice::WriteOnly);
    out.setVersion(QDataStream::Qt_5_0);
    out << confirmHeader.headerId;
    out << confirmHeader.initOk;
    out << confirmHeader.probeId;

    writeInitAcquisitionParameters(out,headerInit.imagingMode);

    connectionSoc->write(block);
  }
  else if(id == 2) { //update header
    throw(vpException(vpException::fatalError, "no update available for virtual server !"));
  }
  else if(id == 3) { // run - stop command
    bool run;
    in >> run;

    emit(runAcquisitionSignal(run));
  }
  else {
    std::cout << "ERROR : unknown data received !" << std::endl;
  }
}

// Will be called whenever the connection is close by the client
void usVirtualServer::connectionAboutToClose()
{
  // Set this text into the label
  std::cout << "Connection to client closed" << std::endl;

  // Close the connection (Say bye)
  connectionSoc->close();

  //delete sequcence reader and reset frame count (to prepare them for a potential new connection)
  m_sequenceReaderPostScan = usSequenceReader<usImagePostScan2D <unsigned char> > ();
  m_sequenceReaderPreScan = usSequenceReader<usImagePreScan2D <unsigned char> > ();
  imageHeader.frameCount = 0;

  // Re-open the sequence to prepare next connection incomming (the server is still running)
  setSequencePath(m_sequencePath);
}

QTcpSocket* usVirtualServer::getSocket() {
  return connectionSoc;
}


void usVirtualServer::writeInitAcquisitionParameters(QDataStream & stream, int imagingMode) {

  int transmitFrequency = 0;
  int samplingFrequency = 0;
  bool postScanMode = false;
  int postScanHeight = 0;
  int postScanWidth = 0;
  int imageDepth = 0;
  int sector = 100;
  bool activateMotor = false;
  int motorPosition = 0;
  int framesPerVolume = 1;
  int stepsPerFrame = 0;
  int transmitFrequencyMin = 0;
  int samplingFrequencyMin = 0;
  int imagingModeMin = 0;
  int imageDepthMin = 0;
  int sectorMin = 100;
  int motorPositionMin = 0;
  int framesPerVolumeMin = 1;
  int stepsPerFrameMin = 0;
  int transmitFrequencyMax = 0;
  int samplingFrequencyMax = 0;
  int imagingModeMax = 27;
  int imageDepthMax = 0;
  int sectorMax = 100;
  int motorPositionMax = 0;
  int framesPerVolumeMax = 0;
  int stepsPerFrameMax = 0;


  if(m_imageType == us::PRESCAN_2D) {
    transmitFrequency = m_preScanImage2d.getTransmitFrequency();
    transmitFrequencyMin = m_preScanImage2d.getTransmitFrequency();
    transmitFrequencyMax = m_preScanImage2d.getTransmitFrequency();

    samplingFrequency = m_preScanImage2d.getSamplingFrequency();
    samplingFrequencyMin = m_preScanImage2d.getSamplingFrequency();
    samplingFrequencyMax = m_preScanImage2d.getSamplingFrequency();

    imageDepth = m_preScanImage2d.getDepth();
    imageDepthMin = m_preScanImage2d.getDepth();
    imageDepthMax = m_preScanImage2d.getDepth();
  }
  else if (m_imageType == us::POSTSCAN_2D) {
    transmitFrequency = m_postScanImage2d.getTransmitFrequency();
    transmitFrequencyMin = m_postScanImage2d.getTransmitFrequency();
    transmitFrequencyMax = m_postScanImage2d.getTransmitFrequency();

    samplingFrequency = m_postScanImage2d.getSamplingFrequency();
    samplingFrequencyMin = m_postScanImage2d.getSamplingFrequency();
    samplingFrequencyMax = m_postScanImage2d.getSamplingFrequency();

    imageDepth = m_postScanImage2d.getDepth();
    imageDepthMin = m_postScanImage2d.getDepth();
    imageDepthMax = m_postScanImage2d.getDepth();

    postScanMode = true;
  }

  stream << transmitFrequency;
  stream << samplingFrequency;
  stream << imagingMode;
  stream << postScanMode;
  stream << postScanHeight;
  stream << postScanWidth;
  stream << imageDepth;
  stream << sector;
  stream << activateMotor;
  stream << motorPosition;
  stream << framesPerVolume;
  stream << stepsPerFrame;
  stream << transmitFrequencyMin;
  stream << samplingFrequencyMin;
  stream << imagingModeMin;
  stream << imageDepthMin;
  stream << sectorMin;
  stream << motorPositionMin;
  stream << framesPerVolumeMin;
  stream << stepsPerFrameMin;
  stream << transmitFrequencyMax;
  stream << samplingFrequencyMax;
  stream << imagingModeMax;
  stream << imageDepthMax;
  stream << sectorMax;
  stream << motorPositionMax;
  stream << framesPerVolumeMax;
  stream << stepsPerFrameMax;
}

void usVirtualServer::setSequencePath(const std::string sequencePath) {

  if(vpIoTools::checkFilename(sequencePath)) { // xml file pointing on a sequence of 2d images (pre-scan or post-scan)

    m_sequencePath = sequencePath;
    m_sequenceReaderPostScan.setSequenceFileName(sequencePath);
    m_sequenceReaderPreScan.setSequenceFileName(sequencePath);

    //try to open post-scan sequence
    try {
      uint64_t localTimestamp;
      m_sequenceReaderPostScan.open(m_postScanImage2d,localTimestamp);
      imageHeader.timeStamp = localTimestamp;
      if(imageHeader.timeStamp == 0) { // timestamps are requested for virtual server
        throw(vpException(vpException::fatalError), "usVirtualServer error : no timestamp associated in sequence !");
      }
      m_imageType = us::POSTSCAN_2D;
      m_isMHDSequence = false;
    } catch(...) {
      //if we have an exception, it's not a post-scan image. So we try a pre-scan
      try {
        uint64_t localTimestamp;
        m_sequenceReaderPreScan.open(m_preScanImage2d,localTimestamp);
        imageHeader.timeStamp = localTimestamp;
        if(imageHeader.timeStamp == 0) { // timestamps are requested for virtual server
          throw(vpException(vpException::fatalError), "usVirtualServer error : no timestamp associated in sequence !");
        }
        invertRowsColsOnPreScan(); //to fit with ultrasonix grabbers (pre-scan image is inverted in porta SDK)
        m_imageType = us::PRESCAN_2D;
        m_isMHDSequence = false;
      }
      catch(...) {
        throw(vpException(vpException::badValue), "usVirtualServer error : trying to open a xml image sequence not managed (neither pre-scan 2D nor post-scan 2D)or with no timestamp associated");
      }
    }
  }
  // case of a directory containing a sequence of mhd/raw images
  else if(vpIoTools::checkDirectory(sequencePath) && usImageIo::getHeaderFormat(vpIoTools::getDirFiles(sequencePath).front()) == usImageIo::FORMAT_MHD) {
    m_MHDSequenceReader.setSequenceDirectory(sequencePath);
    uint64_t localTimestamp;
    // at this point, we don't know the type of image contained in the sequence, we have to try them all
    try {
      m_MHDSequenceReader.acquire(m_rfImage2d,localTimestamp);
    } catch(...) {//if we have an exception, it's not a RF 2D image. So we try a pre-scan 2D
      try {
        m_MHDSequenceReader.acquire(m_preScanImage2d,localTimestamp);
      } catch(...) {// it's not a pre-scan 2D image...
        try {
          m_MHDSequenceReader.acquire(m_postScanImage2d,localTimestamp);
        } catch(...) {// it's not a post-scan 2D image...
          try {
            m_MHDSequenceReader.acquire(m_rfImage3d,localTimestamp);
          } catch(...) {// it's not a pre-scan 2D image...
            try {
              m_MHDSequenceReader.acquire(m_preScanImage3d,localTimestamp);
            } catch(...) {// it's not a pre-scan 2D image...
              try {
                m_MHDSequenceReader.acquire(m_postScanImage3d,localTimestamp);
              } catch(...) {// it's not a valid type
                throw(vpException(vpException::badValue), "usVirtualServer error : trying to open a mhd image sequence not managed !");

              }
            }
          }
        }
      }
    }

    m_imageType = m_MHDSequenceReader.getImageType();
    m_isMHDSequence = true;
    imageHeader.timeStamp = localTimestamp;
  }
  else {
    throw(vpException(vpException::badValue, "usVirtualServer error : sequence path incorrect !"));
  }
}

void usVirtualServer::startSendingLoop() {
  if(m_isMHDSequence)
    sendingLoopSequenceMHD();
  else
    sendingLoopSequenceXml();
}

void usVirtualServer::sendingLoopSequenceXml() {

  bool endOfSequence = false;
  while(m_serverIsSendingImages && ! endOfSequence ) {
    m_previousImageTimestamp = imageHeader.timeStamp;
    //manage first frame sent (already aquired with open() method)
    if(imageHeader.frameCount != 0) {
      if(m_imageType == us::PRESCAN_2D) {
        uint64_t localTimestamp;
        m_sequenceReaderPreScan.acquire(m_preScanImage2d,localTimestamp);
        invertRowsColsOnPreScan(); //to fit with ultrasonix grabbers (pre-scan image is inverted in porta SDK)
        imageHeader.timeStamp = localTimestamp;
        imageHeader.imageType = 0;
      }
      else if(m_imageType == us::POSTSCAN_2D) {
        uint64_t localTimestamp;
        m_sequenceReaderPostScan.acquire(m_postScanImage2d,localTimestamp);
        imageHeader.timeStamp = localTimestamp;
        imageHeader.imageType = 1;
      }
    }

    imageHeader.dataRate = 1000.0 / (imageHeader.timeStamp - m_previousImageTimestamp);

    //WAITING PROCESS (to respect sequence timestamps)
    vpTime::wait((double) (imageHeader.timeStamp - m_previousImageTimestamp));

    QByteArray block;
    QDataStream out(&block,QIODevice::WriteOnly);
    out.setVersion(QDataStream::Qt_5_0);
    out << imageHeader.headerId;
    out << imageHeader.frameCount;
    out << imageHeader.timeStamp;
    out << imageHeader.dataRate;

    if(m_imageType == us::PRESCAN_2D) { //send pre-scan image
      out << (int) m_preScanImage2d.getHeight() * m_preScanImage2d.getWidth(); //datalength in bytes
      out << (int) 8; //sample size in bits
      out << (int) 0;
      out << m_preScanImage2d.getHeight();
      out << m_preScanImage2d.getWidth();
      out << (double).0;//pixelWidth
      out << (double).0;//pixelHeight
      out << m_preScanImage2d.getTransmitFrequency();
      out << m_preScanImage2d.getSamplingFrequency();
      out << m_preScanImage2d.getTransducerRadius();
      out << m_preScanImage2d.getScanLinePitch();
      out << (int) m_preScanImage2d.getScanLineNumber();
      out << (int) (m_postScanImage2d.getDepth() / 1000.0); //int in mm
      out << (double) .0; //degPerFrame
      out << (int) 0;//framesPerVolume
      out << (double) .0;//motorRadius
      out << (int) 0; //motorType
      out.writeRawData((char*)m_preScanImage2d.bitmap,(int) m_preScanImage2d.getHeight() * m_preScanImage2d.getWidth());

      endOfSequence = (m_sequenceReaderPreScan.getFrameCount() == imageHeader.frameCount + 1);
    }
    else if(m_imageType == us::POSTSCAN_2D) { //send post-scan image
      out << (int) m_postScanImage2d.getHeight() * m_postScanImage2d.getWidth(); //datalength in bytes
      out << (int) 8; //sample size in bits
      out << (int) 1;
      out << m_postScanImage2d.getWidth();
      out << m_postScanImage2d.getHeight();
      out << m_postScanImage2d.getWidthResolution();//pixelWidth
      out << m_postScanImage2d.getHeightResolution();//pixelHeight
      out << m_postScanImage2d.getTransmitFrequency();
      out << m_postScanImage2d.getSamplingFrequency();
      out << m_postScanImage2d.getTransducerRadius();
      out << m_postScanImage2d.getScanLinePitch();
      out << (int) m_postScanImage2d.getScanLineNumber();
      out << (int) (m_postScanImage2d.getDepth() / 1000.0); //int in mm
      out << (double) .0; //degPerFrame
      out << (int) 0;//framesPerVolume
      out << (double) .0;//motorRadius
      out << (int) 0; //motorType
      out.writeRawData((char*)m_postScanImage2d.bitmap,(int) m_postScanImage2d.getHeight() * m_postScanImage2d.getWidth());

      endOfSequence = (m_sequenceReaderPostScan.getFrameCount() == imageHeader.frameCount + 1);
    }

    connectionSoc->write(block);
    qApp->processEvents();

    std::cout << "new frame sent, No " << imageHeader.frameCount << std::endl;

    imageHeader.frameCount ++;
  }
}

void usVirtualServer::sendingLoopSequenceMHD() {
  // TO DO
}

/**
* Method to invert rows and columns in the image.
*/
void usVirtualServer::invertRowsColsOnPreScan() {
  usImagePreScan2D<unsigned char> temp = m_preScanImage2d;
  m_preScanImage2d.resize(temp.getWidth(),temp.getHeight());

  for(unsigned int i=0; i<m_preScanImage2d.getHeight(); i++)
    for (unsigned int j=0; j<m_preScanImage2d.getWidth(); j++)
      m_preScanImage2d(i,j,temp(j,i));
}


void usVirtualServer::runAcquisition(bool run) {
  m_serverIsSendingImages = run;
  if(run) {
    emit(startSendingLoopSignal());
  }
}
