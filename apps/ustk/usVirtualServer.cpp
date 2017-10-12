#include "usVirtualServer.h"

usVirtualServer::usVirtualServer(std::string sequenceFileName, QObject *parent) : QObject(parent), m_tcpServer()
{
  imageHeader.frameCount = 0;
  //read sequence parameters
  setSequenceFileName(sequenceFileName); //opens first image of the sequence

  //init TCP server
  // set : acceptTheConnection() will be called whenever there is a new connection
  connect(&m_tcpServer, SIGNAL(newConnection()), this, SLOT(acceptTheConnection()));

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
  std::cout << "id received : " << id << std::endl;

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
    std::cout << "received header RUN/STOP = " << id << std::endl;
    bool run;
    in >> run;

    //TODO !
    if(run) {
      sendNewImage();
      /*probeInfo nfo;
      m_porta->getProbeInfo(nfo);
      if(initWithoutUpdate && nfo.motorized) {
        m_porta->setParam(prmMotorStatus, 0); // disables the motor
        m_porta->setMotorActive(false);
      }
      m_porta->runImage();*/
    }
    else {
      /*m_porta->stopImage();
      m_porta->setParam(prmMotorStatus, 0); // disables the motor
      m_porta->setMotorActive(false);*/
    }
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
}

//TODO !
/*
/// The callback provided to porta to be called when a new frame is acquired from the cine buffer.
bool portaCallback(void* param, unsigned char* addr, int blockIndex, int)
{
  std::cout << "new frame acquired" << std::endl;
  usVirtualServer* server = (usVirtualServer*)param;
  porta* portaInstance = server->getPorta();
  QTcpSocket * socket = server->getSocket();

  if(portaInstance->getCurrentMode() != BiplaneMode)
    imageHeader.frameCount ++;
  else if(imageHeader.frameCount ==0)
    imageHeader.frameCount ++;

  //manage motor offset at the beginning
  int motorStatus;
  portaInstance->getParam(prmMotorStatus,motorStatus);
  if(!server->motorOffsetSkipped && (motorStatus==1) && imageHeader.frameCount == 2) {
    imageHeader.frameCount = 0;
    server->motorOffsetSkipped=true;
  }

  //std::cout << "frame count = " << imageHeader.frameCount << std::endl;
  //std::cout << "image size = " << portaInstance->getFrameSize() << std::endl;
  unsigned char* beginImage;
  //filling image header

  quint64 oldTimeStamp = imageHeader.timeStamp; // for dataRate
  imageHeader.timeStamp =  QDateTime::currentMSecsSinceEpoch();

  imageHeader.dataRate = 1000.0 / (imageHeader.timeStamp - oldTimeStamp);

  if(imageHeader.imageType == 1) { //post scan
    imageHeader.dataLength = imageHeader.frameWidth * imageHeader.frameHeight;
    portaInstance->getBwImage(0,server->postScanImage,false);


    int mx, my;
    portaInstance->getMicronsPerPixel(0, mx, my);

    //bi-plande case
    if(portaInstance->getCurrentMode() == BiplaneMode) {
      portaInstance->getBwImage(1,server->secondBiplaneImage,false);
      std::cout << "getting 2nd image from bi plane" << std::endl;
    }
    imageHeader.pixelHeight = my / 1000000.0;
    imageHeader.pixelWidth = mx / 1000000.0;

  }
  else if (imageHeader.imageType == 0) { //pre scan
    imageHeader.dataLength = portaInstance->getFrameSize() - 4;
    beginImage = addr + 4;
    imageHeader.pixelHeight = 0;
    imageHeader.pixelWidth = 0;
  }
  else if (imageHeader.imageType == 2) { // RF
    int rfSize =  portaInstance->getParam(prmRfNumLines) * portaInstance->getParam(prmRfNumSamples);
    imageHeader.dataLength = rfSize * sizeof(short);
    beginImage = addr + 4;
    imageHeader.pixelHeight = 0;
    imageHeader.pixelWidth = 0;
  }

  //data contains a 4 bytes header before image data
  QByteArray block;
  std::cout << "writing header" << std::endl;
  QDataStream out(&block,QIODevice::WriteOnly);
  out.setVersion(QDataStream::Qt_5_0);
  out << imageHeader.headerId;
  out << imageHeader.frameCount;
  out << imageHeader.timeStamp;
  out << imageHeader.dataRate;
  out << imageHeader.dataLength;
  out << imageHeader.ss;
  out << imageHeader.imageType;
  out << imageHeader.frameWidth;
  out << imageHeader.frameHeight;
  out << imageHeader.pixelWidth;
  out << imageHeader.pixelHeight;
  out << imageHeader.transmitFrequency;
  out << imageHeader.samplingFrequency;
  out << imageHeader.transducerRadius;
  out << imageHeader.scanLinePitch;
  out << imageHeader.scanLineNumber;
  out << imageHeader.imageDepth;
  out << imageHeader.degPerFrame;
  out << imageHeader.framesPerVolume;
  out << imageHeader.motorRadius;
  out << imageHeader.motorType;
  std::cout << "writing image" << std::endl;


  if(imageHeader.imageType == 1) { //post scan
    out.writeRawData((char*)server->postScanImage,imageHeader.dataLength);
  }
  else if (imageHeader.imageType == 0) { // pre-scan
    out.writeRawData((char*)beginImage,imageHeader.dataLength);
  }
  else if (imageHeader.imageType == 2) { // RF
    out.writeRawData((char*)beginImage,imageHeader.dataLength);
  }

  if(server->motorOffsetSkipped && (motorStatus==1)) { // 3D but offset skipped
    socket->write(block);
  }
  else if(motorStatus==0) { // 2D
    socket->write(block);
  }

  // for bi-plane we send a second image
  if(portaInstance->getCurrentMode() == BiplaneMode && imageHeader.imageType == 1) {
    QByteArray block;
    std::cout << "writing 2nd header" << std::endl;
    QDataStream out(&block,QIODevice::WriteOnly);
    out.setVersion(QDataStream::Qt_5_0);
    out << imageHeader.headerId;
    out << imageHeader.frameCount;
    out << imageHeader.timeStamp;
    out << imageHeader.dataRate;
    out << imageHeader.dataLength;
    out << imageHeader.ss;
    out << imageHeader.imageType;
    out << imageHeader.frameWidth;
    out << imageHeader.frameHeight;
    out << imageHeader.pixelWidth;
    out << imageHeader.pixelHeight;
    out << imageHeader.transmitFrequency;
    out << imageHeader.samplingFrequency;
    out << imageHeader.transducerRadius;
    out << imageHeader.scanLinePitch;
    out << imageHeader.scanLineNumber;
    out << imageHeader.imageDepth;
    out << imageHeader.degPerFrame;
    out << imageHeader.framesPerVolume;
    out << imageHeader.motorRadius;
    out << imageHeader.motorType;
    std::cout << "writing 2nd image" << std::endl;
    out.writeRawData((char*)server->secondBiplaneImage,imageHeader.dataLength);
    imageHeader.frameCount ++;

    socket->write(block);
  }

  std::cout << "everything sent" << std::endl;


  std::cout << "headerId = " << imageHeader.headerId  << std::endl;
  std::cout << "frameCount = " << imageHeader.frameCount  << std::endl;
  std::cout << "timeStamp = " << imageHeader.timeStamp  << std::endl;
  std::cout << "dataLength = " << imageHeader.dataLength  << std::endl;
  std::cout << "ss = " << imageHeader.ss  << std::endl;
  std::cout << "imageType = " << imageHeader.imageType  << std::endl;
  std::cout << "frameWidth = " << imageHeader.frameWidth  << std::endl;
  std::cout << "frameHeight = " << imageHeader.frameHeight  << std::endl;
  std::cout << "pixelWidth = " << imageHeader.pixelWidth  << std::endl;
  std::cout << "pixelHeight = " << imageHeader.pixelHeight  << std::endl;
  std::cout << "transmitFrequency = " << imageHeader.transmitFrequency  << std::endl;
  std::cout << "samplingFrequency = " << imageHeader.samplingFrequency  << std::endl;
  std::cout << "transducerRadius = " << imageHeader.transducerRadius  << std::endl;
  std::cout << "scanLinePitch = " << imageHeader.scanLinePitch  << std::endl;
  std::cout << "scanLineNumber = " << imageHeader.scanLineNumber  << std::endl;
  std::cout << "degPerFrame = " << imageHeader.degPerFrame  << std::endl;
  std::cout << "framesPerVolume = " << imageHeader.framesPerVolume  << std::endl;
  std::cout << "motorRadius = " << imageHeader.motorRadius << std::endl;
  std::cout << "motorType = " << imageHeader.motorType << std::endl;

  std::cout << "steps = " << portaInstance->getParam(prmMotorSteps) << std::endl;
  return true;
}*/

/*
bool usVirtualServer::updatePorta(usUpdateHeaderIncomming header) {
  bool initImagingMode;
  // verify imaging mode
  if(m_currentImagingMode != (imagingMode)header.imagingMode) {
    // initialize new imaging mode
    initImagingMode = m_porta->initImagingMode((imagingMode)header.imagingMode);
    if(!initImagingMode) {
      std::cout << "error initializing imaging mode" << std::endl;
      return false;
    }
  }
  else
    initImagingMode = true;

  if(initImagingMode) {
    m_currentImagingMode = (imagingMode)header.imagingMode;
    m_porta->setRawDataCallback(portaCallback,(void*)this);
    //case of 3D
    probeInfo nfo;
    m_porta->getProbeInfo(nfo);
    // Motor settings
    if(nfo.motorized ) {
      //2D
      if(!header.activateMotor) {
        m_porta->goToPosition(header.motorPosition); // 0 = begin position
        m_porta->setParam(prmMotorStatus, 0);
        m_porta->setMotorActive(false);
      }
      //3D
      else {
        m_porta->setParam(prmMotorFrames,header.framesPerVolume);
        m_porta->setParam(prmMotorSteps,header.stepsPerFrame);
        m_porta->setParam(prmMotorStatus, 1);
        m_porta->setMotorActive(true);
      }
    }

    //RF - prescan
    if(!header.postScanMode) {
      if((imagingMode)header.imagingMode == BMode) {
        m_porta->setParam(prmRfMode, 0); //send only Bmode image

        URect rect;
        m_porta->getParam(prmBImageRect,rect);
        imageHeader.frameWidth = rect.right + 1 ;
        imageHeader.frameHeight = rect.bottom;
      }
      else if((imagingMode)header.imagingMode == RfMode) {
        imageHeader.frameWidth = m_porta->getParam(prmRfNumLines) ;
        imageHeader.frameHeight = m_porta->getParam(prmRfNumSamples);
      }
    }
    else { //post-scan
      m_porta->setDisplayDimensions(0,header.postScanWidth,header.postScanHeight);
      m_porta->getDisplayDimensions(0,imageHeader.frameWidth,imageHeader.frameHeight);
      postScanImage = (unsigned char*)malloc(imageHeader.frameWidth*imageHeader.frameHeight*sizeof(unsigned char));

      //bi-plane
      if((imagingMode)header.imagingMode == BiplaneMode) {
        m_porta->setDisplayDimensions(1,header.postScanWidth,header.postScanHeight);
        secondBiplaneImage = (unsigned char*)malloc(imageHeader.frameWidth*imageHeader.frameHeight*sizeof(unsigned char));
      }
    }
    // other acquisition parameters
    if(	m_porta->setParam(prmBImageDepth,header.imageDepth) &&
        m_porta->setParam(prmBSector,header.sector) &&
        m_porta->setParam(prmBTxFreq,header.transmitFrequency) &&
        m_porta->setParam(prmBSamplingFreq,header.samplingFrequency)) {

      int sector;
      m_porta->getParam(prmBSector, sector);
      imageHeader.scanLineNumber = m_porta->getParam(prmBLineDensity) * sector / 100;
      imageHeader.degPerFrame = m_porta->getParam(prmMotorSteps) * (double)(nfo.motorFov/1000.0) / (double)nfo.motorSteps;

      int fpv;
      m_porta->getParam(prmMotorFrames, fpv);
      imageHeader.framesPerVolume = fpv;

      imageHeader.transmitFrequency = m_porta->getParam(prmBTxFreq);
      imageHeader.samplingFrequency = m_porta->getParam(prmBSamplingFreq);

      imageHeader.imageDepth = m_porta->getParam(prmBImageDepth);

      if(!header.postScanMode) {
        if((imagingMode)header.imagingMode == BMode) {
          URect rect;
          m_porta->getParam(prmBImageRect,rect);
          imageHeader.frameWidth = rect.right + 1 ;
          imageHeader.frameHeight = rect.bottom;
        }
        else if((imagingMode)header.imagingMode == RfMode) {
          imageHeader.frameWidth = m_porta->getParam(prmRfNumLines) ;
          imageHeader.frameHeight = m_porta->getParam(prmRfNumSamples);
        }
      }

      if(header.postScanMode)
        imageHeader.imageType = 1;
      else if (header.imagingMode == BMode)
        imageHeader.imageType = 0;
      else if (header.imagingMode == RfMode)
        imageHeader.imageType = 2;

      imageHeader.ss = m_porta->getParam(prmBSampleSize) == 0 ? 8 : 16;

      confirmHeader.initOk = 1;
      return true;
    }
  }
  return false;
}*/

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
    transmitFrequency = m_preScanImage.getTransmitFrequency();
    transmitFrequencyMin = m_preScanImage.getTransmitFrequency();
    transmitFrequencyMax = m_preScanImage.getTransmitFrequency();

    samplingFrequency = m_preScanImage.getSamplingFrequency();
    samplingFrequencyMin = m_preScanImage.getSamplingFrequency();
    samplingFrequencyMax = m_preScanImage.getSamplingFrequency();

    imageDepth = m_preScanImage.getDepth();
    imageDepthMin = m_preScanImage.getDepth();
    imageDepthMax = m_preScanImage.getDepth();
  }
  else if (m_imageType == us::POSTSCAN_2D) {
    transmitFrequency = m_postScanImage.getTransmitFrequency();
    transmitFrequencyMin = m_postScanImage.getTransmitFrequency();
    transmitFrequencyMax = m_postScanImage.getTransmitFrequency();

    samplingFrequency = m_postScanImage.getSamplingFrequency();
    samplingFrequencyMin = m_postScanImage.getSamplingFrequency();
    samplingFrequencyMax = m_postScanImage.getSamplingFrequency();

    imageDepth = m_postScanImage.getDepth();
    imageDepthMin = m_postScanImage.getDepth();
    imageDepthMax = m_postScanImage.getDepth();

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

void usVirtualServer::setSequenceFileName(const std::string sequenceFileName) {
  m_sequenceReaderPostScan.setSequenceFileName(sequenceFileName);
  m_sequenceReaderPreScan.setSequenceFileName(sequenceFileName);

  //try to open post-scan sequence
  try {
    uint64_t localTimestamp;
    m_sequenceReaderPostScan.open(m_postScanImage,localTimestamp);
    imageHeader.timeStamp = localTimestamp;
    if(imageHeader.timeStamp == 0) { // timestamps are requested for virtual server
      throw(vpException(vpException::fatalError), "usVirtualServer error : no timestamp associated in sequence !");
    }
    m_imageType = us::POSTSCAN_2D;
  } catch(...) {
    //if we have an exception, it's not a post-scan image. So we try a pre-scan
    try {
      uint64_t localTimestamp;
      m_sequenceReaderPreScan.open(m_preScanImage,localTimestamp);
      imageHeader.timeStamp = localTimestamp;
      if(imageHeader.timeStamp == 0) { // timestamps are requested for virtual server
        throw(vpException(vpException::fatalError), "usVirtualServer error : no timestamp associated in sequence !");
      }
      invertRowsColsOnPreScan(); //to fit with ultrasonix grabbers (pre-scan image is inverted in porta SDK)
      m_imageType = us::PRESCAN_2D;
    }
    catch(...) {
      throw(vpException(vpException::badValue), "usVirtualServer error : trying to open a image sequence not managed (neither pre-scan 2D nor post-scan 2D)or with no timestamp associated");
    }
  }
}

void usVirtualServer::sendNewImage() {

  m_previousImageTimestamp = imageHeader.timeStamp;
  //manage first frame sent
  if(imageHeader.frameCount != 0) {
    if(m_imageType == us::PRESCAN_2D) {
      uint64_t localTimestamp;
      m_sequenceReaderPreScan.acquire(m_preScanImage,localTimestamp);
      invertRowsColsOnPreScan(); //to fit with ultrasonix grabbers (pre-scan image is inverted in porta SDK)
      imageHeader.timeStamp = localTimestamp;
      imageHeader.imageType = 0;
    }
    else if(m_imageType == us::POSTSCAN_2D) {
      uint64_t localTimestamp;
      m_sequenceReaderPostScan.acquire(m_postScanImage,localTimestamp);
      imageHeader.timeStamp = localTimestamp;
      imageHeader.imageType = 1;
    }
  }

  imageHeader.dataRate = 1000.0 / (imageHeader.timeStamp - m_previousImageTimestamp);

  //WAITING PROCESS TO UPDATE (to respect sequence timestamps)
  vpTime::wait(30);

  std::cout << "new frame acquired !" << std::endl;

  QByteArray block;
  QDataStream out(&block,QIODevice::WriteOnly);
  out.setVersion(QDataStream::Qt_5_0);
  out << imageHeader.headerId;
  out << imageHeader.frameCount;
  out << imageHeader.timeStamp;
  out << imageHeader.dataRate;

  if(m_imageType == us::PRESCAN_2D) { //send pre-scan image
    out << (int) m_preScanImage.getHeight() * m_preScanImage.getWidth(); //datalength in bytes
    out << (int) 8; //sample size in bits
    out << imageHeader.imageType;
    out << m_preScanImage.getHeight();
    out << m_preScanImage.getWidth();
    out << (double).0;//pixelWidth
    out << (double).0;//pixelHeight
    out << m_preScanImage.getTransmitFrequency();
    out << m_preScanImage.getSamplingFrequency();
    out << m_preScanImage.getTransducerRadius();
    out << m_preScanImage.getScanLinePitch();
    out << (int) m_preScanImage.getScanLineNumber();
    out << m_preScanImage.getDepth();
    out << (double) .0; //degPerFrame
    out << (int) 0;//framesPerVolume
    out << (double) .0;//motorRadius
    out << (int) 0; //motorType
    out.writeRawData((char*)m_preScanImage.bitmap,(int) m_preScanImage.getHeight() * m_preScanImage.getWidth());
  }
  else if(m_imageType == us::POSTSCAN_2D) { //send post-scan image
    out << (int) m_postScanImage.getHeight() * m_postScanImage.getWidth(); //datalength in bytes
    out << (int) 8; //sample size in bits
    out << imageHeader.imageType;
    out << m_postScanImage.getWidth();
    out << m_postScanImage.getHeight();
    out << m_postScanImage.getWidthResolution();//pixelWidth
    out << m_postScanImage.getHeightResolution();//pixelHeight
    out << m_postScanImage.getTransmitFrequency();
    out << m_postScanImage.getSamplingFrequency();
    out << m_postScanImage.getTransducerRadius();
    out << m_postScanImage.getScanLinePitch();
    out << (int) m_postScanImage.getScanLineNumber();
    out << m_postScanImage.getDepth();
    out << (double) .0; //degPerFrame
    out << (int) 0;//framesPerVolume
    out << (double) .0;//motorRadius
    out << (int) 0; //motorType
    out.writeRawData((char*)m_preScanImage.bitmap,(int) m_preScanImage.getHeight() * m_preScanImage.getWidth());
  }

  connectionSoc->write(block);

  imageHeader.frameCount ++;
}









/*
bool portaCallback(void* param, unsigned char* addr, int blockIndex, int)
{
  std::cout << "new frame acquired" << std::endl;
  usVirtualServer* server = (usVirtualServer*)param;
  porta* portaInstance = server->getPorta();
  QTcpSocket * socket = server->getSocket();

  if(portaInstance->getCurrentMode() != BiplaneMode)
    imageHeader.frameCount ++;
  else if(imageHeader.frameCount ==0)
    imageHeader.frameCount ++;

  //manage motor offset at the beginning
  int motorStatus;
  portaInstance->getParam(prmMotorStatus,motorStatus);
  if(!server->motorOffsetSkipped && (motorStatus==1) && imageHeader.frameCount == 2) {
    imageHeader.frameCount = 0;
    server->motorOffsetSkipped=true;
  }

  //std::cout << "frame count = " << imageHeader.frameCount << std::endl;
  //std::cout << "image size = " << portaInstance->getFrameSize() << std::endl;
  unsigned char* beginImage;
  //filling image header

  quint64 oldTimeStamp = imageHeader.timeStamp; // for dataRate
  imageHeader.timeStamp =  QDateTime::currentMSecsSinceEpoch();

  imageHeader.dataRate = 1000.0 / (imageHeader.timeStamp - oldTimeStamp);

  if(imageHeader.imageType == 1) { //post scan
    imageHeader.dataLength = imageHeader.frameWidth * imageHeader.frameHeight;
    portaInstance->getBwImage(0,server->postScanImage,false);


    int mx, my;
    portaInstance->getMicronsPerPixel(0, mx, my);

    //bi-plande case
    if(portaInstance->getCurrentMode() == BiplaneMode) {
      portaInstance->getBwImage(1,server->secondBiplaneImage,false);
      std::cout << "getting 2nd image from bi plane" << std::endl;
    }
    imageHeader.pixelHeight = my / 1000000.0;
    imageHeader.pixelWidth = mx / 1000000.0;

  }
  else if (imageHeader.imageType == 0) { //pre scan
    imageHeader.dataLength = portaInstance->getFrameSize() - 4;
    beginImage = addr + 4;
    imageHeader.pixelHeight = 0;
    imageHeader.pixelWidth = 0;
  }
  else if (imageHeader.imageType == 2) { // RF
    int rfSize =  portaInstance->getParam(prmRfNumLines) * portaInstance->getParam(prmRfNumSamples);
    imageHeader.dataLength = rfSize * sizeof(short);
    beginImage = addr + 4;
    imageHeader.pixelHeight = 0;
    imageHeader.pixelWidth = 0;
  }

  //data contains a 4 bytes header before image data
  QByteArray block;
  std::cout << "writing header" << std::endl;
  QDataStream out(&block,QIODevice::WriteOnly);
  out.setVersion(QDataStream::Qt_5_0);
  out << imageHeader.headerId;
  out << imageHeader.frameCount;
  out << imageHeader.timeStamp;
  out << imageHeader.dataRate;
  out << imageHeader.dataLength;
  out << imageHeader.ss;
  out << imageHeader.imageType;
  out << imageHeader.frameWidth;
  out << imageHeader.frameHeight;
  out << imageHeader.pixelWidth;
  out << imageHeader.pixelHeight;
  out << imageHeader.transmitFrequency;
  out << imageHeader.samplingFrequency;
  out << imageHeader.transducerRadius;
  out << imageHeader.scanLinePitch;
  out << imageHeader.scanLineNumber;
  out << imageHeader.imageDepth;
  out << imageHeader.degPerFrame;
  out << imageHeader.framesPerVolume;
  out << imageHeader.motorRadius;
  out << imageHeader.motorType;
  std::cout << "writing image" << std::endl;


  if(imageHeader.imageType == 1) { //post scan
    out.writeRawData((char*)server->postScanImage,imageHeader.dataLength);
  }
  else if (imageHeader.imageType == 0) { // pre-scan
    out.writeRawData((char*)beginImage,imageHeader.dataLength);
  }
  else if (imageHeader.imageType == 2) { // RF
    out.writeRawData((char*)beginImage,imageHeader.dataLength);
  }

  if(server->motorOffsetSkipped && (motorStatus==1)) { // 3D but offset skipped
    socket->write(block);
  }
  else if(motorStatus==0) { // 2D
    socket->write(block);
  }

  // for bi-plane we send a second image
  if(portaInstance->getCurrentMode() == BiplaneMode && imageHeader.imageType == 1) {
    QByteArray block;
    std::cout << "writing 2nd header" << std::endl;
    QDataStream out(&block,QIODevice::WriteOnly);
    out.setVersion(QDataStream::Qt_5_0);
    out << imageHeader.headerId;
    out << imageHeader.frameCount;
    out << imageHeader.timeStamp;
    out << imageHeader.dataRate;
    out << imageHeader.dataLength;
    out << imageHeader.ss;
    out << imageHeader.imageType;
    out << imageHeader.frameWidth;
    out << imageHeader.frameHeight;
    out << imageHeader.pixelWidth;
    out << imageHeader.pixelHeight;
    out << imageHeader.transmitFrequency;
    out << imageHeader.samplingFrequency;
    out << imageHeader.transducerRadius;
    out << imageHeader.scanLinePitch;
    out << imageHeader.scanLineNumber;
    out << imageHeader.imageDepth;
    out << imageHeader.degPerFrame;
    out << imageHeader.framesPerVolume;
    out << imageHeader.motorRadius;
    out << imageHeader.motorType;
    std::cout << "writing 2nd image" << std::endl;
    out.writeRawData((char*)server->secondBiplaneImage,imageHeader.dataLength);
    imageHeader.frameCount ++;

    socket->write(block);
  }

  std::cout << "everything sent" << std::endl;


  std::cout << "headerId = " << imageHeader.headerId  << std::endl;
  std::cout << "frameCount = " << imageHeader.frameCount  << std::endl;
  std::cout << "timeStamp = " << imageHeader.timeStamp  << std::endl;
  std::cout << "dataLength = " << imageHeader.dataLength  << std::endl;
  std::cout << "ss = " << imageHeader.ss  << std::endl;
  std::cout << "imageType = " << imageHeader.imageType  << std::endl;
  std::cout << "frameWidth = " << imageHeader.frameWidth  << std::endl;
  std::cout << "frameHeight = " << imageHeader.frameHeight  << std::endl;
  std::cout << "pixelWidth = " << imageHeader.pixelWidth  << std::endl;
  std::cout << "pixelHeight = " << imageHeader.pixelHeight  << std::endl;
  std::cout << "transmitFrequency = " << imageHeader.transmitFrequency  << std::endl;
  std::cout << "samplingFrequency = " << imageHeader.samplingFrequency  << std::endl;
  std::cout << "transducerRadius = " << imageHeader.transducerRadius  << std::endl;
  std::cout << "scanLinePitch = " << imageHeader.scanLinePitch  << std::endl;
  std::cout << "scanLineNumber = " << imageHeader.scanLineNumber  << std::endl;
  std::cout << "degPerFrame = " << imageHeader.degPerFrame  << std::endl;
  std::cout << "framesPerVolume = " << imageHeader.framesPerVolume  << std::endl;
  std::cout << "motorRadius = " << imageHeader.motorRadius << std::endl;
  std::cout << "motorType = " << imageHeader.motorType << std::endl;

  std::cout << "steps = " << portaInstance->getParam(prmMotorSteps) << std::endl;
  return true;
}*/

/**
* Method to invert rows and columns in the image.
*/
void usVirtualServer::invertRowsColsOnPreScan() {
  usImagePreScan2D<unsigned char> temp = m_preScanImage;
  m_preScanImage.resize(temp.getWidth(),temp.getHeight());

  for(unsigned int i=0; i<m_preScanImage.getHeight(); i++)
    for (unsigned int j=0; j<m_preScanImage.getWidth(); j++)
      m_preScanImage(i,j,temp(j,i));
}
