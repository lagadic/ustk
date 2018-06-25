#include "usNetworkServer.h"

usNetworkServer::usNetworkServer(QObject *parent) : QObject(parent)
{

  // init porta hardware

  // creating porta object
  m_porta = new porta;
  std::cout << "porta instance created" << std::endl;

  // porta settings
  char *portaFirmware = USTK_PORTA_FIRMWARE_PATH;
  char *portaSettings = USTK_PORTA_SETTINGS_PATH;
  char *portaLicense = USTK_PORTA_LICENCE_PATH;
  char *portaLut = "";

  int usm = 3; // us module version 3 (MDP/TOUCH)
  int pci = 3;
  int highVoltage = 0; // true only if version 2

  // init porta
  bool initPortaSucess = m_porta->init(256 * 1024 * 1024, portaFirmware, portaSettings, portaLicense, portaLut, usm,
                                       pci, highVoltage, 0, 64);
  if (!initPortaSucess) {
    std::cout << "error initializing porta sdk !" << std::endl;
  }

  // init TCP server
  // set : acceptTheConnection() will be called whenever there is a new connection
  connect(&tcpServer, SIGNAL(newConnection()), this, SLOT(acceptTheConnection()));

  // Start listening on port 8080
  QString portNum = QString::number(8080);
  bool status = tcpServer.listen(QHostAddress::Any, portNum.toUShort());

  // Check, if the server did start correctly or not
  if (status == true) {
    std::cout << "TCP server Started\nServer now listening on port# " << portNum.toStdString() << std::endl;
  } else {
    std::cout << "TCP server start failure" << tcpServer.errorString().toStdString() << std::endl;
  }

  connect(this, SIGNAL(writeOnSocketSignal()), this, SLOT(writeOnSocketSlot()));
}

usNetworkServer::~usNetworkServer() {}

// This function is called whenever we have an incoming connection
void usNetworkServer::acceptTheConnection()
{

  // Accept and establish the connection. Note that, the data-transfer happens with `connectionSoc` and not with
  // `tcpServer`
  // `tcpServer` only waits and listens to new connections
  connectionSoc = tcpServer.nextPendingConnection();

  // Set : readIncomingData() will be called whenever the data (coming from client) is available
  connect(connectionSoc, SIGNAL(readyRead()), this, SLOT(readIncomingData()));

  // Set : connectionAboutToClose() will be called whenever the connection is close by the client
  connect(connectionSoc, SIGNAL(disconnected()), this, SLOT(connectionAboutToClose()));
}

// Will be called whenever the data (coming from client) is available
void usNetworkServer::readIncomingData()
{
  // headers possible to be received
  usNetworkServer::usInitHeaderIncomming headerInit;
  usNetworkServer::usUpdateHeaderIncomming headerUpdate;

  // reading part

  // prepare reading in QDataStream
  QDataStream in;
  in.setDevice(connectionSoc);
  in.setVersion(QDataStream::Qt_5_0);

  // read header id
  int id;
  in >> id;
  std::cout << "id received : " << id << std::endl;

  if (id == 1) { // init header

    if (m_porta->isImaging()) // case of init received while an acquisition is already runnig
      m_porta->stopImage();

    initWithoutUpdate = true;
    in >> headerInit.probeId;
    in >> headerInit.slotId;
    in >> headerInit.imagingMode;

    initPorta(headerInit);

    // send back default params
    QByteArray block;
    QDataStream out(&block, QIODevice::WriteOnly);
    out.setVersion(QDataStream::Qt_5_0);
    out << confirmHeader.headerId;
    out << confirmHeader.initOk;
    out << confirmHeader.probeId;

    writeInitAcquisitionParameters(out, headerInit.imagingMode, headerInit.probeId);

    connectionSoc->write(block);
  } else if (id == 2) { // update header

    if (m_porta->isImaging()) // case of update received while an acquisition is already runnig
      m_porta->stopImage();

    initWithoutUpdate = false;

    in >> headerUpdate.transmitFrequency;
    in >> headerUpdate.samplingFrequency;
    in >> headerUpdate.imagingMode;
    in >> headerUpdate.postScanMode;
    in >> headerUpdate.postScanHeight;
    in >> headerUpdate.postScanWidth;
    in >> headerUpdate.imageDepth;
    in >> headerUpdate.sector;
    in >> headerUpdate.activateMotor;
    in >> headerUpdate.motorPosition;
    in >> headerUpdate.framesPerVolume;
    in >> headerUpdate.stepsPerFrame;

    std::cout << "received header UPDATE = " << id << std::endl;
    std::cout << "transmitFrequency = " << headerUpdate.transmitFrequency << std::endl;
    std::cout << "samplingFrequency = " << headerUpdate.samplingFrequency << std::endl;
    std::cout << "imagingMode = " << headerUpdate.imagingMode << std::endl;
    std::cout << "postScanMode = " << headerUpdate.postScanMode << std::endl;
    std::cout << "postScanHeigh = " << headerUpdate.postScanHeight << std::endl;
    std::cout << "postScanWidtht = " << headerUpdate.postScanWidth << std::endl;
    std::cout << "imageDepth = " << headerUpdate.imageDepth << std::endl;
    std::cout << "sector = " << headerUpdate.sector << std::endl;
    std::cout << "activateMotor = " << headerUpdate.activateMotor << std::endl;
    std::cout << "motorPosition = " << headerUpdate.motorPosition << std::endl;
    std::cout << "framesPerVolume = " << headerUpdate.framesPerVolume << std::endl;
    std::cout << "degreesPerFrame = " << headerUpdate.stepsPerFrame << std::endl;

    if (updatePorta(headerUpdate)) { // sucess
      // send back default params
      QByteArray block;
      QDataStream out(&block, QIODevice::WriteOnly);
      out.setVersion(QDataStream::Qt_5_0);
      out << confirmHeader.headerId;
      out << confirmHeader.initOk;
      out << confirmHeader.probeId;

      writeUpdateAcquisitionParameters(out, headerUpdate, confirmHeader.probeId);
      std::cout << "bytes written to confirm update" << connectionSoc->write(block) << std::endl;
    }
  } else if (id == 3) { // run - stop command
    std::cout << "received header RUN/STOP = " << id << std::endl;
    bool run;
    in >> run;

    std::cout << "command run = " << run << std::endl;

    if (run) {
      probeInfo nfo;
      m_porta->getProbeInfo(nfo);
      if (initWithoutUpdate && nfo.motorized) {
#if USTK_PORTA_VERSION_MAJOR < 6
        m_porta->setParam(prmMotorStatus, 0); // disables the motor
#endif
        m_porta->setMotorActive(false);
      }
      if (!m_porta->runImage()) {
        std::cout << "porta run error" << std::endl;
      } else {
        std::cout << "porta run OK" << std::endl;
      }
    } else {
      m_porta->stopImage();
#if USTK_PORTA_VERSION_MAJOR < 6
      m_porta->setParam(prmMotorStatus, 0); // disables the motor
#endif
      m_porta->setMotorActive(false);
    }
  } else {
    std::cout << "ERROR : unknown data received !" << std::endl;
  }
}

// Will be called whenever the connection is close by the client
void usNetworkServer::connectionAboutToClose()
{
  // Set this text into the label
  std::cout << "Connection to client closed" << std::endl;

  if (m_porta->isImaging())
    m_porta->stopImage();
  // Close the connection (Say bye)
  connectionSoc->close();
}

/// The callback provided to porta to be called when a new frame is acquired from the cine buffer.
#if USTK_PORTA_VERSION_MAJOR > 5 // used only for RF in sdk version >= 6
int portaCallback(void *param, unsigned char *addr, int blockIndex, int)
#else
bool portaCallback(void *param, unsigned char *addr, int blockIndex, int)
#endif
{
  std::cout << "new frame acquired" << std::endl;
  usNetworkServer *server = (usNetworkServer *)param;
  porta *portaInstance = server->getPorta();
  QTcpSocket *socket = server->getSocket();

  if (portaInstance->getCurrentMode() != BiplaneMode)
    server->imageHeader.frameCount++;
  else if (server->imageHeader.frameCount == 0)
    server->imageHeader.frameCount++;

  // manage motor offset at the beginning
  int motorStatus;
  portaInstance->getParam(prmMotorStatus, motorStatus);
  if (!server->motorOffsetSkipped && (motorStatus == 1) && server->imageHeader.frameCount == 2) {
    server->imageHeader.frameCount = 0;
    server->motorOffsetSkipped = true;
  }

  std::cout << "frame count = " << server->imageHeader.frameCount << std::endl;
  unsigned char *beginImage;
  // filling image header

  quint64 oldTimeStamp = server->imageHeader.timeStamp; // for dataRate
  server->imageHeader.timeStamp = QDateTime::currentMSecsSinceEpoch();

  server->imageHeader.dataRate = 1000.0 / (server->imageHeader.timeStamp - oldTimeStamp);

  if (server->imageHeader.imageType == 1) { // post scan
    server->imageHeader.dataLength = server->imageHeader.frameWidth * server->imageHeader.frameHeight;
    portaInstance->getBwImage(0, server->postScanImage, false);

    int mx, my;
    portaInstance->getMicronsPerPixel(0, mx, my);

    // bi-plande case
    if (portaInstance->getCurrentMode() == BiplaneMode) {
      portaInstance->getBwImage(1, server->secondBiplaneImage, false);
      std::cout << "getting 2nd image from bi plane" << std::endl;
    }
    server->imageHeader.pixelHeight = my / 1000000.0;
    server->imageHeader.pixelWidth = mx / 1000000.0;

  }
  // data contains a 4 bytes header before image data
  else if (server->imageHeader.imageType == 0) { // pre scan
    server->imageHeader.dataLength = portaInstance->getParam(prmBNumLines) * portaInstance->getParam(prmBNumSamples);
    beginImage = addr + 4;
    server->imageHeader.pixelHeight = 0;
    server->imageHeader.pixelWidth = 0;
  } else if (server->imageHeader.imageType == 2) { // RF
    int rfSize = portaInstance->getParam(prmRfNumLines) * portaInstance->getParam(prmRfNumSamples);
    server->imageHeader.dataLength = rfSize * sizeof(short);
    beginImage = addr + 4;
    server->imageHeader.pixelHeight = 0;
    server->imageHeader.pixelWidth = 0;
  }
  /*
  QByteArray block;
  std::cout << "writing header on socket" << std::endl;
  QDataStream out(&block, QIODevice::WriteOnly);
  out.setVersion(QDataStream::Qt_5_0);
  out << server->imageHeader.headerId;
  out << server->imageHeader.frameCount;
  out << server->imageHeader.timeStamp;
  out << server->imageHeader.dataRate;
  out << server->imageHeader.dataLength;
  out << server->imageHeader.ss;
  out << server->imageHeader.imageType;
  out << server->imageHeader.frameWidth;
  out << server->imageHeader.frameHeight;
  out << server->imageHeader.pixelWidth;
  out << server->imageHeader.pixelHeight;
  out << server->imageHeader.transmitFrequency;
  out << server->imageHeader.samplingFrequency;
  out << server->imageHeader.transducerRadius;
  out << server->imageHeader.scanLinePitch;
  out << server->imageHeader.scanLineNumber;
  out << server->imageHeader.imageDepth;
  out << server->imageHeader.degPerFrame;
  out << server->imageHeader.framesPerVolume;
  out << server->imageHeader.motorRadius;
  out << server->imageHeader.motorType;
  std::cout << "writing image on socket" << std::endl;

  if (server->imageHeader.imageType == 1) { // post scan
    out.writeRawData((char *)server->postScanImage, server->imageHeader.dataLength);
  } else if (server->imageHeader.imageType == 0) { // pre-scan
    out.writeRawData((char *)beginImage, server->imageHeader.dataLength);
  } else if (server->imageHeader.imageType == 2) { // RF
    out.writeRawData((char *)beginImage, server->imageHeader.dataLength);
  }

  quint64 dataWrittenSize=0;

  if (server->motorOffsetSkipped && (motorStatus == 1)) { // 3D but offset skipped
    dataWrittenSize = socket->write(block);
  } else if (motorStatus == 0) { // 2D
    dataWrittenSize = socket->write(block);
  }

  std::cout << "written data size = " << dataWrittenSize << std::endl;

  // for bi-plane we send a second image
  if (portaInstance->getCurrentMode() == BiplaneMode && server->imageHeader.imageType == 1) {
    QByteArray block;
    std::cout << "writing 2nd header" << std::endl;
    QDataStream out(&block, QIODevice::WriteOnly);
    out.setVersion(QDataStream::Qt_5_0);
    out << server->imageHeader.headerId;
    out << server->imageHeader.frameCount;
    out << server->imageHeader.timeStamp;
    out << server->imageHeader.dataRate;
    out << server->imageHeader.dataLength;
    out << server->imageHeader.ss;
    out << server->imageHeader.imageType;
    out << server->imageHeader.frameWidth;
    out << server->imageHeader.frameHeight;
    out << server->imageHeader.pixelWidth;
    out << server->imageHeader.pixelHeight;
    out << server->imageHeader.transmitFrequency;
    out << server->imageHeader.samplingFrequency;
    out << server->imageHeader.transducerRadius;
    out << server->imageHeader.scanLinePitch;
    out << server->imageHeader.scanLineNumber;
    out << server->imageHeader.imageDepth;
    out << server->imageHeader.degPerFrame;
    out << server->imageHeader.framesPerVolume;
    out << server->imageHeader.motorRadius;
    out << server->imageHeader.motorType;
    std::cout << "writing 2nd image" << std::endl;
    out.writeRawData((char *)server->secondBiplaneImage, server->imageHeader.dataLength);
    server->imageHeader.frameCount++;

    socket->write(block);
  }
  */

  server->beginImage = (char *)beginImage;
  server->motorStatus = motorStatus;
  server->biPlane = portaInstance->getCurrentMode() == BiplaneMode;

  server->writeOnSocketFromOtherThread();

  /*std::cout << "headerId = " << server->imageHeader.headerId  << std::endl;
  std::cout << "frameCount = " << server->imageHeader.frameCount  << std::endl;
  std::cout << "timeStamp = " << server->imageHeader.timeStamp  << std::endl;
  std::cout << "dataLength = " << server->imageHeader.dataLength  << std::endl;
  std::cout << "ss = " << server->imageHeader.ss  << std::endl;
  std::cout << "imageType = " << server->imageHeader.imageType  << std::endl;
  std::cout << "frameWidth = " << server->imageHeader.frameWidth  << std::endl;
  std::cout << "frameHeight = " << server->imageHeader.frameHeight  << std::endl;
  std::cout << "pixelWidth = " << server->imageHeader.pixelWidth  << std::endl;
  std::cout << "pixelHeight = " << server->imageHeader.pixelHeight  << std::endl;
  std::cout << "transmitFrequency = " << server->imageHeader.transmitFrequency  << std::endl;
  std::cout << "samplingFrequency = " << server->imageHeader.samplingFrequency  << std::endl;
  std::cout << "transducerRadius = " << server->imageHeader.transducerRadius  << std::endl;
  std::cout << "scanLinePitch = " << server->imageHeader.scanLinePitch  << std::endl;
  std::cout << "scanLineNumber = " << server->imageHeader.scanLineNumber  << std::endl;
  std::cout << "degPerFrame = " << server->imageHeader.degPerFrame  << std::endl;
  std::cout << "framesPerVolume = " << server->imageHeader.framesPerVolume  << std::endl;
  std::cout << "motorRadius = " << server->imageHeader.motorRadius << std::endl;
  std::cout << "motorType = " << server->imageHeader.motorType << std::endl;

  std::cout << "steps = " << portaInstance->getParam(prmMotorSteps) << std::endl;*/
  return true;
}

/// The callback provided to porta (versions 6+) for B mode images
#if USTK_PORTA_VERSION_MAJOR > 5
int portaCallbackImage(void *param, int id, int)
{
  std::cout << "new frame acquired" << std::endl;
  usNetworkServer *server = (usNetworkServer *)param;
  porta *portaInstance = server->getPorta();
  QTcpSocket *socket = server->getSocket();

  if (portaInstance->getCurrentMode() != BiplaneMode)
    server->imageHeader.frameCount++;
  else if (server->imageHeader.frameCount == 0)
    server->imageHeader.frameCount++;

  // manage motor offset at the beginning
  int motorStatus;
  portaInstance->getParam(prmMotorStatus, motorStatus);
  if (!server->motorOffsetSkipped && (motorStatus == 1) && server->imageHeader.frameCount == 2) {
    server->imageHeader.frameCount = 0;
    server->motorOffsetSkipped = true;
  }

  // std::cout << "frame count = " << server->imageHeader.frameCount << std::endl;
  // std::cout << "image size = " << portaInstance->getFrameSize() << std::endl;
  unsigned char *beginImage;
  // filling image header

  quint64 oldTimeStamp = server->imageHeader.timeStamp; // for dataRate
  server->imageHeader.timeStamp = QDateTime::currentMSecsSinceEpoch();

  server->imageHeader.dataRate = 1000.0 / (server->imageHeader.timeStamp - oldTimeStamp);

  if (server->imageHeader.imageType == 1) { // post scan
    server->imageHeader.dataLength = server->imageHeader.frameWidth * server->imageHeader.frameHeight;
    portaInstance->getBwImage(0, server->postScanImage, false);

    int mx, my;
    portaInstance->getMicronsPerPixel(0, mx, my);

    // bi-plande case
    if (portaInstance->getCurrentMode() == BiplaneMode) {
      portaInstance->getBwImage(1, server->secondBiplaneImage, false);
      std::cout << "getting 2nd image from bi plane" << std::endl;
    }
    server->imageHeader.pixelHeight = my / 1000000.0;
    server->imageHeader.pixelWidth = mx / 1000000.0;

  } else if (server->imageHeader.imageType == 0) { // pre scan
    server->imageHeader.dataLength = portaInstance->getParam(prmBNumLines) * portaInstance->getParam(prmBNumSamples);

    beginImage = new unsigned char[portaInstance->getParam(prmBNumLines) * portaInstance->getParam(prmBNumSamples) *
                                   sizeof(unsigned char)];
    portaGetBwImagePrescan(0, beginImage);
    server->imageHeader.pixelHeight = 0;
    server->imageHeader.pixelWidth = 0;
  }

  QByteArray block;
  std::cout << "writing header" << std::endl;
  QDataStream out(&block, QIODevice::WriteOnly);
  out.setVersion(QDataStream::Qt_5_0);
  out << server->imageHeader.headerId;
  out << server->imageHeader.frameCount;
  out << server->imageHeader.timeStamp;
  out << server->imageHeader.dataRate;
  out << server->imageHeader.dataLength;
  out << server->imageHeader.ss;
  out << server->imageHeader.imageType;
  out << server->imageHeader.frameWidth;
  out << server->imageHeader.frameHeight;
  out << server->imageHeader.pixelWidth;
  out << server->imageHeader.pixelHeight;
  out << server->imageHeader.transmitFrequency;
  out << server->imageHeader.samplingFrequency;
  out << server->imageHeader.transducerRadius;
  out << server->imageHeader.scanLinePitch;
  out << server->imageHeader.scanLineNumber;
  out << server->imageHeader.imageDepth;
  out << server->imageHeader.degPerFrame;
  out << server->imageHeader.framesPerVolume;
  out << server->imageHeader.motorRadius;
  out << server->imageHeader.motorType;
  std::cout << "writing image" << std::endl;

  if (server->imageHeader.imageType == 1) { // post scan
    out.writeRawData((char *)server->postScanImage, server->imageHeader.dataLength);
  } else if (server->imageHeader.imageType == 0) { // pre-scan
    out.writeRawData((char *)beginImage, server->imageHeader.dataLength);
  }

  if (server->motorOffsetSkipped && (motorStatus == 1)) { // 3D but offset skipped
    socket->write(block);
  } else if (motorStatus == 0) { // 2D
    socket->write(block);
  }

  // for bi-plane we send a second image
  if (portaInstance->getCurrentMode() == BiplaneMode && server->imageHeader.imageType == 1) {
    QByteArray block;
    std::cout << "writing 2nd header" << std::endl;
    QDataStream out(&block, QIODevice::WriteOnly);
    out.setVersion(QDataStream::Qt_5_0);
    out << server->imageHeader.headerId;
    out << server->imageHeader.frameCount;
    out << server->imageHeader.timeStamp;
    out << server->imageHeader.dataRate;
    out << server->imageHeader.dataLength;
    out << server->imageHeader.ss;
    out << server->imageHeader.imageType;
    out << server->imageHeader.frameWidth;
    out << server->imageHeader.frameHeight;
    out << server->imageHeader.pixelWidth;
    out << server->imageHeader.pixelHeight;
    out << server->imageHeader.transmitFrequency;
    out << server->imageHeader.samplingFrequency;
    out << server->imageHeader.transducerRadius;
    out << server->imageHeader.scanLinePitch;
    out << server->imageHeader.scanLineNumber;
    out << server->imageHeader.imageDepth;
    out << server->imageHeader.degPerFrame;
    out << server->imageHeader.framesPerVolume;
    out << server->imageHeader.motorRadius;
    out << server->imageHeader.motorType;
    std::cout << "writing 2nd image" << std::endl;
    out.writeRawData((char *)server->secondBiplaneImage, server->imageHeader.dataLength);
    server->imageHeader.frameCount++;

    socket->write(block);
  }

  std::cout << "everything sent" << std::endl;

  /*std::cout << "headerId = " << server->imageHeader.headerId  << std::endl;
  std::cout << "frameCount = " << server->imageHeader.frameCount  << std::endl;
  std::cout << "timeStamp = " << server->imageHeader.timeStamp  << std::endl;
  std::cout << "dataLength = " << server->imageHeader.dataLength  << std::endl;
  std::cout << "ss = " << server->imageHeader.ss  << std::endl;
  std::cout << "imageType = " << server->imageHeader.imageType  << std::endl;
  std::cout << "frameWidth = " << server->imageHeader.frameWidth  << std::endl;
  std::cout << "frameHeight = " << server->imageHeader.frameHeight  << std::endl;
  std::cout << "pixelWidth = " << server->imageHeader.pixelWidth  << std::endl;
  std::cout << "pixelHeight = " << server->imageHeader.pixelHeight  << std::endl;
  std::cout << "transmitFrequency = " << server->imageHeader.transmitFrequency  << std::endl;
  std::cout << "samplingFrequency = " << server->imageHeader.samplingFrequency  << std::endl;
  std::cout << "transducerRadius = " << server->imageHeader.transducerRadius  << std::endl;
  std::cout << "scanLinePitch = " << server->imageHeader.scanLinePitch  << std::endl;
  std::cout << "scanLineNumber = " << server->imageHeader.scanLineNumber  << std::endl;
  std::cout << "degPerFrame = " << server->imageHeader.degPerFrame  << std::endl;
  std::cout << "framesPerVolume = " << server->imageHeader.framesPerVolume  << std::endl;
  std::cout << "motorRadius = " << server->imageHeader.motorRadius << std::endl;
  std::cout << "motorType = " << server->imageHeader.motorType << std::endl;

  std::cout << "steps = " << portaInstance->getParam(prmMotorSteps) << std::endl;*/
  return true;
}
#endif

// porta init
void usNetworkServer::initPorta(usNetworkServer::usInitHeaderIncomming header)
{
  // prepare return value
  confirmHeader.initOk = 0;

  // check probe
  char name[1024];
  int code;
  probeInfo nfo;

  if (m_porta->isConnected()) {
    code = m_porta->getProbeID(header.slotId);
    std::cout << "probe Id " << code << std::endl;
    confirmHeader.probeId = code;
    // select the code read
    if (m_porta->selectProbe(code) && m_porta->getProbeInfo(nfo)) {
      // Select slot passed by the header
      if (m_porta->activateProbeConnector(header.slotId)) {

        if (m_porta->getProbeName(name, 1024, code)) {
          std::cout << "detected probe : " << name << std::endl;
        }
        QString settingsPath = QString(USTK_PORTA_SETTINGS_PATH) + QString("presets/imaging/");
        settingsPath += getProbeSettingsFromId(code);
        m_porta->loadPreset(settingsPath.toStdString().c_str());
#if USTK_PORTA_VERSION_MAJOR > 5
        int mode = header.imagingMode;
        if (mode == 12) // RF
          mode = BMode;
        if (!m_porta->initImagingMode((imagingMode)mode)) { // in 6.x SDK, there is no more RF mode
#else
        if (!m_porta->initImagingMode((imagingMode)header.imagingMode)) {
#endif
          std::cout << "error initializing imaging mode" << std::endl;
        } else {
          if (code == header.probeId) {

#if USTK_PORTA_VERSION_MAJOR > 5
            if (header.imagingMode == RfMode)
              m_porta->setRawDataCallback(portaCallback, (void *)this);
            else
              m_porta->setDisplayCallback(0, portaCallbackImage, (void *)this);

#else
            m_porta->setRawDataCallback(portaCallback, (void *)this);
#endif
            // case of 3D, we disable motor by default and set it to middle frame
            if (nfo.motorized) {
              m_porta->goToPosition(0); // 0 = begin position
#if USTK_PORTA_VERSION_MAJOR <= 5
              m_porta->setParam(prmMotorStatus, 0);
#endif
              m_porta->setMotorActive(false);
              imageHeader.motorRadius = nfo.motorRadius / 1000000.0; // from microns to meters
              if (header.probeId == 15)                              // 4DC7
                imageHeader.motorType = 1;                           // see usMotorType
              else
                imageHeader.motorType = 0;
            }
            m_currentImagingMode = (imagingMode)header.imagingMode;
            confirmHeader.initOk = 1;
          }
        }
      }
    }
  }

// must set it to 128 for 4DC7 and C5-2 probes
#if USTK_PORTA_VERSION_MAJOR <= 5
  m_porta->setParam(prmBLineDensity, 128);
#endif

  imageHeader.frameCount = 0;
  imageHeader.transducerRadius = nfo.radius / 1000000.0;                              // from microns to  meters
  imageHeader.scanLinePitch = (nfo.pitch / 1000000.0) / imageHeader.transducerRadius; // in meters

  int sector;
  m_porta->getParam(prmBSector, sector);
  imageHeader.scanLineNumber = m_porta->getParam(prmBLineDensity) * sector / 100;
  imageHeader.degPerFrame = m_porta->getParam(prmMotorSteps) * (double)(nfo.motorFov / 1000.0) / (double)nfo.motorSteps;
  imageHeader.motorRadius = nfo.motorRadius / 1000000.0; // from microns to meters
  if (code == 15)
    imageHeader.motorType = 1;
  else
    imageHeader.motorType = 0;

  int fpv;
  m_porta->getParam(prmMotorFrames, fpv);
  imageHeader.framesPerVolume = fpv;

  imageHeader.transmitFrequency = m_porta->getParam(prmBTxFreq);
  imageHeader.samplingFrequency = m_porta->getParam(prmBSamplingFreq);

  imageHeader.imageDepth = m_porta->getParam(prmBImageDepth);

/// The callback provided to porta to be called when a new frame is acquired from the cine buffer.
#if USTK_PORTA_VERSION_MAJOR > 5
  portaRect rect;
#else
  URect rect;
#endif
  m_porta->getParam(prmBImageRect, rect);

  if (header.imagingMode == 12) { // RF
    imageHeader.imageType = 2;
    imageHeader.frameWidth = m_porta->getParam(prmRfNumLines);
    imageHeader.frameHeight = m_porta->getParam(prmRfNumSamples);
#if USTK_PORTA_VERSION_MAJOR <= 5
    m_porta->setParam(prmRfMode, 1);
#endif
    imageHeader.ss = 16;
  } else {
    imageHeader.imageType = 0;
    imageHeader.frameWidth = rect.right + 1;
    imageHeader.frameHeight = rect.bottom;
    imageHeader.ss = 8;
#if USTK_PORTA_VERSION_MAJOR > 5
    m_porta->setDisplayDimensions(0, rect.right, rect.bottom);
#endif
  }

  motorOffsetSkipped = false;
  std::cout << "end init porta" << std::endl;
}

bool usNetworkServer::updatePorta(usUpdateHeaderIncomming header)
{
  bool initImagingMode;
  // verify imaging mode
  if (m_currentImagingMode != (imagingMode)header.imagingMode) {
    // initialize new imaging mode
    initImagingMode = m_porta->initImagingMode((imagingMode)header.imagingMode);
    if (!initImagingMode) {
      std::cout << "error initializing imaging mode" << std::endl;
      return false;
    }
  } else
    initImagingMode = true;

  if (initImagingMode) {
    m_currentImagingMode = (imagingMode)header.imagingMode;
    m_porta->setRawDataCallback(portaCallback, (void *)this);
    // case of 3D
    probeInfo nfo;
    m_porta->getProbeInfo(nfo);
    // Motor settings
    if (nfo.motorized) {
      // 2D
      if (!header.activateMotor) {
        m_porta->setMotorActive(true);
        m_porta->goToPosition(header.motorPosition); // 0 = begin position
        m_porta->setParam(prmMotorStatus, 0);
        m_porta->setMotorActive(false);
      }
      // 3D
      else {
        m_porta->setParam(prmMotorFrames, header.framesPerVolume);
        m_porta->setParam(prmMotorSteps, header.stepsPerFrame);
        m_porta->setParam(prmMotorStatus, 1);
        m_porta->setMotorActive(true);
      }
    }

    // RF - prescan
    if (!header.postScanMode) {
      if ((imagingMode)header.imagingMode == BMode) {
#if USTK_PORTA_VERSION_MAJOR > 5
        portaRect rect;
#else
        m_porta->setParam(prmRfMode, 0); // send only Bmode image
        URect rect;
#endif
        m_porta->getParam(prmBImageRect, rect);
        imageHeader.frameWidth = rect.right + 1;
        imageHeader.frameHeight = rect.bottom;
      } else if ((imagingMode)header.imagingMode == RfMode) {
        imageHeader.frameWidth = m_porta->getParam(prmRfNumLines);
        imageHeader.frameHeight = m_porta->getParam(prmRfNumSamples);
      }
    } else { // post-scan
      m_porta->setDisplayDimensions(0, header.postScanWidth, header.postScanHeight);
      m_porta->getDisplayDimensions(0, imageHeader.frameWidth, imageHeader.frameHeight);
      postScanImage = (unsigned char *)malloc(imageHeader.frameWidth * imageHeader.frameHeight * sizeof(unsigned char));

      // bi-plane
      if ((imagingMode)header.imagingMode == BiplaneMode) {
        m_porta->setDisplayDimensions(1, header.postScanWidth, header.postScanHeight);
        secondBiplaneImage =
            (unsigned char *)malloc(imageHeader.frameWidth * imageHeader.frameHeight * sizeof(unsigned char));
      }
    }
    // other acquisition parameters
    if (m_porta->setParam(prmBImageDepth, header.imageDepth) && m_porta->setParam(prmBSector, header.sector) &&
        m_porta->setParam(prmBTxFreq, header.transmitFrequency) &&
        m_porta->setParam(prmBSamplingFreq, header.samplingFrequency)) {

      int sector;
      m_porta->getParam(prmBSector, sector);
      imageHeader.scanLineNumber = m_porta->getParam(prmBLineDensity) * sector / 100;
      imageHeader.degPerFrame =
          m_porta->getParam(prmMotorSteps) * (double)(nfo.motorFov / 1000.0) / (double)nfo.motorSteps;

      int fpv;
      m_porta->getParam(prmMotorFrames, fpv);
      imageHeader.framesPerVolume = fpv;

      imageHeader.transmitFrequency = m_porta->getParam(prmBTxFreq);
      imageHeader.samplingFrequency = m_porta->getParam(prmBSamplingFreq);

      imageHeader.imageDepth = m_porta->getParam(prmBImageDepth);

      if (!header.postScanMode) {
        if ((imagingMode)header.imagingMode == BMode) {
#if USTK_PORTA_VERSION_MAJOR > 5
          portaRect rect;
#else
          URect rect;
#endif
          m_porta->getParam(prmBImageRect, rect);
          imageHeader.frameWidth = rect.right + 1;
          imageHeader.frameHeight = rect.bottom;
        } else if ((imagingMode)header.imagingMode == RfMode) {
          imageHeader.frameWidth = m_porta->getParam(prmRfNumLines);
          imageHeader.frameHeight = m_porta->getParam(prmRfNumSamples);
        }
      }

      if (header.postScanMode) {
        imageHeader.imageType = 1;
        imageHeader.ss = 8;
      } else if (header.imagingMode == BMode) {
        imageHeader.imageType = 0;
        imageHeader.ss = 8;
      } else if (header.imagingMode == RfMode) {
        imageHeader.imageType = 2;
        imageHeader.ss = 16;
      }

      confirmHeader.initOk = 1;
      return true;
    }
  }
  return false;
}

QTcpSocket *usNetworkServer::getSocket() { return connectionSoc; }

porta *usNetworkServer::getPorta() { return m_porta; }

QString usNetworkServer::getProbeSettingsFromId(int probeId)
{
  QString settingName;
  if (probeId == 10) {
    settingName = QString("FAST-General (C5-2 60mm).xml");
  } else if (probeId == 12) {
    settingName = QString("GEN-General (BPL9-5 55mm).xml");
  } else if (probeId == 13) {
    settingName = QString("GEN-General (BPC8-4 10mm).xml");
  } else if (probeId == 14) {
    settingName = QString("GEN-General (PAXY).xml");
  } else if (probeId == 15) {
    settingName = QString("GEN-General (4DC7-3 40mm).xml");
  }
  return settingName;
}

void usNetworkServer::writeInitAcquisitionParameters(QDataStream &stream, int imagingMode, int probeId)
{
  int transmitFrequency;
  int samplingFrequency;
  bool postScanMode = false;
  int postScanHeight = 0;
  int postScanWidth = 0;
  int imageDepth;
  int sector;
  bool activateMotor = false;
  int motorPosition = 0;
  int framesPerVolume = 1;
  int stepsPerFrame = 0;
  int transmitFrequencyMin;
  int samplingFrequencyMin;
  int imagingModeMin = 0;
  int imageDepthMin;
  int sectorMin;
  int motorPositionMin = 0;
  int framesPerVolumeMin = 1;
  int stepsPerFrameMin = 0;
  int transmitFrequencyMax;
  int samplingFrequencyMax;
  int imagingModeMax = 27;
  int imageDepthMax;
  int sectorMax;
  int motorPositionMax = 0;
  int framesPerVolumeMax = 0;
  int stepsPerFrameMax = 0;

  // case of 4DC7 probe
  if (probeId == 15) {
    motorPositionMax = 75;
    framesPerVolumeMax = 31;

    m_porta->getParam(prmMotorSteps, stepsPerFrame);
    m_porta->getParam(prmMotorFrames, framesPerVolume);
  }

  m_porta->getParam(prmBTxFreq, transmitFrequency);
  m_porta->getParam(prmBSamplingFreq, samplingFrequency);
  m_porta->getParam(prmBImageDepth, imageDepth);
  m_porta->getParam(prmBSector, sector);

  int motorStatus;
  m_porta->getParam(prmMotorStatus, motorStatus);
  if (motorStatus == 1) {
    activateMotor = true;
  }

  m_porta->getParamMinMax(prmBTxFreq, transmitFrequencyMin, transmitFrequencyMax);
  m_porta->getParamMinMax(prmBSamplingFreq, samplingFrequencyMin, samplingFrequencyMax);
  m_porta->getParamMinMax(prmBImageDepth, imageDepthMin, imageDepthMax);
  m_porta->getParamMinMax(prmBSector, sectorMin, sectorMax);

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

void usNetworkServer::writeUpdateAcquisitionParameters(QDataStream &stream, usUpdateHeaderIncomming header, int probeId)
{

  int framesPerVolume = 1;
  int stepsPerFrame = 0;

  int transmitFrequencyMin;
  int samplingFrequencyMin;
  int imagingModeMin = 0;
  int imageDepthMin;
  int sectorMin;
  int motorPositionMin = 0;
  int framesPerVolumeMin = 1;
  int stepsPerFrameMin = 0;

  int transmitFrequencyMax;
  int samplingFrequencyMax;
  int imagingModeMax = 27;
  int imageDepthMax;
  int sectorMax;
  int motorPositionMax = 0;
  int framesPerVolumeMax = 1;
  int stepsPerFrameMax = 0;

  if (probeId == 15) { // for 4DC7
    stepsPerFrame = 8;
    motorPositionMax = 75;
    framesPerVolumeMax = 31;

    m_porta->getParam(prmMotorSteps, stepsPerFrame);
    m_porta->getParam(prmMotorFrames, framesPerVolume);
  }

  m_porta->getParam(prmBTxFreq, header.transmitFrequency);
  m_porta->getParam(prmBSamplingFreq, header.samplingFrequency);
  m_porta->getDisplayDimensions(0, header.postScanWidth, header.postScanHeight);
  m_porta->getParam(prmBImageDepth, header.imageDepth);
  m_porta->getParam(prmBSector, header.sector);

  int motorStatus;
  m_porta->getParam(prmMotorStatus, motorStatus);
  header.activateMotor = (motorStatus == 1);
  std::cout << "writeUpdateAcquisitionParameters(), motor status = " << motorStatus << std::endl;

  m_porta->getParamMinMax(prmBTxFreq, transmitFrequencyMin, transmitFrequencyMax);
  m_porta->getParamMinMax(prmBSamplingFreq, samplingFrequencyMin, samplingFrequencyMax);
  m_porta->getParamMinMax(prmBImageDepth, imageDepthMin, imageDepthMax);
  m_porta->getParamMinMax(prmBSector, sectorMin, sectorMax);

  stream << header.transmitFrequency;
  stream << header.samplingFrequency;
  stream << header.imagingMode;
  stream << header.postScanMode;
  stream << header.postScanHeight;
  stream << header.postScanWidth;
  stream << header.imageDepth;
  stream << header.sector;
  stream << header.activateMotor;
  stream << header.motorPosition;
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

void usNetworkServer::writeOnSocketFromOtherThread() { emit(writeOnSocketSignal()); }

void usNetworkServer::writeOnSocketSlot()
{
  QByteArray block;
  std::cout << "writing header on socket" << std::endl;
  std::cout << "image number : " << imageHeader.frameCount << std::endl;
  QDataStream out(&block, QIODevice::WriteOnly);
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
  std::cout << "writing image on socket" << std::endl;

  if (imageHeader.imageType == 1) { // post scan
    out.writeRawData((char *)postScanImage, imageHeader.dataLength);
  } else if (imageHeader.imageType == 0) { // pre-scan
    out.writeRawData((char *)beginImage, imageHeader.dataLength);
  } else if (imageHeader.imageType == 2) { // RF
    out.writeRawData((char *)beginImage, imageHeader.dataLength);
  }

  quint64 dataWrittenSize = 0;

  if (motorOffsetSkipped && (motorStatus == 1)) { // 3D but offset skipped
    dataWrittenSize = connectionSoc->write(block);
  } else if (motorStatus == 0) { // 2D
    dataWrittenSize = connectionSoc->write(block);
  }

  std::cout << "written data size = " << dataWrittenSize << std::endl;

  // for bi-plane we send a second image
  if (biPlane && imageHeader.imageType == 1) {
    QByteArray block;
    std::cout << "writing 2nd header" << std::endl;
    QDataStream out(&block, QIODevice::WriteOnly);
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
    out.writeRawData((char *)secondBiplaneImage, imageHeader.dataLength);
    imageHeader.frameCount++;

    connectionSoc->write(block);
  }
}