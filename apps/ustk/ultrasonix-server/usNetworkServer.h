#ifndef US_NETWORK_GRABBER_H
#define US_NETWORK_GRABBER_H

#include <usPortaConfig.h>

#include <QtCore/QDateTime>
#include <QtCore/QFile>
#include <QtCore/QString>
#include <QtNetwork/QTcpServer>
#include <QtNetwork/QTcpSocket>
#include <QtWidgets/QApplication>

// for URect
#define COMPILE_PORTA_DEMO

// Ultrasonix SDK includes
#include <ImagingModes.h>
#include <porta.h>
#include <porta_params_def.h>
#include <utx_opt.h>

#if USTK_PORTA_VERSION_MAJOR > 5
#include <porta_def.h>
#include <porta_wrapper.h>
#define RfMode 12
#else
#include <porta_std_includes.h>
#endif

#include <cmath>
#include <ctime>
#include <iostream>
#include <vector>

#include <usPortaConfig.h>

class usNetworkServer : public QObject
{
  Q_OBJECT

public:
  // Following headers must be the same in the client grabber !

  // header sent by the client to init porta
  struct usInitHeaderIncomming {
    usInitHeaderIncomming() : headerId(1) {} // set header Id to 1 by default
    int headerId; // to differenciate usInitHeaderIncomming (=1) / usAcquisitionParameters (=2) /
                  // usRunControlHeaderSent(=3)

    // probe / slot selection
    int probeId; // available probes : 4DC7 (id = 15) , C5-2 (id = 10)
    int slotId;  // 3 slots on ultrasonix station (0 : top, 2 : middle, 1 : botom)

    int imagingMode; // see ImagingModes.h
  };

  // header sent by the client to update porta config
  struct usUpdateHeaderIncomming {
    usUpdateHeaderIncomming() : headerId(2) {} // set header Id to 1 by default
    int headerId; // to differenciate usInitHeaderIncomming (=1) / usUpdateHeaderIncomming (=2)

    int transmitFrequency;
    int samplingFrequency;
    int imagingMode;
    bool postScanMode;
    int postScanHeight;
    int postScanWidth;
    int imageDepth;
    int sector;
    bool activateMotor;
    int motorPosition;
    int framesPerVolume;
    int stepsPerFrame;
  };

  // header sent to the client to confirm porta is initialized
  struct usInitHeaderConfirmation {
    usInitHeaderConfirmation() : headerId(1) {} // set header Id to 1 by default
    int headerId;                               // to differenciate usInitHeaderConfirmation (=1) / usImageHeader (=2)

    int initOk;  // 1 if init ok, 0 otherwise
    int probeId; // unique code for each probe (4DC7 = 15, C 50-62 = 10)
  };

  //
  struct usImageHeader {
    usImageHeader() : headerId(2) {} // set header Id to 2 by default
    int headerId;                    // to differenciate usInitHeaderConfirmation (=1) / usImageHeader (=2)

    quint32 frameCount;
    quint64 timeStamp;

    double dataRate; // fps

    int dataLength; // image size in bytes
    int ss;

    int imageType;

    int frameWidth;
    int frameHeight;

    double pixelWidth;  // width of a pixel of a post-scan frame (meters)
    double pixelHeight; // height of a pixel of a post-scan frame (meters)

    int transmitFrequency;
    int samplingFrequency;

    double transducerRadius;
    double scanLinePitch;
    unsigned int scanLineNumber;
    int imageDepth;

    double degPerFrame;
    int framesPerVolume;
    double motorRadius;
    int motorType;
  };

  explicit usNetworkServer(QObject *parent = 0);
  ~usNetworkServer();

  QTcpSocket *getSocket();
  porta *getPorta();

  usImageHeader imageHeader;

  unsigned char *postScanImage;

  // for bi-plane
  unsigned char *secondBiplaneImage;

  bool motorOffsetSkipped;

private slots:
  // Called automatically when a client attempts to connect
  void acceptTheConnection();

  // Called automatically when client has closed the connection
  void connectionAboutToClose();

  // to get the name of the xml settings file corresponding to a probe Id
  QString getProbeSettingsFromId(int probeId);

  // Called automatically when data sent by a client is fully available to the server
  void readIncomingData();

private:
  // Variable(socket) to store listening tcpserver
  QTcpServer tcpServer;

  // Variable(socket) to store newly established connection with the client
  QTcpSocket *connectionSoc;

  void initPorta(usInitHeaderIncomming header);
  bool updatePorta(usUpdateHeaderIncomming header);
  void writeInitAcquisitionParameters(QDataStream &out, int imagingMode, int probeId);
  void writeUpdateAcquisitionParameters(QDataStream &stream, usUpdateHeaderIncomming header, int probeId);

  porta *m_porta;
  imagingMode m_currentImagingMode;

  usInitHeaderConfirmation confirmHeader;

  bool initWithoutUpdate;
};

#endif // US_NETWORK_GRABBER_H
