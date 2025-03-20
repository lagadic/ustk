#include <visp3/ustk_gui/usUltrasonixClientWidget.h>

#if defined(USTK_HAVE_VTK_QT) && defined(VISP_HAVE_MODULE_USTK_GRABBER)

usUltrasonixClientWidget::usUltrasonixClientWidget()
{
  serverIpAdressLabel = new QLabel(QString("Server ip: "), this);
  probeSelectionLabel = new QLabel(QString("Probe (on top slot) : "), this);

  connectPushButton = new QPushButton(QString("Connect to server"), this);
  initPushButton = new QPushButton(QString("Init acquisition"), this);

  center3Dprobe = new QPushButton(QString("Center 3D probe motor"), this);
  startPushButton = new QPushButton(QString("Start Acquisition"), this);
  stopPushButton = new QPushButton(QString("Stop Acquisition"), this);

  ipTextEdit = new QLineEdit(QString("192.168.100.2"), this);
  probeSelectComboBox = new QComboBox(this);
  probeSelectComboBox->addItem(QString("4DC7 (3D)"));
  probeSelectComboBox->addItem(QString("C5-2 (2D)"));

  // ip validation
  QString ipRange = "(?:[0-1]?[0-9]?[0-9]|2[0-4][0-9]|25[0-5])";
#if defined(USTK_HAVE_VTK_QT6)
  QRegularExpression ipRegex("^" + ipRange + "\\." + ipRange + "\\." + ipRange + "\\." + ipRange + "$");
  ipValidator = new QRegularExpressionValidator(ipRegex, this);
#else
  QRegExp ipRegex("^" + ipRange + "\\." + ipRange + "\\." + ipRange + "\\." + ipRange + "$");
  ipValidator = new QRegExpValidator(ipRegex, this);
#endif

  ipTextEdit->setValidator(ipValidator);

  // layout
  Layout = new QGridLayout;
  Layout->addWidget(serverIpAdressLabel, 0, 0, Qt::AlignRight);
  Layout->addWidget(connectPushButton, 1, 0, 1, 2, Qt::AlignCenter);
  Layout->addWidget(probeSelectionLabel, 2, 0, Qt::AlignRight);
  Layout->addWidget(initPushButton, 3, 0, 3, 2, Qt::AlignCenter);
  Layout->addWidget(center3Dprobe, 4, 0, 4, 2, Qt::AlignCenter);
  Layout->addWidget(startPushButton, 5, 0, 5, 2, Qt::AlignCenter);
  Layout->addWidget(stopPushButton, 6, 0, 6, 2, Qt::AlignCenter);

  Layout->addWidget(ipTextEdit, 0, 1, Qt::AlignLeft);
  Layout->addWidget(probeSelectComboBox, 2, 1, Qt::AlignLeft);

  this->setLayout(Layout);

  connect(connectPushButton, SIGNAL(clicked()), this, SLOT(connectToServerSlot()));
  connect(initPushButton, SIGNAL(clicked()), this, SLOT(initAcquisitionSlot()));

  connect(center3Dprobe, SIGNAL(clicked()), this, SIGNAL(center3DProbeMotor()));

  connect(startPushButton, SIGNAL(clicked()), this, SIGNAL(runAcquisition()));
  connect(stopPushButton, SIGNAL(clicked()), this, SIGNAL(stopAcquisition()));
}

usUltrasonixClientWidget::~usUltrasonixClientWidget()
{
  delete serverIpAdressLabel;
  delete probeSelectionLabel;
  delete connectPushButton;
  delete initPushButton;
  delete startPushButton;
  delete center3Dprobe;
  delete stopPushButton;
  delete ipTextEdit;
  delete probeSelectComboBox;
  delete ipValidator;
}

void usUltrasonixClientWidget::initAcquisitionSlot()
{
  initHeader.slotId = 0;      // top slot
  initHeader.imagingMode = 0; // B-mode

  if (probeSelectComboBox->currentIndex() == 0) { // 4DC7
    initHeader.probeId = 15;
    emit(initAcquisition(initHeader));
  }
  else if (probeSelectComboBox->currentIndex() == 1) { // C5-2
    initHeader.probeId = 10;
    emit(initAcquisition(initHeader));
  }
  else {
    QMessageBox msgBox;
    msgBox.setText("Wrong probe selection !");
    msgBox.exec();
  }
}

void usUltrasonixClientWidget::connectToServerSlot()
{
  hostAddress.setAddress(ipTextEdit->text());
  emit(connectToServer(hostAddress));
}
#endif
