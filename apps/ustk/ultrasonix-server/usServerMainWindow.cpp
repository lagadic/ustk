#include "usServerMainWindow.h"

usServerMainWindow::usServerMainWindow(QObject *parent)
{
  m_startStopButton = new QPushButton(QString("Start server"), this);
  m_isStarted = false;

  connect(m_startStopButton, SIGNAL(clicked()), this, SLOT(startStopSlot()));
}

usServerMainWindow::~usServerMainWindow() {
  delete m_startStopButton;
}

void usServerMainWindow::startStopSlot() {
  if(m_isStarted) {
    m_isStarted = false;
    emit(stopServer());
  }
  else {
    m_isStarted = true;
    emit(startServer());
	m_startStopButton->setText(QString("Stop server"));
  }
}
