#include "usServerMainWindow.h"

usServerMainWindow::usServerMainWindow(QObject *parent) : QObject(parent)
{
  m_startButton = new QPushButton(this);
  m_stopButton = new QPushButton(this);

  connect(m_startButton, SIGNAL(clicked()), this, SIGNAL(startServer()));
  connect(m_stopButton, SIGNAL(clicked()), this, SIGNAL(stopServer()));
}

usServerMainWindow::~usServerMainWindow() {
  delete m_startButton;
  delete m_stopButton;
}
