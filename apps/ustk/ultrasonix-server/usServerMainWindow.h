#ifndef US_SERVER_MAIN_WINDOW_H
#define US_SERVER_MAIN_WINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <QtWidgets/QApplication>

class usServerMainWindow : public QMainWindow
{
  Q_OBJECT

public:
  usServerMainWindow(QObject *parent = 0);
  ~usServerMainWindow();
  
public slots:
  void startStopSlot();

signals:
  void startServer();
  void stopServer();

private:
  QPushButton * m_startStopButton;
  bool m_isStarted;
};

#endif // US_SERVER_MAIN_WINDOW_H
