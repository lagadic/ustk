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

signals:
  void startServer();
  void stopServer();

private:
  QPushButton * m_startButton;
  QPushButton * m_stopButton;

};

#endif // US_SERVER_MAIN_WINDOW_H
