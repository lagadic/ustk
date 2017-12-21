#ifndef US_QT_DISPLAY_H
#define US_QT_DISPLAY_H

#include <QCursor>
#include <QDebug>
#include <QElapsedTimer>
#include <QImage>
#include <QLabel>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPainter>
#include <QResizeEvent>
#include <QSharedPointer>
#include <QThread>
#include <QVBoxLayout>
#include <QWidget>
#include <armadillo>
#include <visp3/core/vpMatrix.h>
#include <visp3/ustk_core/usImagePostScan2D.h>
#include <visp3/ustk_core/usImagePreScan2D.h>
#include <visp3/ustk_core/usImageRF2D.h>
#include <visp3/ustk_elastography/qcustomplot.h>
// scale definitions
#define SX_rf 2.21e-3 //[m/A-line]
#define SY_rf 3.85e-5 //[m/sample]

using namespace arma;

class VISP_EXPORT UsQtDisplay : public QWidget
{
  Q_OBJECT
public:
  UsQtDisplay(QWidget *parent = 0);
  virtual ~UsQtDisplay();
  void setCommonSharedPreScanImage(QSharedPointer<usImagePreScan2D<unsigned char> > t_PreScanIm);
  void setCommonSharedPostScanImage(QSharedPointer<usImagePostScan2D<unsigned char> > t_PreScanIm);
  void setCommonSharedRFImage(QSharedPointer<usImageRF2D<short int> > t_RFIm);
  void setCommonSharedStrainImage(QSharedPointer<vpMatrix> t_StrainIm);
  void setCommonSharedFilteredImage(QSharedPointer<vpMatrix> t_FilteredIm);
  void setCommonSharedArmaImage(QSharedPointer<mat> t_ArmaIm);
  void SetStatusPostScan(bool t_status) { m_Pstatus = t_status; }
  void SetTrackerStatus(bool t_status) { m_trackerStat = t_status; }
public slots:
  void chgImage(void);
  void DrawElasto(void);
  void drawMesh(QVector<QLineF> vlines);
  void DrawLineCentroid(QPointF t_point);
  void writeLabelFPS(double v);
  void setColormap(int t_idx);
  void changeX_ROI(double t_x);
  void changeY_ROI(double t_y);
  void setStoreImages(bool t_state);
  void enabledFilteredImage(void);
signals:
  void sPattern(int, int, int, int);
  void sPatternT(int, int, int, int, QPointF, QPointF);
  void InitPosition(int, int);
  void sUpdateROIPos(int, int);
  void sUpdateCentoid_m(double, double);
  void saveImage(QImage, QString);
  //
  void updateFrame();

private:
  QMutex m_mutex;
  void initPlot(void);
  void Disp(void);
  void DispTracker(void);
  void moveROI(void);
  void convert_mat2QImage(void);
  QCustomPlot *m_QCplot;
  QCPColorMap *m_colorMap;
  QCPColorScale *m_colorScale;
  QComboBox *m_cColors;
  QLabel *m_lColors;
  bool m_initialized;
  QHBoxLayout *m_hlayout;
  QLine m_roil;
  QLine m_cenl;
  bool m_haveCentroid;
  ////////////////
  QImage m_Ima;
  uchar *m_pImage;
  QRectF m_region;
  QPoint m_initp, m_endp;
  QPoint m_Xp;
  QPoint m_poss;
  QRectF m_rectangle;
  bool m_sel;
  bool m_motion;
  bool m_selected;
  bool m_chgd;
  bool m_dispInitialized;
  bool m_drawing_mesh;
  bool m_dispTimeStore;
  bool m_drawing_elas;
  QPixmap m_psq;

  QPixmap m_Pix;
  QLabel *m_disp;
  QLabel *m_lTitle;
  QVBoxLayout *m_NLayout;
  int m_w;
  int m_h;
  QPointF m_fsize;
  QPointF m_pAdd;
  // ROI displacement variables
  double m_tXroi;
  double m_tYroi;
  // Shared pointers for preScanImage and StrainIm
  QSharedPointer<usImagePreScan2D<unsigned char> > s_PreScanIm;
  QSharedPointer<usImagePostScan2D<unsigned char> > s_PostScanIm;
  QSharedPointer<usImageRF2D<short int> > s_RFIm;
  QSharedPointer<vpMatrix> s_StrainIm;
  QSharedPointer<vpMatrix> s_FilteredIm;
  //// For tracker
  QSharedPointer<mat> s_Ic;
  // Image storage
  bool m_storeIm;
  int m_idx;
  // Filtered image
  bool m_isFilterEnabled;
  QSpacerItem *m_spacer;
  // Adding modality PostScan
  bool m_Pstatus;
  bool m_trackerStat;

protected:
  void resizeEvent(QResizeEvent *event);
  void mousePressEvent(QMouseEvent *e);
  void mouseReleaseEvent(QMouseEvent *e);
  void mouseMoveEvent(QMouseEvent *e);
};
#endif
