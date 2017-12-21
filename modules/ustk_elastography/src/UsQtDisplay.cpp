#include <visp3/ustk_elastography/UsQtDisplay.h>

UsQtDisplay::UsQtDisplay(QWidget *parent) : QWidget(parent)
{
  this->setMinimumSize(320, 370);
  m_idx = 0;
  m_Pstatus = false;
  m_trackerStat = false;
  m_drawing_elas = false;
  m_drawing_mesh = false;
  m_motion = false;
  m_sel = false;
  m_selected = false;
  m_dispInitialized = false;
  m_dispTimeStore = false;
  m_storeIm = false;
  m_lTitle = new QLabel("Ultrasound acquisition");
  m_lTitle->setStyleSheet("font-weight: bold; color: black");
  m_lTitle->setMaximumHeight(80);
  ///////////////////////////////
  m_cColors = new QComboBox();
  m_lColors = new QLabel("Colormap: ");
  m_hlayout = new QHBoxLayout();
  m_cColors->setMaximumWidth(180);
  m_cColors->addItem("Hot");
  m_cColors->addItem("Jet");
  m_cColors->addItem("Gray");
  m_hlayout->addWidget(m_lColors, 0, Qt::AlignRight);
  m_hlayout->addWidget(m_cColors, 0, Qt::AlignLeft);
  m_hlayout->setSpacing(0);
  // Creating custom plot
  m_QCplot = new QCustomPlot;
  m_QCplot->setMinimumSize(300, 300);
  /////////////////////////////
  m_disp = new QLabel;
  m_disp->setMinimumSize(320, 320);
  m_disp->setMaximumSize(320, 320);
  m_disp->setAutoFillBackground(true);
  QPalette pal = m_disp->palette();
  pal.setColor(QPalette::Window, QColor(Qt::black));
  m_disp->setPalette(pal);

  m_spacer = new QSpacerItem(10, 10, QSizePolicy::Minimum, QSizePolicy::Expanding);

  m_NLayout = new QVBoxLayout(this);
  m_NLayout->addWidget(m_lTitle, 0, Qt::AlignCenter);
  m_NLayout->addWidget(m_disp);
  m_NLayout->addLayout(m_hlayout);
  m_NLayout->addSpacerItem(m_spacer);

  // Central line
  m_haveCentroid = false;
  int mid_point_x = (m_disp->width() / 2.0) - 1.0;
  m_cenl = QLine(QPoint(mid_point_x, 0), QPoint(mid_point_x, m_disp->height() - 1));

  // Filter enabled?
  m_isFilterEnabled = false;

  connect(m_cColors, SIGNAL(currentIndexChanged(int)), SLOT(setColormap(int)));
  initPlot();
}

UsQtDisplay::~UsQtDisplay()
{
  delete m_disp;
  delete m_NLayout;
}

void UsQtDisplay::setCommonSharedPreScanImage(QSharedPointer<usImagePreScan2D<unsigned char> > t_PreScanIm)
{
  s_PreScanIm = t_PreScanIm;
}

void UsQtDisplay::setCommonSharedPostScanImage(QSharedPointer<usImagePostScan2D<unsigned char> > t_PostScanIm)
{
  s_PostScanIm = t_PostScanIm;
}

void UsQtDisplay::setCommonSharedRFImage(QSharedPointer<usImageRF2D<short int> > t_RFIm) { s_RFIm = t_RFIm; }

void UsQtDisplay::setCommonSharedStrainImage(QSharedPointer<vpMatrix> t_StrainIm) { s_StrainIm = t_StrainIm; }

void UsQtDisplay::setCommonSharedFilteredImage(QSharedPointer<vpMatrix> t_FilteredIm) { s_FilteredIm = t_FilteredIm; }

void UsQtDisplay::setCommonSharedArmaImage(QSharedPointer<mat> t_ArmaIm) { s_Ic = t_ArmaIm; }

void UsQtDisplay::Disp(void)
{
  // Just memory allocation at the begining, after just fill pointer tranfers
  if (!m_dispInitialized) {
    m_Ima = QImage(s_PreScanIm.data()->bitmap, s_PreScanIm.data()->getWidth(), s_PreScanIm.data()->getHeight(),
                   s_PreScanIm.data()->getWidth(), QImage::Format_Indexed8);
    m_w = s_RFIm.data()->getWidth();
    m_h = s_RFIm.data()->getHeight();
    m_pImage = m_Ima.bits();
    m_dispInitialized = true;
  } else
    m_Ima = QImage(s_PreScanIm.data()->bitmap, s_PreScanIm.data()->getWidth(), s_PreScanIm.data()->getHeight(),
                   s_PreScanIm.data()->getWidth(), QImage::Format_Indexed8);
  QImage I = m_Ima.convertToFormat(QImage::Format_RGB888);
  I = I.scaled(m_disp->width(), m_disp->height());
  m_Pix = QPixmap::fromImage(I);
  // Display image
  m_disp->setPixmap((m_drawing_elas) ? m_psq : m_Pix);
  // m_disp->setPixmap(m_Pix);
  m_disp->update();

  // If store image activated
  if (m_storeIm) {
    QString filename = QString("%1frame%2.png").arg("/mnt/ramdisk/").arg(m_idx, 5, 10, QChar('0'));
    emit saveImage(m_disp->pixmap()->toImage(), filename);
  }
  m_sel = true;
  m_idx++;
}

void UsQtDisplay::moveROI(void)
{
  emit sUpdateROIPos((int)round(m_tXroi * m_fsize.x()), (int)round(m_tYroi * m_fsize.y()));
}

void UsQtDisplay::chgImage(void)
{
  if (m_trackerStat)
    DispTracker();
  else
    Disp();
}

void UsQtDisplay::initPlot(void)
{
  // configure axis rect:
  m_QCplot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
  // Disabling the axes and the thick labels for our plot
  m_QCplot->xAxis->setVisible(false);
  m_QCplot->xAxis->setTickLabels(false);
  m_QCplot->yAxis->setVisible(false);
  m_QCplot->yAxis->setTickLabels(false);
  // Setting a backgroung
  // m_QCplot->setBackground(QBrush(QColor(180, 180, 180)));

  // set up the QCPColorMap:
  m_colorMap = new QCPColorMap(m_QCplot->xAxis, m_QCplot->yAxis);
  m_colorMap->data()->fill(0);
  // add a color scale:
  /*m_colorScale = new QCPColorScale(m_QCplot);
  m_QCplot->plotLayout()->addElement(0, 1, m_colorScale); // add it to the right of the main axis rect
  m_colorScale->setType(QCPAxis::atRight); // scale shall be vertical bar with tick/axis labels right (actually atRight
  is already the default)
  m_colorMap->setColorScale(m_colorScale); // associate the color map with the color scale */
  // m_colorScale->axis()->setLabel("Strain range");

  // set the color gradient of the color map to one of the presets:
  m_colorMap->setGradient(QCPColorGradient::gpHot);
  // we could have also created a QCPColorGradient instance and added own colors to
  // the gradient, see the documentation of QCPColorGradient for what's possible.

  // rescale the data dimension (color) such that all data points lie in the span visualized by the color gradient:
  m_colorMap->rescaleDataRange(true);
  m_colorMap->setDataRange(QCPRange(0.99, 1));

  // make sure the axis rect and color scale synchronize their bottom and top margins (so they line up):
  // QCPMarginGroup *marginGroup = new QCPMarginGroup(m_QCplot);
  // m_QCplot->axisRect()->setMarginGroup(QCP::msBottom|QCP::msTop, marginGroup);
  // m_colorScale->setMarginGroup(QCP::msBottom|QCP::msTop, marginGroup);
  m_QCplot->axisRect()->setAutoMargins(QCP::msNone);

  // rescale the key (x) and value (y) axes so the whole color map is visible:
  m_QCplot->rescaleAxes(true);
  m_initialized = false;
}

void UsQtDisplay::setColormap(int t_idx)
{
  switch (t_idx) {
  case 0:
    m_colorMap->setGradient(QCPColorGradient::gpHot);
    break;
  case 1:
    m_colorMap->setGradient(QCPColorGradient::gpJet);
    break;
  case 2:
    m_colorMap->setGradient(QCPColorGradient::gpGrayscale);
    break;
  default:
    m_colorMap->setGradient(QCPColorGradient::gpHot);
    break;
  }
  ///// Maybe remove next line during the acquisition ///////
  DrawElasto();
}

/*!
 * \brief UsQtDisplay::changeX_ROI
 * \param t_x: displacement in m
 */
void UsQtDisplay::changeX_ROI(double t_x)
{
  t_x /= (((double)m_w / m_disp->width()) * SX_rf);
  m_tXroi = (((t_x + m_rectangle.width()) < m_disp->width()) && ((t_x + m_rectangle.width()) >= 0))
                ? t_x + m_rectangle.x()
                : m_tXroi;
  moveROI();
}

/*!
 * \brief UsQtDisplay::changeY_ROI
 * \param t_y: displacement in m
 */
void UsQtDisplay::changeY_ROI(double t_y)
{
  t_y /= (((double)m_h / m_disp->height()) * SY_rf);
  m_tYroi = (((t_y + m_rectangle.height()) < m_disp->height()) && ((t_y + m_rectangle.height()) >= 0))
                ? t_y + m_rectangle.y()
                : m_tYroi;
  moveROI();
}

void UsQtDisplay::setStoreImages(bool t_state)
{
  m_storeIm = t_state;
  qDebug() << "Storage time:" << QTime::currentTime().toString("hh:mm:ss.zzz") << endl;
}

void UsQtDisplay::enabledFilteredImage() { m_isFilterEnabled = true; }

void UsQtDisplay::DrawElasto(void)
{
  int st_w = s_StrainIm.data()->getCols();
  int st_h = s_StrainIm.data()->getRows();
  if (!m_initialized) {
    m_colorMap->data()->setSize(st_w, st_h); // we want the color map to have nx * ny data points
    m_initialized = true;
  }
  // now we assign some data, by accessing the QCPColorMapData instance of the color map:
  vpMatrix t_vpm = (m_isFilterEnabled) ? *(s_FilteredIm.data()) : *(s_StrainIm.data());
  for (int xIndex = 0; xIndex < st_w; ++xIndex) {
    for (int yIndex = 0; yIndex < st_h; ++yIndex) {
      m_colorMap->data()->setCell(xIndex, st_h - yIndex - 1, *(t_vpm.data + xIndex * st_h + yIndex));
    }
  }
  if (!m_drawing_elas)
    m_colorMap->rescaleDataRange(true);
  // m_QCplot->rescaleAxes(true);
  m_QCplot->replot();

  /// Overlaping  ///////////////////////////
  // Scale to plot
  double t_sw = st_w / m_fsize.x();
  double t_sh = st_h / m_fsize.y();

  QPointF t_difpoint((m_rectangle.width() - t_sw) / 2.0, (m_rectangle.height() - t_sh) / 2.0);

  m_psq = QPixmap(m_disp->width(), m_disp->height());
  QPainter p(&m_psq);
  p.drawPixmap(0, 0, m_Pix);
  p.setRenderHint(QPainter::Antialiasing, true);
  p.setOpacity(0.7); // 0.6 good
  p.drawPixmap(QPoint(m_tXroi, m_tYroi) + (t_difpoint), m_QCplot->toPixmap().scaled(t_sw, t_sh));
  p.setOpacity(1.0);
  if (m_haveCentroid) {
    p.setPen(QPen(Qt::green, 2));
    p.drawLine(m_cenl);
    p.setPen(QPen(Qt::yellow, 2));
    p.drawLine(m_roil);
    m_haveCentroid = false;
  }
  m_drawing_elas = true;
  m_isFilterEnabled = false;
  /*QString filename = QString("%1frame%2.png").arg("/mnt/ramdisk/").arg(m_idx, 5, 10, QChar('0'));
  qDebug() << filename << endl;
  m_psq.toImage().save(filename, "PNG");*/
  ///// Maybe remove next line during the acquisition ///////
  Disp();
}

void UsQtDisplay::DrawLineCentroid(QPointF t_point)
{
  int midRoiPx = (t_point.x() / ((double)m_w / m_disp->width()));
  int midRoiPy = (t_point.y() / ((double)m_h / m_disp->height()));

  double t_centerX = floor((double)m_rectangle.width() / 2.0);
  double t_centerY = floor((double)m_rectangle.height() / 2.0);
  double dif_x = (midRoiPx - t_centerX);

  double perw = 0.07 * t_centerX; //// Try changing this value and check units above
  //// Try without this
  double dXc = (dif_x > perw) ? perw : (dif_x < -perw) ? -perw : dif_x;
  int mid_point_x = m_tXroi + t_centerX + dXc;
  int mid_point_y = m_tYroi + t_centerY;
  double Dx = (mid_point_x - floor((double)m_disp->width() / 2.0));
  double Dy = (mid_point_y - floor((double)m_disp->height() / 2.0));

  m_roil = QLine(QPoint(mid_point_x, 0), QPoint(mid_point_x, m_disp->height() - 1));
  m_haveCentroid = true;
  // To elasto object
  emit sUpdateROIPos((int)round((m_tXroi + dXc) * m_fsize.x()), (int)round(m_tYroi * m_fsize.y()));
  // To robot object in m --- pixels*(Aline/pixel)*(m/Aline)
  emit sUpdateCentoid_m(Dx * m_fsize.x() * SX_rf, Dy * m_fsize.y() * SY_rf);

  m_tXroi += dXc;
}

void UsQtDisplay::writeLabelFPS(double v) { m_lTitle->setText("Tracking @ " + QString::number(v) + " FPS"); }

void UsQtDisplay::DispTracker(void)
{
  convert_mat2QImage();
  QImage I = m_Ima.convertToFormat(QImage::Format_RGB888);
  I = I.scaled(m_disp->width(), m_disp->height());
  m_Pix = QPixmap::fromImage(I);
  // Display image
  m_disp->setPixmap((m_drawing_mesh) ? m_psq : m_Pix);
  m_disp->update();
  // emit updatedQImage();
  m_sel = true;
  m_drawing_mesh = false;
  m_idx++;
}

void UsQtDisplay::convert_mat2QImage(void)
{
  if (!m_dispInitialized) {
    m_Ima = QImage(s_Ic.data()->n_cols, s_Ic.data()->n_rows, QImage::Format_Indexed8);
    m_w = m_Ima.width();
    m_h = m_Ima.height();
    m_dispInitialized = true;
  }
  uchar *t_bits = m_Ima.bits();
  //#pragma omp parallel for
  for (int y = 0; y < m_h; y++) {
    for (int x = 0; x < m_w; x++) {
      *(t_bits + y * m_w + x) = static_cast<unsigned char>(s_Ic.data()->at(y, x));
    }
  }
}

void UsQtDisplay::drawMesh(QVector<QLineF> vlines)
{
  m_psq = QPixmap(m_disp->width(), m_disp->height());
  QPainter p(&m_psq);
  p.setPen(QPen(Qt::green, 3));
  p.drawPixmap(0, 0, m_Pix);
  p.drawLines(vlines);
  m_drawing_mesh = true;
  m_disp->setPixmap(m_psq);
  m_disp->update();
}

////////////////// Window's events ////////////////////////////

//=======================================================
// Window Resize Event -- Every resize
//=======================================================
void UsQtDisplay::resizeEvent(QResizeEvent *event) { event->accept(); }

//=======================================================
// Mouse Press Event -- Click to drag a position
//=======================================================
void UsQtDisplay::mousePressEvent(QMouseEvent *e)
{
  if (e->button() == Qt::LeftButton) {
    if (m_sel) {
      m_initp.setX(e->pos().x());
      m_initp.setY(e->pos().y());
      m_motion = false;
    }
  }
  e->accept();
}
//=======================================================
// Mouse Release Event -- Select Region when release
//=======================================================
void UsQtDisplay::mouseReleaseEvent(QMouseEvent *e)
{
  if (e->button() == Qt::LeftButton) {
    if (m_sel) {
      if (m_initp.x() > e->pos().x() && m_initp.y() > e->pos().y()) {
        m_endp.setX(m_initp.x());
        m_endp.setY(m_initp.y());
        m_initp.setX(e->pos().x());
        m_initp.setY(e->pos().y());
      } else if (m_initp.x() == e->pos().x() || m_initp.y() == e->pos().y()) {
        QMessageBox::warning(this, "Error", "Select a rectangular area");
        return;
      } else {
        m_endp.setX(e->pos().x());
        m_endp.setY(e->pos().y());
      }

      m_rectangle = QRectF((float)m_initp.x() - m_disp->pos().x(), (float)m_initp.y() - m_disp->pos().y(),
                           (float)m_endp.x() - (float)m_initp.x(), (float)m_endp.y() - (float)m_initp.y());

      m_fsize.setX((float)m_w / m_disp->width());
      m_fsize.setY((float)m_h / m_disp->height());

      m_pAdd = QPointF(m_rectangle.x() + ceil(m_rectangle.width() / 2.0),
                       m_rectangle.y() + ceil(m_rectangle.height() / 2.0));

      // For ROI management
      m_tXroi = m_rectangle.x();
      m_tYroi = m_rectangle.y();
      emit InitPosition(m_rectangle.x(), m_rectangle.y());

      // Cut ROI and put in a varible
      if (m_trackerStat) {
        emit sPatternT((int)round(m_rectangle.x() * m_fsize.x()), (int)round(m_rectangle.y() * m_fsize.y()),
                       (int)round(m_rectangle.width() * m_fsize.x()), (int)round(m_rectangle.height() * m_fsize.y()),
                       m_fsize, m_pAdd);
      } else {
        emit sPattern((int)round(m_rectangle.x() * m_fsize.x()), (int)round(m_rectangle.y() * m_fsize.y()),
                      (int)round(m_rectangle.width() * m_fsize.x()), (int)round(m_rectangle.height() * m_fsize.y()));
      }
      m_selected = true;
    }
  }
  e->accept();
}

//=====================================================================
// Mose move Event -- click to move perspective camera or select region
//=====================================================================
void UsQtDisplay::mouseMoveEvent(QMouseEvent *e)
{
  if (m_sel) {
    this->setCursor(Qt::CrossCursor);
    QPixmap psq(m_disp->width(), m_disp->height());
    QPainter p(&psq);
    p.setPen(QPen(Qt::red, 3));
    float xi = (float)m_initp.x() - m_disp->pos().x();
    float yi = (float)m_initp.y() - m_disp->pos().y();
    float wt = (float)e->pos().x() - (float)m_initp.x();
    float ht = (float)e->pos().y() - (float)m_initp.y();
    QVector<QRectF> vrect;
    // Rectangle grid
    vrect.push_back(QRectF(xi, yi, wt / 2.0, ht / 2.0));
    vrect.push_back(QRectF(xi + wt / 2.0, yi, wt / 2.0, ht / 2.0));
    vrect.push_back(QRectF(xi, yi + ht / 2.0, wt / 2.0, ht / 2.0));
    vrect.push_back(QRectF(xi + wt / 2.0, yi + ht / 2.0, wt / 2.0, ht / 2.0));
    p.drawPixmap(0, 0, m_Pix);
    p.drawRects(vrect);
    m_disp->setPixmap(psq);
  } else {
    this->setCursor(Qt::ArrowCursor);
  }
  e->accept();
}
