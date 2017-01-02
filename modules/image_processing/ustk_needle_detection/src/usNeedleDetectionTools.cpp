/****************************************************************************
 *
 * This file is part of the UsNeedleDetection software.
 * Copyright (C) 2013 - 2016 by Inria. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License ("GPL") as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 * See the file COPYING at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact the
 * authors at Alexandre.Krupa@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Authors:
 * Pierre Chatelain
 * Alexandre Krupa
 *
 *****************************************************************************/

#include <visp3/ustk_needle_detection/usNeedleDetectionTools.h>

void usNeedleDetectionTools::arithmeticMean(vpMatrix points, double *mean,
                                            unsigned int npts, unsigned int d)
{
  for (unsigned int i=0; i<d; i++)
  {
    mean[i] = 0;
    for (unsigned int j=0; j<npts; j++)
    {
      mean[i] += points[j][i];;
    }
    mean[i] /= (double) npts;
  }
}

vpColVector usNeedleDetectionTools::geometricMedian(vpMatrix points,
                                                    unsigned int npts, unsigned int d)
{
  if (npts == 0) {
    std::cerr << "Error: In usNeedleDetectionTools::geometricMedian(): "
              << "argument 2 should be > 0." << std::endl;
    exit(EXIT_FAILURE);
  }
  if (npts == 1) {
    return points.getRow(0).t();
  }
  // Weiszfeld's algorithm (IRLS)
  vpColVector oldMedian = vpColVector(d);
  vpColVector newMedian = vpColVector(d);
  double eps = 1e-3;
  double diff = 1;
  arithmeticMean(points, oldMedian.data, npts, d);
  while (diff > eps)
  {
    double norm = 0;
    for (unsigned int j=0; j<npts; j++)
      norm += 1 / (points.getRow(j).t()-oldMedian).euclideanNorm();
    newMedian = 0;
    for (unsigned int j=0; j<npts; j++)
      newMedian += points.getRow(j).t() / (points.getRow(j).t()-oldMedian).euclideanNorm();
    newMedian /= norm;
    diff = (oldMedian - newMedian).euclideanNorm();
    oldMedian = newMedian;
  }
  return oldMedian;
}

double usNeedleDetectionTools::variance(vpMatrix points, unsigned int npts, unsigned int d)
{
  double *mean = new double[3];
  double sumSquare = 0;
  arithmeticMean(points, mean, npts, d);
  for (unsigned int i=0; i<npts; i++)
    for (unsigned int j=0; j<d; j++)
      sumSquare += (points[i][j] - mean[j]) * (points[i][j] - mean[j]);
  return sumSquare/npts;
}

bool diff(int i, int j)
{
  return (i!=j);
}

short usNeedleDetectionTools::quantile(short *data, unsigned int num, unsigned int n)
{
  unsigned int *hist = new unsigned int[512];
  for (unsigned int i=0; i<512; i++)
    hist[i] = 0;
  for (unsigned int i=0; i<n; i++)
    hist[data[i]]++;
  unsigned int counter = 0;
  unsigned int idx = 511;
  while (counter<num)
  {
    counter += hist[idx];
    idx--;
  }
  delete[] hist;
  return idx+1;
}

#if defined USNEEDLEDETECTION_HAVE_VTK
short usNeedleDetectionTools::quantile(vtkDataSet *data, unsigned int num)
{
  unsigned int n = data->GetNumberOfPoints();
  unsigned int *hist = new unsigned int[512];
  for (unsigned int i=0; i<n; i++)
    hist[((vtkShortArray*)data->GetPointData()->GetScalars())->GetValue(i)]++;
  unsigned int counter = 0;
  unsigned int idx = 511;
  while (counter<num)
  {
    counter += hist[idx];
    idx--;
  }
  delete[] hist;
  return idx+1;
}
#endif

void usNeedleDetectionTools::computeQuantile(unsigned char *data, unsigned int dataSize,
                                             unsigned int nDesired, unsigned int &q,
                                             unsigned int &nThresholded)
{
  unsigned int *hist = new unsigned int[256];
  for (unsigned int i=0; i<256; i++)
    hist[i] = 0;
  for (unsigned int i=0; i<dataSize; i++)
    hist[data[i]]++;
  nThresholded = 0;
  q = 256;
  while (nThresholded<nDesired)
  {
    q--;
    nThresholded += hist[q];
  }
  delete[] hist;
}

void usNeedleDetectionTools::computeQuantile(short *data, unsigned int dataSize,
                                             unsigned int nDesired, unsigned int &q,
                                             unsigned int &nThresholded)
{
  unsigned int *hist = new unsigned int[511];
  for (unsigned int i=0; i<511; i++)
    hist[i] = 0;
  for (unsigned int i=0; i<dataSize; i++)
    hist[data[i]]++;
  nThresholded = 0;
  q = 512;
  while (nThresholded < nDesired)
  {
    q--;
    nThresholded += hist[q];
  }
  delete[] hist;
}

void usNeedleDetectionTools::computeQuantile(const unsigned int *data, unsigned int dataSize,
                                             unsigned int nDesired, unsigned int &q,
                                             unsigned int &nThresholded)
{
  unsigned int max = 0;
  for (unsigned int i = 0; i < dataSize; ++i)
    if (data[i] > max)
      max = data[i];
  unsigned int *hist = new unsigned int[max + 1];
  for (unsigned int i = 0; i < max + 1; i++)
    hist[i] = 0;
  for (unsigned int i = 0; i < dataSize; i++)
    hist[data[i]]++;
  nThresholded = 0;
  q = max + 1;
  while (nThresholded < nDesired)
  {
    q--;
    nThresholded += hist[q];
  }
  delete[] hist;
}

#if defined USNEEDLEDETECTION_HAVE_VTK
int usNeedleDetectionTools::findTip(vtkDataArray *data, unsigned int dataSize, double gap)
{
  int stop = -1;
  std::cerr << "Needle profile: ";
  for (int i=1; i<dataSize; i++) {
    std::cerr << data->GetTuple(i)[0] << " ";
    if (data->GetTuple(i-1)[0] - data->GetTuple(i)[0] > gap)
      stop = i-1;
  }
  std::cerr << std::endl;
  std::cerr << "Tip index: " << stop << std::endl;
  return stop;
}

bool usNeedleDetectionTools::findTip(vtkImageData *image, const vpMatrix &model, int *VOI, double *tip, unsigned int nPoints, double gap)
{
  bool found = 0;
  vpColVector a(nPoints);
  vpColVector p0(3);
  vpColVector p(3);
  double t = 0.0;
  a[0] = 1;
  for (unsigned int i=1; i<nPoints; ++i)
    a[i] = a[i-1] * t;
  p0 = model * a;
  t = 1.0;
  a[0] = 1;
  for (unsigned int i=1; i<nPoints; ++i)
    a[i] = a[i-1] * t;
  p = model * a;
  double l = (p-p0).euclideanNorm();
  double intensity1;
  t = 0.5;
  a[0] = 1;
  for (unsigned int i=1; i<nPoints; ++i)
    a[i] = a[i-1] * t;
  p = model * a;
  intensity1 = image->GetScalarComponentAsDouble(p[0], p[1], p[2], 0);
  while (inside(p,VOI)) {
    t += 1.0 / l;
    a[0] = 1;
    for (unsigned int i=1; i<nPoints; ++i)
      a[i] = a[i-1] * t;
    p = model * a;
    double intensity2 = image->GetScalarComponentAsDouble(vpMath::round(p[0]), vpMath::round(p[1]), vpMath::round(p[2]), 0);
    //std::cerr << "("<< p[0] << "," << p[1] << "," << p[2] << "): " << intensity2 << std::endl;
    if (intensity1 - intensity2 > gap) {
      found = true;
      for (unsigned int i=0; i<3; ++i)
        tip[i] = p[i];
    }
    intensity1 = intensity2;
  }
  return found;
}
#endif

bool usNeedleDetectionTools::findEntry(const vpMatrix &model, double *entry, unsigned int nPoints,
                                       const vpColVector &origin, const vpColVector &entryPlane,
                                       int VOI[6])
{
  //std::cerr << "In usNeedleDetectionTools::findEntry()" << std::endl;
  bool found = 0;
  vpColVector a(nPoints);
  vpColVector p0(3);
  vpColVector p(3);
  double t = 1.0;
  a[0] = 1;
  for (unsigned int i=1; i<nPoints; ++i)
    a[i] = a[i-1] * t;
  p0 = model * a;
  t = 0.0;
  a[0] = 1;
  for (unsigned int i=1; i<nPoints; ++i)
    a[i] = a[i-1] * t;
  p = model * a;
  double l = (p-p0).euclideanNorm();
  double dotProduct = 1.0;
  double bestDotProduct = DBL_MAX;

  if (nPoints == 2) {
    found = intersectionLinePlane(model, entryPlane,
                                  - origin[0] * entryPlane[0] - origin[1] * entryPlane[1],
        entry[0], entry[1], entry[2]);
    return found;
  }

  while (dotProduct < bestDotProduct && inside(p,VOI)) {
    bestDotProduct = dotProduct;
    t -= 0.5 / l;
    a[0] = 1;
    for (unsigned int i=1; i<nPoints; ++i)
      a[i] = a[i-1] * t;
    p = model * a;
    dotProduct = std::abs(entryPlane * (p - origin)) / (p - origin).euclideanNorm();
    if (dotProduct < bestDotProduct) {
      found = true;
      entry[0] = p[0];
      entry[1] = p[1];
      entry[2] = p[2];
    }
  }
  //std::cerr << "Out usNeedleDetectionTools::findEntry()" << std::endl;
  return found;
}

#if defined USNEEDLEDETECTION_HAVE_VTK
bool usNeedleDetectionTools::findTip(vtkImageData *image, const vpMatrix &model, int *VOI, double *tip, unsigned int nPoints, double gap, double length)
{
  bool found = 0;
  vpColVector a(nPoints);
  vpColVector p0(3);
  vpColVector p(3);
  double t = 0.0;
  a[0] = 1;
  for (unsigned int i=1; i<nPoints; ++i)
    a[i] = a[i-1] * t;
  p0 = model * a;
  t = 1.0;
  a[0] = 1;
  for (unsigned int i=1; i<nPoints; ++i)
    a[i] = a[i-1] * t;
  p = model * a;
  double l = (p-p0).euclideanNorm();
  double minLengthError = DBL_MAX;
  double intensity1;
  t = 0.5;
  a[0] = 1;
  for (unsigned int i=1; i<nPoints; ++i)
    a[i] = a[i-1] * t;
  p = model * a;
  intensity1 = image->GetScalarComponentAsDouble(p[0], p[1], p[2], 0);
  while (inside(p,VOI)) {
    double lengthError = 0.0;
    t += 1.0 / l;
    a[0] = 1;
    for (unsigned int i=1; i<nPoints; ++i)
      a[i] = a[i-1] * t;
    p = model * a;
    double intensity2 = image->GetScalarComponentAsDouble(vpMath::round(p[0]), vpMath::round(p[1]), vpMath::round(p[2]), 0);
    lengthError = abs((p-p0).euclideanNorm() - length);
    //std::cerr << "("<< p[0] << "," << p[1] << "," << p[2] << "): I=" << intensity2 << " dL=" << lengthError << std::endl;
    if (intensity1 - intensity2 > gap && lengthError < minLengthError) {
      found = true;
      minLengthError = lengthError;
      for (unsigned int i=0; i<3; ++i)
        tip[i] = p[i];
    }
    intensity1 = intensity2;
  }
  return found;
}

bool usNeedleDetectionTools::findTipUsingMeanValues(vtkImageData *image, const vpMatrix &model, int *VOI, double *tip, unsigned int nPoints, double gap, double length)
{
  //std::cerr << "Finding tip..." << std::endl;
  vpColVector a(nPoints);
  vpColVector p0(3);
  vpColVector p(3);
  double t = 0.0;
  a[0] = 1;
  for (unsigned int i=1; i<nPoints; ++i)
    a[i] = a[i-1] * t;
  p0 = model * a;
  t = 1.0;
  a[0] = 1;
  for (unsigned int i=1; i<nPoints; ++i)
    a[i] = a[i-1] * t;
  p = model * a;
  double l = (p-p0).euclideanNorm();
  double minLengthError = DBL_MAX;
  t = 0.0;
  a[0] = 1;
  for (unsigned int i=1; i<nPoints; ++i)
    a[i] = a[i-1] * t;
  p = model * a;
  if (inside(p,VOI)) {
    bool found = 0;
    double intensity1 = 0;
    intensity1 += image->GetScalarComponentAsDouble(p[0], p[1], p[2], 0);
    intensity1 += image->GetScalarComponentAsDouble(p[0]+1.0, p[1], p[2], 0);
    intensity1 += image->GetScalarComponentAsDouble(p[0], p[1]+1.0, p[2], 0);
    intensity1 += image->GetScalarComponentAsDouble(p[0], p[1], p[2]+1.0, 0);
    intensity1 += image->GetScalarComponentAsDouble(p[0]-1.0, p[1], p[2], 0);
    intensity1 += image->GetScalarComponentAsDouble(p[0], p[1]-1.0, p[2], 0);
    intensity1 += image->GetScalarComponentAsDouble(p[0], p[1], p[2]-1.0, 0);
    intensity1 /= 7.0;
    while (inside(p,VOI)) {
      t += 1.0 / l;
      a[0] = 1;
      for (unsigned int i=1; i<nPoints; ++i)
        a[i] = a[i-1] * t;
      p = model * a;
      double intensity2 = 0;
      intensity2 += image->GetScalarComponentAsDouble(vpMath::round(p[0]), vpMath::round(p[1]), vpMath::round(p[2]), 0);
      intensity2 += image->GetScalarComponentAsDouble(vpMath::round(p[0])+1.0, vpMath::round(p[1]), vpMath::round(p[2]), 0);
      intensity2 += image->GetScalarComponentAsDouble(vpMath::round(p[0]), vpMath::round(p[1])+1.0, vpMath::round(p[2]), 0);
      intensity2 += image->GetScalarComponentAsDouble(vpMath::round(p[0]), vpMath::round(p[1]), vpMath::round(p[2])+1.0, 0);
      intensity2 += image->GetScalarComponentAsDouble(vpMath::round(p[0])-1.0, vpMath::round(p[1]), vpMath::round(p[2]), 0);
      intensity2 += image->GetScalarComponentAsDouble(vpMath::round(p[0]), vpMath::round(p[1])-1.0, vpMath::round(p[2]), 0);
      intensity2 += image->GetScalarComponentAsDouble(vpMath::round(p[0]), vpMath::round(p[1]), vpMath::round(p[2])-1.0, 0);
      intensity2 /= 7.0;
      double lengthError = abs((p-p0).euclideanNorm() - length);
      //std::cerr << "("<< p[0] << "," << p[1] << "," << p[2] << "): I=" << intensity2 << " dL=" << lengthError << std::endl;
      if (intensity1 - intensity2 > gap && lengthError < minLengthError) {
        found = true;
        minLengthError = lengthError;
        for (unsigned int i=0; i<3; ++i)
          tip[i] = p[i];
      }
      intensity1 = intensity2;
    }
    //std::cerr << " done." << std::endl;
    return found;
  }
  else
    return false;
}
#endif

bool usNeedleDetectionTools::inside(const vpColVector point, int *VOI) {
  return (vpMath::round(point[0]) > (VOI[0] + 1) && vpMath::round(point[0]) < (VOI[1] - 1)
      && vpMath::round(point[1]) > (VOI[2] + 1) && vpMath::round(point[1]) < (VOI[3] - 1)
      && vpMath::round(point[2]) > (VOI[4] + 1) && vpMath::round(point[2]) < (VOI[5] - 1));
}

#if defined USNEEDLEDETECTION_HAVE_VTK
unsigned int usNeedleDetectionTools::findTip(vtkDataArray *data, unsigned int dataSize,
                                             double threshL, double threshU)
{
  int start = -1;
  int stop = -1;
  bool keep = false;
  if (threshL>threshU)
  {
    std::cerr << "Error in findTip: threshU should be greater than or equal to threshL."
              << std::endl;
    return 0;
  }
  for (unsigned int i=0; i<dataSize; i++)
  {
    if (start==-1)
    {
      if (data->GetTuple(i)[0] >= threshL)
        start = i;
      if (data->GetTuple(i)[0] >= threshU)
        keep = true;
    }
    else if (keep && data->GetTuple(i)[0]<threshL)
    {
      stop = i-1;
      keep = false;
    }
    else if (!keep && data->GetTuple(i)[0]>threshU)
      keep = true;
  }
  if (keep)
    stop = dataSize-1;
  std::cerr << "Tip index: " << stop << std::endl;
  return stop;
}

unsigned int usNeedleDetectionTools::findTip(vtkDataArray *data, unsigned int dataSize,
                                             double threshL, double threshU, int prediction)
{
  int idx;
  int stop = -1;
  int *keep = new int[dataSize];
  int error = dataSize;
  if (threshL>threshU)
  {
    std::cerr << "Error in findTip: threshU should be greater than or equal to threshL."
              << std::endl;
    delete[] keep;
    return 0;
  }
  keep[0] = 1;
  for (unsigned int i=1; i<dataSize; i++)
    keep[i] = (data->GetTuple(i)[0]>threshU || (data->GetTuple(i)[0]>threshL && keep[i-1]));
  for (unsigned int i=0; i<dataSize; i++)
    std::cerr << keep[i] << " ";
  std::cerr << std::endl;
  for (idx=1; idx<dataSize; idx++)
    if ((keep[idx-1] && !keep[idx]) || (keep[idx] && idx==dataSize-1))
      if (vpMath::abs(idx-prediction)<=error)
      {
        error = vpMath::abs(idx-prediction);
        stop = idx;
      }
  std::cerr << "Tip index: " << stop << std::endl;
  delete[] keep;
  return stop;
}

vpList<double*> usNeedleDetectionTools::getThresholdedIds(vtkImageData *image,
                                                          double threshold)
{
  unsigned int npts = image->GetNumberOfPoints();
  vpList<double*> points = vpList<double*>();
  for (unsigned int i=0; i<npts; i++)
    if (((vtkUnsignedIntArray*)image->GetPointData()->GetScalars())->GetValue(i) > threshold)
      points.addRight(image->GetPoint(i));
  return points;
}

void usNeedleDetectionTools::getThresholdedCoordinates(vtkImageData *image,
                                                       vpMatrix &points,
                                                       double threshold)
{
  unsigned int npts = image->GetNumberOfPoints();
  unsigned int nThresholded = 0;
  if (image->GetScalarType() == VTK_UNSIGNED_CHAR)
  {
    unsigned char *uchar_ptr = (unsigned char*)image->GetScalarPointer();
    for (unsigned int i=0; i<npts; i++)
      if (*(uchar_ptr+i) >= threshold)
        ++nThresholded;
    points.resize(nThresholded, 3);
    unsigned int counter = 0;
    for (unsigned int i=0; i<npts; i++)
      if (*(uchar_ptr+i) >= threshold)
      {
        points[counter][0] = image->GetPoint(i)[0];
        points[counter][1] = image->GetPoint(i)[1];
        points[counter][2] = image->GetPoint(i)[2];
        counter++;
      }
  }
  else
  {
    short *short_ptr = (short*)image->GetScalarPointer();
    for (unsigned int i=0; i<npts; i++)
      if (*(short_ptr+i) >= threshold)
        ++nThresholded;
    points.resize(nThresholded, 3);
    unsigned int counter = 0;
    for (unsigned int i=0; i<npts; i++)
      if (*(short_ptr+i) >= threshold)
      {
        points[counter][0] = image->GetPoint(i)[0];
        points[counter][1] = image->GetPoint(i)[1];
        points[counter][2] = image->GetPoint(i)[2];
        counter++;
      }
  }
}
#endif
#if 0
double usNeedleDetectionTools::getThresholdedCoordinates(const usVolume<unsigned int> &V,
                                                         vpMatrix &points,
                                                         unsigned int nDesired)
{
  unsigned int npts = V.getSize();
  unsigned int nThresholded;
  unsigned int q;
  const unsigned int *data_ptr = V.getConstData();
  usNeedleDetectionTools::computeQuantile(data_ptr, npts, nDesired, q, nThresholded);
  points.resize(nThresholded, 3);
  unsigned int counter = 0;
  for (unsigned int z = 0; z < V.getDimZ(); ++z)
    for (unsigned int y = 0; y < V.getDimY(); ++y)
      for (unsigned int x = 0; x < V.getDimZ(); ++x)
        if (V(x,y,z) >= q)
        {
          points[counter][0] = x;
          points[counter][1] = y;
          points[counter][2] = z;
          ++counter;
        }
  return q;
}
#endif
#if defined USNEEDLEDETECTION_HAVE_VTK
double usNeedleDetectionTools::getThresholdedCoordinates(vtkImageData *image,
                                                         vpMatrix &points,
                                                         unsigned int nDesired)
{
  unsigned int npts = image->GetNumberOfPoints();
  unsigned int nThresholded;
  unsigned int q;
  if (image->GetScalarType() == VTK_UNSIGNED_CHAR)
  {
    unsigned char *uchar_ptr = (unsigned char*)image->GetScalarPointer();
    usNeedleDetectionTools::computeQuantile(uchar_ptr, npts, nDesired, q, nThresholded);
    points = vpMatrix(nThresholded, 3);
    unsigned int counter = 0;
    for (unsigned int i=0; i<npts; i++)
      if (*(uchar_ptr+i) >= q)
      {
        points[counter][0] = image->GetPoint(i)[0];
        points[counter][1] = image->GetPoint(i)[1];
        points[counter][2] = image->GetPoint(i)[2];
        counter++;
      }
  }
  else
  {
    short *short_ptr = (short*)image->GetScalarPointer();
    usNeedleDetectionTools::computeQuantile(short_ptr, npts, nDesired, q, nThresholded);
    points = vpMatrix(nThresholded, 3);
    unsigned int counter = 0;
    for (unsigned int i=0; i<npts; i++)
      if (*(short_ptr+i) >= q)
      {
        points[counter][0] = image->GetPoint(i)[0];
        points[counter][1] = image->GetPoint(i)[1];
        points[counter][2] = image->GetPoint(i)[2];
        counter++;
      }
  }
  return (double)q;
}
#endif

double usNeedleDetectionTools::dist3(double *x, double *y)
{
  double d = 0;
  for (int i=0; i<3; i++)
    d += (x[i]-y[i]) * (x[i]-y[i]);
  return sqrt(d);
}

double usNeedleDetectionTools::angle(double *p1, double *p2, double *q1, double *q2)
{
  // Returns the unsigned angle between two lines (in rad)
  vpColVector d1(3);
  vpColVector d2(3);
  for (unsigned int i=0; i<3; i++)
  {
    d1[i] = p1[i]-p2[i];
    d2[i] = q1[i]-q2[i];
  }
  double orientation = vpColVector::dotProd(d1,d2) / (d1.euclideanNorm() * d2.euclideanNorm());
  return acos(orientation);
}

void usNeedleDetectionTools::linearRegression(vpMatrix &points, double &Xm, double &Ym, double &Zm, double &u, double &v, double &w)
{
  unsigned int n = points.getRows();
  vpColVector X = points.getCol(0);
  vpColVector Y = points.getCol(1);
  vpColVector Z = points.getCol(2);
  Xm = vpColVector::mean(X);
  Ym = vpColVector::mean(Y);
  Zm = vpColVector::mean(Z);
  double Sxx = -Xm*Xm + X*X/n;
  double Sxy = -Xm*Ym + X*Y/n;
  double Syy = -Ym*Ym + Y*Y/n;
  double Sxz = -Xm*Zm + X*Z/n;
  double Szz = -Zm*Zm + Z*Z/n;
  double Syz = -Ym*Zm + Y*Z/n;
  double theta = atan2(2*Sxy,(Sxx-Syy))/2;
  double K11 = (Syy+Szz)*pow(cos(theta),2)+(Sxx+Szz)*pow(sin(theta),2)-2*Sxy*cos(theta)*sin(theta);
  double K22 = (Syy+Szz)*pow(sin(theta),2)+(Sxx+Szz)*pow(cos(theta),2)+2*Sxy*cos(theta)*sin(theta);
  //double K12 = -Sxy*(pow(cos(theta),2)-pow(sin(theta),2))+(Sxx-Syy)*cos(theta)*sin(theta);
  double K10 = Sxz*cos(theta)+Syz*sin(theta);
  double K01 = -Sxz*sin(theta)+Syz*cos(theta);
  double K00 = Sxx+Syy;
  double c2 = -K00-K11-K22;
  double c1 = K00*K11+K00*K22+K11*K22-K01*K01-K10*K10;
  double c0 = K01*K01*K11+K10*K10*K22-K00*K11*K22;
  //
  double p = c1-pow(c2,2)/3;
  double q = 2*pow(c2,3)/27-c1*c2/3+c0;
  double R = pow(q,2)/4+pow(p,3)/27;
  double delta2;
  if (R>0)
    delta2 = -c2/2+pow(sqrt(R)-q/2,1.0/3.0)+pow(-sqrt(R)-q/2,1.0/3.0);
  else
  {
    double rho = sqrt(-pow(p,3)/27);
    double phi = acos(-q/(2*rho));
    delta2 = vpMath::minimum(-c2/3+2*pow(rho,1.0/3.0)*cos(phi/3),
                             vpMath::minimum(-c2/3+2*pow(rho,1.0/3.0)*cos((phi+2*M_PI)/3),
                                             -c2/3+2*pow(rho,1.0/3.0)*cos((phi+4*M_PI)/3)));
  }
  //
  double a = -K10/(K11-delta2)*cos(theta)+K01/(K22-delta2)*sin(theta);
  double b = -K10/(K11-delta2)*sin(theta)-K01/(K22-delta2)*cos(theta);
  u = ((1+b*b)*Xm-a*b*Ym+a*Zm)/(1+a*a+b*b);
  v = (-a*b*Xm+(1+a*a)*Ym+b*Zm)/(1+a*a+b*b);
  w = (a*Xm+b*Ym+(a*a+b*b)*Zm)/(1+a*a+b*b);
}

void usNeedleDetectionTools::linearRegression(vpMatrix &points, double &a, double &b, double &u, double &v, double &w)
{
  unsigned int n = points.getCols();
  vpColVector X = points.getRow(0).t();
  vpColVector Y = points.getRow(1).t();
  vpColVector Z = points.getRow(2).t();
  double Xm = vpColVector::mean(X);
  double Ym = vpColVector::mean(Y);
  double Zm = vpColVector::mean(Z);
  double Sxx = -Xm*Xm + X*X/n;
  double Sxy = -Xm*Ym + X*Y/n;
  double Syy = -Ym*Ym + Y*Y/n;
  double Sxz = -Xm*Zm + X*Z/n;
  double Szz = -Zm*Zm + Z*Z/n;
  double Syz = -Ym*Zm + Y*Z/n;
  double theta = atan2(2*Sxy,(Sxx-Syy))/2;
  double K11 = (Syy+Szz)*pow(cos(theta),2)+(Sxx+Szz)*pow(sin(theta),2)-2*Sxy*cos(theta)*sin(theta);
  double K22 = (Syy+Szz)*pow(sin(theta),2)+(Sxx+Szz)*pow(cos(theta),2)+2*Sxy*cos(theta)*sin(theta);
  //double K12 = -Sxy*(pow(cos(theta),2)-pow(sin(theta),2))+(Sxx-Syy)*cos(theta)*sin(theta);
  double K10 = Sxz*cos(theta)+Syz*sin(theta);
  double K01 = -Sxz*sin(theta)+Syz*cos(theta);
  double K00 = Sxx+Syy;
  double c2 = -K00-K11-K22;
  double c1 = K00*K11+K00*K22+K11*K22-K01*K01-K10*K10;
  double c0 = K01*K01*K11+K10*K10*K22-K00*K11*K22;
  //
  double p = c1-pow(c2,2)/3;
  double q = 2*pow(c2,3)/27-c1*c2/3+c0;
  double R = pow(q,2)/4+pow(p,3)/27;
  double delta2;
  if (R>0)
    delta2 = -c2/2+pow(sqrt(R)-q/2,1.0/3.0)+pow(-sqrt(R)-q/2,1.0/3.0);
  else
  {
    double rho = sqrt(-pow(p,3)/27);
    double phi = acos(-q/(2*rho));
    delta2 = vpMath::minimum(-c2/3+2*pow(rho,1.0/3.0)*cos(phi/3),
                             vpMath::minimum(-c2/3+2*pow(rho,1.0/3.0)*cos((phi+2*M_PI)/3),
                                             -c2/3+2*pow(rho,1.0/3.0)*cos((phi+4*M_PI)/3)));
  }
  //
  a = -K10/(K11-delta2)*cos(theta)+K01/(K22-delta2)*sin(theta);
  b = -K10/(K11-delta2)*sin(theta)-K01/(K22-delta2)*cos(theta);
  u = ((1+b*b)*Xm-a*b*Ym+a*Zm)/(1+a*a+b*b);
  v = (-a*b*Xm+(1+a*a)*Ym+b*Zm)/(1+a*a+b*b);
  w = (a*Xm+b*Ym+(a*a+b*b)*Zm)/(1+a*a+b*b);
}

void usNeedleDetectionTools::computeControlPoints(const vpMatrix &model, vpMatrix &mss, double **points, const unsigned int &nPoints) {
  if (nPoints > 2) {
    vpMatrix extremities(2,3);
    for (unsigned int i=0; i<3; ++i) {
      extremities[0][i] = points[0][i];
      extremities[1][i] = points[nPoints-1][i];
    }
    vpMatrix tapprox = approximateCoordinates(extremities, mss, nPoints);
    vpColVector a(nPoints);
    vpColVector p(3);
    for (unsigned int i=1; i<nPoints-1; ++i) {
      double t = tapprox[1][0] + i * (tapprox[1][1] - tapprox[0][1]) / (nPoints - 1);
      a[0] = 1;
      for (unsigned int j=1; j<nPoints; ++j)
        a[j] = a[j-1] * t;
      p = model * a;
      for (unsigned int j=0; j<3; ++j)
        points[i][j] = p[j];
    }
  }
}

vpMatrix usNeedleDetectionTools::approximateCoordinates(vpMatrix X,
                                                        vpMatrix MSS,
                                                        unsigned int s)
{
  vpColVector P1 = MSS.getRow(0).t();
  vpColVector P2 = MSS.getRow(s-1).t();
  unsigned int n = X.getRows(); // Number of points
  unsigned int d = X.getCols(); // Dimension
  vpMatrix tapprox(s,n); // Approximate curvilinear coordinate
  vpColVector dirVec(d); // Direction vector
  vpColVector ndirVec(d); // Normalized direction vector
  for (unsigned int j=0; j<d; j++)
    dirVec[j] = (double)P2[j] - (double)P1[j];

  double ss = dirVec.sumSquare();
  if (ss == 0.0) {
    std::cerr << "Error: in usNeedleDetectionTools::approximateCoordinates(): "
              << "first and last points are equal." << std::endl;
    exit(EXIT_FAILURE);
  }
  ndirVec = dirVec / ss;
  for  (unsigned int j=0 ; j < n ; j++)
  {
    double tapps = 1.0;
    double tapp = 0.0;
    for (unsigned int k=0; k<d; k++)
    {
      tapp += (X[j][k] - P1[k]) * ndirVec[k];
    }
    tapprox[0][j] = tapps;
    for(unsigned i=1; i<s; i++)
    {
      tapps *= tapp;
      tapprox[i][j] = tapps;
    }
  }
  return tapprox;
}


bool usNeedleDetectionTools::intersectionLinePlane(vpMatrix line,
                                                   const vpColVector &plane, double offset,
                                                   double &x, double &y, double &z)
{
  vpColVector c1 = line.getCol(0);
  vpColVector c2 = line.getCol(1);
  double D = c2 * plane;
  if (D == 0.0)
    return false;
  vpColVector T(2);
  T[0] = 1.0;
  T[1] = - (c1 * plane + offset) / D;
  vpColVector I = line * T;
  x = I[0];
  y = I[1];
  z = I[2];
  return true;
}
