/****************************************************************************
 *
 * This file is part of the ustk software.
 * Copyright (C) 2016 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ustk with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at ustk@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Author:
 * Jason Chevrie
 *
 *****************************************************************************/

#include <visp3/ustk_core/usGeometryTools.h>

#include <limits>

#include <visp3/core/vpException.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpTranslationVector.h>

namespace usGeometryTools
{

double getPointPlaneDistance(const vpColVector &point, const usOrientedPlane3D &plane)
{
  vpColVector p(plane.getPosition());
  vpColVector d(plane.getDirection());
  return vpColVector::dotProd(d, point - p);
}

bool IsPointInFrontOfPlane(const vpColVector &point, const usOrientedPlane3D &plane)
{
  return getPointPlaneDistance(point, plane) >= 0;
}

bool DoesSegmentCrossPlane(const usPolynomialCurve3D &poly, const usOrientedPlane3D &plane)
{
  return (IsPointInFrontOfPlane(poly.getStartPoint(), plane) != IsPointInFrontOfPlane(poly.getEndPoint(), plane));
}

bool DoesSegmentCrossPlane(const usBSpline3D &spline, const usOrientedPlane3D &plane)
{
  /*int N = spline.getNbSegments();
  for(int i=0 ; i<N ; i++)
  {
      if(DoesSegmentCrossPlane(spline.accessSegment(i), plane)) return true;
  }
  return false;*/
  return (IsPointInFrontOfPlane(spline.accessSegment(0).getStartPoint(), plane) !=
          IsPointInFrontOfPlane(spline.accessLastSegment().getEndPoint(), plane));
}

bool DoesSegmentCrossPlaneDirect(const usPolynomialCurve3D &poly, const usOrientedPlane3D &plane)
{
  return (!IsPointInFrontOfPlane(poly.getStartPoint(), plane) && IsPointInFrontOfPlane(poly.getEndPoint(), plane));
}

bool DoesSegmentCrossPlaneDirect(const usBSpline3D &spline, const usOrientedPlane3D &plane)
{
  /*int N = spline.getNbSegments();
  for(int i=0 ; i<N ; i++)
  {
      if(DoesSegmentCrossPlaneDirect(spline.accessSegment(i), plane)) return true;
  }
  return false;*/
  return (!IsPointInFrontOfPlane(spline.accessSegment(0).getStartPoint(), plane) &&
          IsPointInFrontOfPlane(spline.accessLastSegment().getEndPoint(), plane));
}

vpColVector getPlaneCurveCrossingPoint(const usPolynomialCurve3D &poly, const usOrientedPlane3D &plane,
                                       double threshold, double *t)
{
  if (threshold <= 0)
    threshold = 1e-4 * poly.getParametricLength();

  double start = poly.getStartParameter();
  vpColVector Pstart(poly.getPoint(start));
  bool sideStart = IsPointInFrontOfPlane(Pstart, plane);
  double end = poly.getEndParameter();
  vpColVector Pend(poly.getPoint(end));
  bool sideEnd = IsPointInFrontOfPlane(Pend, plane);
  double mid = (start + end) / 2;
  vpColVector Pmid(poly.getPoint(mid));
  bool sideMid = IsPointInFrontOfPlane(Pmid, plane);

  double d = (Pend - Pstart).euclideanNorm();

  while ((sideStart != sideEnd) && (d > threshold)) {
    if (sideMid == sideStart) {
      start = mid;
      Pstart = Pmid;
    } else if (sideMid == sideEnd) {
      end = mid;
      Pend = Pmid;
    }

    mid = (start + end) / 2;
    Pmid = poly.getPoint(mid);
    sideMid = IsPointInFrontOfPlane(Pmid, plane);
    d = (Pend - Pstart).euclideanNorm();
  }

  if (sideStart == sideEnd)
    throw vpException(
        vpException::badValue,
        "usGeometryTools::getPlaneCurveCrossingPoint(usPolynomialCurve3D): segment and plane do not cross");

  if (t)
    *t = mid;
  return poly.getPoint(mid);
}

vpColVector getPlaneCurveCrossingPoint(const usBSpline3D &spline, const usOrientedPlane3D &plane, double threshold,
                                       double *t)
{
  int seg = 0;
  double L = 0;
  int N = spline.getNbSegments();
  /*while(seg < N && !usGeometryTools::DoesSegmentCrossPlane(spline.accessSegment(seg), plane))
  {
      L += spline.accessSegment(seg).getParametricLength();
      seg++;
  }*/

  if (N > 1) {
    int start = 0;
    vpColVector Pstart(spline.accessSegment(start).getStartPoint());
    bool sideStart = IsPointInFrontOfPlane(Pstart, plane);
    int end = N - 1;
    vpColVector Pend(spline.accessSegment(end).getStartPoint());
    bool sideEnd = IsPointInFrontOfPlane(Pend, plane);
    int mid = (start + end) / 2;
    vpColVector Pmid(spline.accessSegment(mid).getStartPoint());
    bool sideMid = IsPointInFrontOfPlane(Pmid, plane);

    if (sideStart == sideEnd)
      seg = N - 1;
    else {
      while (end - start > 1) {
        if (sideMid == sideStart) {
          start = mid;
          Pstart = Pmid;
        } else if (sideMid == sideEnd) {
          end = mid;
          Pend = Pmid;
        }

        mid = (start + end) / 2;
        Pmid = spline.accessSegment(mid).getStartPoint();
        sideMid = IsPointInFrontOfPlane(Pmid, plane);
      }
      seg = start;
    }
    for (int i = 0; i < seg; i++)
      L += spline.accessSegment(i).getParametricLength();
  } /**/

  if (seg >= N) {
    throw vpException(vpException::badValue,
                      "usGeometryTools::getPlaneCurveCrossingPoint(usBSpline3D): spline and plane do not cross");
  } else {
    double t1 = -1;
    vpColVector P = getPlaneCurveCrossingPoint(spline.accessSegment(seg), plane, threshold, &t1);
    if (t)
      *t = L + t1;
    return P;
  }
}

vpColVector projectPointOnPlane(const vpColVector &point, const usOrientedPlane3D &plane, const vpColVector &direction)
{
  vpColVector p(plane.getPosition());
  vpColVector d(plane.getDirection());
  if (direction.getCols() != 3) {
    return (point - vpColVector::dotProd(point - p, d) * d);
  } else {
    vpColVector dp = direction;
    dp.normalize();
    double cos_theta = vpColVector::dotProd(dp, d);
    if (fabs(cos_theta) < std::numeric_limits<double>::epsilon()) {
      return (point - vpColVector::dotProd(point - p, d) * d);
    } else {
      return (point - vpColVector::dotProd(point - p, d) / cos_theta * dp);
    }
  }
}

vpColVector projectPointOnCurve(const vpColVector &point, const usPolynomialCurve3D &poly, double threshold, double *t)
{
  if (point.size() != 3)
    throw vpException(vpException::dimensionError, "usGeometryTools::projectPointOnCurve: invalid point dimension");

  double start = poly.getStartParameter();
  double stop = poly.getEndParameter();

  if (threshold <= 0)
    threshold = fabs(stop - start) / 1000;

  double middle = (start + stop) / 2;
  while ((stop - start) > threshold) {
    double d0 = (poly.getPoint(start) - point).euclideanNorm();
    double d1 = (poly.getPoint(middle) - point).euclideanNorm();
    double d2 = (poly.getPoint(stop) - point).euclideanNorm();

    if (d0 <= d1 && d0 < d2)
      stop = middle;
    else if (d2 < d0 && d2 <= d1)
      start = middle;
    else {
      start = (start + middle) / 2;
      stop = (middle + stop) / 2;
    }
    middle = (start + stop) / 2;
  }

  vpColVector P = poly.getPoint(middle);
  if (t)
    *t = middle;

  return P;
}

vpColVector projectPointOnCurve(const vpColVector &point, const usBSpline3D &spline, double threshold, int *index,
                                double *t)
{
  if (point.size() != 3)
    throw vpException(vpException::dimensionError, "usGeometryTools::projectPointOnCurve: invalid point dimension");

  int seg = 0;
  double min = std::numeric_limits<double>::infinity();
  for (int i = 0; i < spline.getNbSegments(); i++) {
    double d = (spline.accessSegment(i).getStartPoint() - point).sumSquare();
    if (d < min) {
      min = d;
      seg = i;
    }
  }

  if ((spline.accessLastSegment().getEndPoint() - point).sumSquare() < min) {
    seg = spline.getNbSegments();
  }

  if (index)
    *index = seg;
  if (seg == 0)
    return projectPointOnCurve(point, spline.accessSegment(seg), threshold, t);
  else if (seg == spline.getNbSegments()) {
    seg--;
    if (index)
      *index = seg;
    return projectPointOnCurve(point, spline.accessSegment(seg), threshold, t);
  } else {
    double t1;
    double t2;
    vpColVector P1 = projectPointOnCurve(point, spline.accessSegment(seg - 1), threshold, &t1);
    vpColVector P2 = projectPointOnCurve(point, spline.accessSegment(seg), threshold, &t2);

    if ((P1 - point).sumSquare() < (P2 - point).sumSquare()) {
      if (t)
        *t = t1;
      return P1;
    } else {
      if (t)
        *t = t2;
      return P2;
    }
  }
}

usOrientedPlane3D getNormalPlane(const usPolynomialCurve3D &p, double l)
{
  return usOrientedPlane3D(p.getPoint(l), p.getTangent(l));
}

usPolynomialCurve3D convertBSplineToPolynomial(const usBSpline3D &spline, int order)
{
  double L = spline.getParametricLength();
  int nbPoints = order + 1;
  int nbSegments = order;
  std::vector<vpColVector> p;
  std::vector<double> l;
  for (int i = 0; i < nbPoints; i++)
    p.push_back(spline.getPoint(i * L / nbSegments));
  l.push_back(0);
  for (int i = 0; i < nbSegments; i++)
    l.push_back(l.back() + (p.at(i + 1) - p.at(i)).euclideanNorm());

  usPolynomialCurve3D P(order);
  P.defineFromPoints(p, l, order);

  return P;
}

usBSpline3D convertPolynomialToBSpline(const usPolynomialCurve3D &poly, int nbSegments, int order)
{
  double L = poly.getParametricLength();
  int nbPoints = nbSegments + 1;
  std::vector<vpColVector> p;
  std::vector<double> l;
  for (int i = 0; i < nbPoints; i++)
    p.push_back(poly.getPoint(i * L / nbSegments));
  for (int i = 0; i < nbSegments; i++)
    l.push_back((p.at(i + 1) - p.at(i)).euclideanNorm());

  usBSpline3D S;
  S.defineFromPoints(p, l, order);

  return S;
}

vpPoseVector findPointCloudRelativePose(std::vector<vpColVector> &P1, std::vector<vpColVector> &P2, double *res)
{
  unsigned int nbPoints = P1.size();
  if (nbPoints < 2)
    throw vpException(vpException::dimensionError, "usGeometryTools::findPointCloudRelativePose: not enough points");
  if (nbPoints != P2.size())
    throw vpException(vpException::dimensionError,
                      "usGeometryTools::findPointCloudRelativePose: non matching number of points");

  vpColVector Mean1(3, 0);
  vpColVector Mean2(3, 0);

  for (unsigned int i = 0; i < nbPoints; i++) {
    Mean1 += P1.at(i);
    Mean2 += P2.at(i);
  }
  Mean1 /= nbPoints;
  Mean2 /= nbPoints;

  vpMatrix H(3, 3, 0);
  for (unsigned int i = 0; i < nbPoints; i++) {
    H += (P1.at(i) - Mean1) * (P2.at(i) - Mean2).t();
  }

  vpColVector w;
  vpMatrix V;
  H.svd(w, V);

  vpMatrix Rm = V * H.t();
  if (Rm.det() < 0) {
    for (int i = 0; i < 3; i++)
      V[i][2] *= -1;
    Rm = V * H.t();
  }
  vpRotationMatrix R;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      R[i][j] = Rm[i][j];
    }
  }

  vpTranslationVector T(Mean2 - R * Mean1);

  vpPoseVector p(T, R);

  if (res != nullptr) {
    *res = 0;
    for (unsigned int i = 0; i < nbPoints; i++) {
      *res += (R * P1.at(i) + T - (vpTranslationVector)P2.at(i)).sumSquare();
    }
  }

  return p;
}

vpTranslationVector findPointCloudRelativePosition(std::vector<vpColVector> &P1, std::vector<vpColVector> &P2,
                                                   const vpRotationMatrix &worldRprobe, double *res)
{
  unsigned int nbPoints = P1.size();
  if (nbPoints < 2)
    throw vpException(vpException::dimensionError, "usGeometryTools::findPointCloudRelativePose: not enough points");
  if (nbPoints != P2.size())
    throw vpException(vpException::dimensionError,
                      "usGeometryTools::findPointCloudRelativePose: non matching number of points");

  vpColVector Mean1(3, 0);
  vpColVector Mean2(3, 0);

  for (unsigned int i = 0; i < nbPoints; i++) {
    Mean1 += P1.at(i);
    Mean2 += P2.at(i);
  }
  Mean1 /= nbPoints;
  Mean2 /= nbPoints;

  vpTranslationVector T(Mean2 - worldRprobe * Mean1);

  if (res != nullptr) {
    *res = 0;
    for (unsigned int i = 0; i < nbPoints; i++) {
      *res += (worldRprobe * P1.at(i) + T - (vpTranslationVector)P2.at(i)).sumSquare();
    }
  }

  return T;
}

vpRotationMatrix findPointCloudRelativeRotation(std::vector<vpColVector> &P1, std::vector<vpColVector> &P2, double *res)
{
  unsigned int nbPoints = P1.size();
  if (nbPoints < 2)
    throw vpException(vpException::dimensionError,
                      "usGeometryTools::findPointCloudRelativeRotation: not enough points");
  if (nbPoints != P2.size())
    throw vpException(vpException::dimensionError,
                      "usGeometryTools::findPointCloudRelativeRotation: non matching number of points");

  vpMatrix H(3, 3, 0);
  for (unsigned int i = 0; i < nbPoints; i++)
    H += P1.at(i) * P2.at(i).t();

  vpColVector w;
  vpMatrix V;
  H.svd(w, V);

  vpMatrix Rm = V * H.t();
  if (Rm.det() < 0) {
    for (int i = 0; i < 3; i++)
      V[i][2] *= -1;
    Rm = V * H.t();
  }
  vpRotationMatrix R;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      R[i][j] = Rm[i][j];
    }
  }

  if (res != nullptr) {
    *res = 0;
    for (unsigned int i = 0; i < nbPoints; i++) {
      *res += (R * P1.at(i) - P2.at(i)).sumSquare();
    }
  }

  return R;
}

vpPoseVector ICPPointCloudRelativePose(std::vector<vpColVector> &P1, std::vector<vpColVector> &P2, double *res,
                                       vpPoseVector *initialGuess)
{
  if (P1.size() < 1 || P2.size() < 1)
    throw vpException(vpException::dimensionError,
                      "usGeometryTools::ICPPointCloudRelativePose: not enough points to match");

  vpTranslationVector T;
  vpRotationMatrix R;

  if (initialGuess) {
    T = initialGuess->getTranslationVector();
    R = initialGuess->getRotationMatrix();
  } else {
    vpColVector meanP1(3, 0);
    for (unsigned int i = 0; i < P1.size(); i++)
      meanP1 += P1[i];
    meanP1 /= (double)P1.size();
    vpColVector meanP2(3, 0);
    for (unsigned int i = 0; i < P2.size(); i++)
      meanP2 += P2[i];
    meanP2 /= (double)P2.size();
    T = meanP2 - meanP1;
  }

  vpMatrix dist(P1.size(), P2.size(), 0);
  std::vector<vpColVector> matchPoint(P1.size(), vpColVector(3));
  double residual = 0;
  double resRatio = 1;
  int it = 0;

  while ((it < 10) && (resRatio > 0.01)) {
    for (unsigned int i = 0; i < P1.size(); i++) {
      for (unsigned int j = 0; j < P2.size(); j++) {
        dist[i][j] = ((R * P1.at(i) + T) - (vpTranslationVector)P2.at(j)).sumSquare();
      }
    }

    for (unsigned int i = 0; i < P1.size(); i++) {
      int index = 0;
      double min = std::numeric_limits<double>::max();
      for (unsigned int j = 0; j < P2.size(); j++) {
        if (dist[i][j] < min) {
          min = dist[i][j];
          index = j;
        }
      }
      matchPoint.at(i) = P2.at(index);
    }

    double resi = 0;
    vpPoseVector pose(findPointCloudRelativePose(P1, matchPoint, &resi));
    T = pose.getTranslationVector();
    R = pose.getRotationMatrix();
    if (residual > 0)
      resRatio = fabs(residual - resi) / residual;
    else
      resRatio = 1;
    residual = resi;
  }

  if (res)
    *res = residual;

  return vpPoseVector(T, R);
}

vpTranslationVector ICPPointCloudRelativePosition(std::vector<vpColVector> &P1, std::vector<vpColVector> &P2,
                                                  const vpRotationMatrix &worldRprobe, double *res,
                                                  vpTranslationVector *initialGuess)
{
  (void)P1;
  (void)P2;
  (void)worldRprobe;
  throw vpException(vpException::notImplementedError,
                    "usGeometryTools::ICPPointCloudRelativePosition: non implement yet");
}

vpRotationMatrix ICPPointCloudRelativeRotation(std::vector<vpColVector> &P1, std::vector<vpColVector> &P2, double *res,
                                               vpRotationMatrix *initialGuess)
{
  (void)P1;
  (void)P2;
  throw vpException(vpException::notImplementedError,
                    "usGeometryTools::ICPPointCloudRelativeRotation: non implement yet");
}

bool fitCircleTo2DPointCloud(const std::vector<vpColVector> &points, vpColVector &center, double &r)
{
  // 2D nonlinear least square fitting (Coope93)
  unsigned int nbPoints = points.size();
  if (nbPoints < 3)
    throw vpException(vpException::ioError, "usGeometryTools::fitCircleTo2DPointCloud: not enough points");

  // Create data matrix with centered vectors
  vpMatrix M(3, nbPoints);
  vpColVector mean(2, 0);

  for (unsigned int i = 0; i < nbPoints; i++)
    mean += points.at(i);
  mean /= nbPoints;

  vpColVector d(nbPoints);
  for (unsigned int i = 0; i < nbPoints; i++) {
    vpColVector v(points.at(i) - mean);
    M.insert(v, 0, i);
    d[i] = v.sumSquare();
  }

  vpColVector x(nbPoints, 1);
  vpMatrix B(nbPoints, 3);
  B.insert(M.t(), 0, 0);
  B.insert(x, 0, 2);

  vpColVector y = B.pseudoInverse(0) * d;

  center.resize(2);
  center[0] = y[0] / 2;
  center[1] = y[1] / 2;

  r = sqrt(y[2] + pow(center.euclideanNorm(), 2));

  center += mean;

  return true;
}

bool fitSphereTo3DPointCloud(const std::vector<vpColVector> &points, vpColVector &center, double &r)
{
  // 2D nonlinear least square fitting (Coope93)
  unsigned int nbPoints = points.size();
  if (nbPoints < 3)
    throw vpException(vpException::ioError, "usGeometryTools::fitCircleTo2DPointCloud: not enough points");

  // Create data matrix with centered vectors
  vpMatrix M(4, nbPoints);
  vpColVector mean(3, 0);

  for (unsigned int i = 0; i < nbPoints; i++)
    mean += points.at(i);
  mean /= nbPoints;

  vpColVector d(nbPoints);
  for (unsigned int i = 0; i < nbPoints; i++) {
    vpColVector v(points.at(i) - mean);
    M.insert(v, 0, i);
    d[i] = v.sumSquare();
  }

  vpColVector x(nbPoints, 1);
  vpMatrix B(nbPoints, 4);
  B.insert(M.t(), 0, 0);
  B.insert(x, 0, 3);

  vpColVector y = B.pseudoInverse(0) * d;

  center.resize(3);
  center[0] = y[0] / 2;
  center[1] = y[1] / 2;
  center[2] = y[2] / 2;

  r = sqrt(y[3] + pow(center.euclideanNorm(), 2));

  center += mean;

  return true;
}

} // namespace usGeometryTools
