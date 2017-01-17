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
 * Authors:
 * Pierre Chatelain
 *
 *****************************************************************************/

/**
 * @file usRectangle.h
 * @brief Rectangle.
 */

#ifndef __usRectangle_h_
#define __usRectangle_h_

#include <visp3/core/vpConfig.h>
/**
 * @class usRectangle
 * @brief Representation of a rectangle.
 * @ingroup module_ustk_core
 *
 * This class implements a (possibly inclined) rectangle of the plane.
 */
class VISP_EXPORT usRectangle {
 public:
  /// Default constructor.
  usRectangle();

  /** Constructor.
   * @param cx The center x-axis.
   * @param cy The center y-axis.
   * @param width The rectangle width.
   * @param height The rectangle height.
   * @param theta The rectangle orientation (rad).
   */
  usRectangle(double cx, double cy, double width, double height, double theta);

  /// Set the center of the rectangle.
  void setCenter(double cx, double cy);

  /** Set the corners of the rectangle.
   *
   *  @warning This method doesn't check whether the 4 points actually form a rectangle!
   *  The behaviour is undefined if it is not the case.
   */
  void setPoints(double x1, double y1, double x2, double y2,
		 double x3, double y3, double x4, double y4);

  /// Get the center x-axis.
  double getCx() const;

  /// Get the center y-axis.
  double getCy() const;

  /// Get the top-left corner x-axis.
  double getX1() const;

  /// Get the bottom-left corner x-axis.
  double getX2() const;

  /// Get the bottom-right corner x-axis.
  double getX3() const;

  /// Get the top-right corner x-axis.
  double getX4() const;

  /// Get the top-left corner y-axis.
  double getY1() const;

  /// Get the bottom-left corner y-axis.
  double getY2() const;

  /// Get the bottom-right corner y-axis.
  double getY3() const;

  /// Get the top-right corner y-axis.
  double getY4() const;

  /// Set the size of the rectangle.
  void setSize(double w, double h);

  /// Get the rectangle width.
  double getWidth() const;

  /// Get the rectangle height.
  double getHeight() const;

  /// Set the rectangle orientation (rad).
  void setOrientation(double theta);

  /// Get the rectangle orientation (rad).
  double getOrientation() const;

  /// Check whether the point (x,y) is inside the rectangle.
  bool isInside(double x, double y) const;

 private:
  double m_cx;
  double m_cy;
  double m_width;
  double m_height;
  double m_theta;
  double m_x1;
  double m_x2;
  double m_x3;
  double m_x4;
  double m_y1;
  double m_y2;
  double m_y3;
  double m_y4;
  bool isLeft(double x, double y, double x1, double y1, double x2, double y2) const;
};

#endif // US_RECTANGLE_H

