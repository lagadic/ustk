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
 * Author:
 * Jason Chevrie
 *
 *****************************************************************************/

/*!
  \example testUsOrientedPlane3D.cpp

  USTK usOrientedPlane3D test

  This example tests all functions declared in the usOrientedPlane3D class.
*/

#include <visp3/ustk_core/usOrientedPlane3D.h>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMatrix.h>

int main()
{
  std::cout << "Start testUsOrientedPlane3D" << std::endl;
  usOrientedPlane3D plane;
  std::cout << "done: usOrientedPlane3D default contructor" << std::endl;
  usOrientedPlane3D plane2(plane);
  std::cout << "done: usOrientedPlane3D copy contructor" << std::endl;
  vpPoseVector pose(1, 2, 3, 0.5, 1, 2);
  usOrientedPlane3D plane3(pose);
  std::cout << "done: usOrientedPlane3D contructor from pose" << std::endl;
  vpColVector position(3, 1);
  vpColVector direction(3, 0);
  direction[1] = 1;
  usOrientedPlane3D plane4(position, direction);
  std::cout << "done: usOrientedPlane3D contructor from position and direction" << std::endl;
  plane2 = plane4;
  std::cout << "done: usOrientedPlane3D assignment operator" << std::endl;

  plane.setPose(pose);
  std::cout << "done: usOrientedPlane3D::setPose" << std::endl;
  plane.getPose();
  std::cout << "done: usOrientedPlane3D::getPose" << std::endl;

  plane.setPosition(position);
  std::cout << "done: usOrientedPlane3D::setPosition" << std::endl;
  plane.getPosition();
  std::cout << "done: usOrientedPlane3D::getPosition" << std::endl;

  plane.setDirection(direction);
  std::cout << "done: usOrientedPlane3D::setDirection" << std::endl;
  plane.getDirection();
  std::cout << "done: usOrientedPlane3D::getDirection" << std::endl;

  vpHomogeneousMatrix H(1, 2, 3, 0.5, 1, 2);
  plane.moveInLocalFrame(H);
  std::cout << "done: usOrientedPlane3D::moveInLocalFrame(const vpHomogeneousMatrix&)" << std::endl;
  plane.moveInLocalFrame(1, 2, 3, 0.5, 1, 2);
  std::cout << "done: usOrientedPlane3D::moveInLocalFrame(double, double, double, double, double, double)" << std::endl;
  plane.moveInWorldFrame(H);
  std::cout << "done: usOrientedPlane3D::moveInWorldFrame(const vpHomogeneousMatrix&)" << std::endl;
  plane.moveInWorldFrame(1, 2, 3, 0.5, 1, 2);
  std::cout << "done: usOrientedPlane3D::moveInWorldFrame(double, double, double, double, double, double)" << std::endl;

  return 0;
}
