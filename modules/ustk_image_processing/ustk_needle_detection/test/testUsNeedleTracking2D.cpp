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
 * Marc Pouliquen
 *
 *****************************************************************************/

#include <iostream>
#include <sstream>
#include <string>

// visp
#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_XML2
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>

// ustk
#include <visp3/ustk_core/us.h>
#include <visp3/ustk_core/usSequenceReader.h>
#include <visp3/ustk_needle_detection/usNeedleTrackerSIR2D.h>

int main()
{
  std::string xml_filename;

  // Get the ustk-dataset package path or USTK_DATASET_PATH environment variable value
  if (xml_filename.empty()) {
    std::string env_ipath = us::getDataSetPath();
    if (!env_ipath.empty())
      xml_filename = env_ipath + "/needle/water_bath_minimal_noise_png/sequence.xml";
    else {
      std::cout << "You should set USTK_DATASET_PATH environment var to access to ustk dataset" << std::endl;
      return 0;
    }
  }

  ///////////////////////////////////////////////////////////////////////
  //
  // Initializations.
  //
  ///////////////////////////////////////////////////////////////////////

  usSequenceReader<usImagePostScan2D<unsigned char> > reader;
  reader.setSequenceFileName(xml_filename);

  // Read the first image
  usImagePostScan2D<unsigned char> I;
  reader.acquire(I);

  // Initialize the needle model
  usPolynomialCurve2D needle(2);
  vpMatrix controlPoints(2, 2);

  // initial needle position for needle insertion sequence of ustk-dataset repo
  controlPoints[0][0] = 197;
  controlPoints[1][0] = 218;
  controlPoints[0][1] = 211;
  controlPoints[1][1] = 236;
  needle.setControlPoints(controlPoints.t());

  // Initialization of the needle detector
  usNeedleTrackerSIR2D needleDetector;

  unsigned int nControlPoints = 2;
  unsigned int nParticles = 200;

  needleDetector.setSigma(1.0);
  needleDetector.setSigma1(10.0);
  needleDetector.setSigma2(1.0);
  needleDetector.init(I, nControlPoints, nParticles, needle);
  std::cout << "Needle detector initialized." << std::endl;

  controlPoints = needleDetector.getNeedle()->getControlPoints();

  vpColVector entryPose;

  ///////////////////////////////////////////////////////////////////////
  //
  // Start needle detection.
  //
  ///////////////////////////////////////////////////////////////////////

  vpColVector tipMean;

  // ground truth values
  std::vector<vpColVector> tipGroundTruth;
  vpColVector vec(2);
  vec[0] = 210;
  vec[1] = 235;
  tipGroundTruth.push_back(vec);
  vec[1] = 234;
  tipGroundTruth.push_back(vec);
  vec[0] = 218;
  vec[1] = 242;
  tipGroundTruth.push_back(vec);
  vec[0] = 224;
  vec[1] = 248;
  tipGroundTruth.push_back(vec);
  vec[0] = 229;
  vec[1] = 252;
  tipGroundTruth.push_back(vec);
  vec[0] = 232;
  vec[1] = 254;
  tipGroundTruth.push_back(vec);
  vec[0] = 234;
  vec[1] = 255;
  tipGroundTruth.push_back(vec);
  vec[0] = 237;
  vec[1] = 256;
  tipGroundTruth.push_back(vec);
  vec[0] = 243;
  vec[1] = 259;
  tipGroundTruth.push_back(vec);

  for (int i = 0; i < 9; i++) {
    reader.acquire(I);
    needleDetector.run(I, 0.0);

    tipMean = needleDetector.getNeedle()->getPoint(1.0);
    entryPose = needleDetector.getNeedle()->getPoint(0.0);
    std::cout << "Tip position: (" << tipMean[0] << "," << tipMean[1] << ")" << std::endl;
    std::cout << "Needle length: " << needleDetector.getNeedle()->getLength() << std::endl;
    std::cout << "Number of control points: " << needleDetector.getNeedle()->getOrder() << std::endl;

    // Output
    if (std::abs(tipMean[0] - tipGroundTruth.at(i)[0]) > 2 || std::abs(tipMean[1] - tipGroundTruth.at(i)[1]) > 2) {
      std::cout << "Error exeeds the test threshold(2), i error=" << std::abs(tipMean[0] - tipGroundTruth.at(i)[0])
                << ", j error= " << std::abs(tipMean[1] - tipGroundTruth.at(i)[1]) << std::endl;

      return 1;
    }
  }
  tipGroundTruth.clear();
  std::cout << "toto\n";

  return 0;
}

#else
int main() { std::cout << "You should install xml2 to use this example" << std::endl; }
#endif
