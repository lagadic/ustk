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
 * Marc Pouliquen
 *
 *****************************************************************************/
#include <iostream>
#include <math.h>
#include <visp3/core/vpXmlParserRectOriented.h>
#include <visp3/ustk_core/usImagePostScan2D.h>
#include <visp3/ustk_core/usSequenceReader.h>
#include <visp3/ustk_template_tracking/usDenseTracker2D.h>

#if defined(VISP_HAVE_XML2)

bool compare(int number1, int number2) { return (number2 == number1); }

int main()
{
  std::string sequenceFilename;

  // Get the ustk-dataset package path or USTK_DATASET_PATH environment variable value
  std::string env_ipath = us::getDataSetPath();
  if (!env_ipath.empty()) {
    sequenceFilename = env_ipath + "/post-scan/trackingSequence2D/trackingSequence.xml";
  }
  else {
    std::cout << "You should set USTK_DATASET_PATH environment var to access to ustk dataset" << std::endl;
    return 0;
  }

  // image
  usImagePostScan2D<unsigned char> image;

  // sequence reader init
  usSequenceReader<usImagePostScan2D<unsigned char> > reader;
  reader.setSequenceFileName(sequenceFilename);
  reader.acquire(image);

  vpImagePoint p(198, 230);
  vpRectOriented rectangle(p, 100, 50), groundTruthRectangle;

  // tracker init
  usDenseTracker2D tracker;
  tracker.init(image, rectangle);

  std::string genericFilenameXmlRect = env_ipath + "/post-scan/trackingSequence2D/test/groundTruthRectangle%04d.xml";

  vpXmlParserRectOriented xmlSettingsReader;

  unsigned int iteration = 0;
  while (!reader.end()) {
    // run tracker on new image
    reader.acquire(image);
    tracker.update(image);
    rectangle = tracker.getTarget();

    // get ground truth value
    char buffer[FILENAME_MAX];
    snprintf(buffer, FILENAME_MAX, genericFilenameXmlRect.c_str(), iteration + 1);
    xmlSettingsReader.parse(std::string(buffer));
    groundTruthRectangle = xmlSettingsReader.getRectangle();

    // compare (rounded values)
    if (!compare(rectangle.getCenter().get_i(), groundTruthRectangle.getCenter().get_i()) ||
        !compare(rectangle.getCenter().get_j(), groundTruthRectangle.getCenter().get_j()) ||
        !compare(rectangle.getHeight(), groundTruthRectangle.getHeight()) ||
        !compare(rectangle.getWidth(), groundTruthRectangle.getWidth()) ||
        !compare(rectangle.getOrientation(), groundTruthRectangle.getOrientation())) {
      std::cout << "Tracked rectangle differ from ground truth at iteration " << iteration << std::endl;
      return 1;
    }
    iteration++;
  }
  std::cout << "Test passed\n";
  return 0;
}
#else
int main()
{
  std::cout << "This test requires xml2 to run.\n";
  return 0;
}
#endif
