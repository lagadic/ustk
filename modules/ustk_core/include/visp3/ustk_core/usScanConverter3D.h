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
 * Jason Chevrie
 * Marc Pouliquen
 *
 *****************************************************************************/

/**
 * @file usScanConverter3D.h
 * @brief 3D scan-converter
 */

#ifndef __usScanConverter3D_h_
#define __usScanConverter3D_h_

#include <cmath>
#include <vector>

#include <visp3/ustk_core/usImagePreScan3D.h>
#include <visp3/ustk_core/usImagePostScan3D.h>


/**
 * @class usScanConverter3D
 * @brief 3D scan converter
 * @ingroup module_ustk_core
 *
 * This class allows to convert 3D pre-scan ultrasound images to post-scan.
 * The converter should be initialized through init() and then applied through convert().
 * This class accepts only images acquired by a convex transducer and a tilting motor for now.
 *
 * @warning Converting with this class uses a lot of RAM when computing the LUTs in init().
 */
class VISP_EXPORT usScanConverter3D
{
protected:
    class VoxelWeightAndIndex
    {
        friend class usScanConverter3D;
        unsigned int _outputIndex;
        unsigned int _inputIndex[8];
        double _W[8];
    };

    std::vector<VoxelWeightAndIndex> _lookupTable1;
    std::vector<VoxelWeightAndIndex> _lookupTable2;

    usImagePreScan3D<unsigned char> _VpreScan;
    usImagePostScan3D<unsigned char> _VpostScan;

    double _resolution;
    bool _SweepInZdirection;

    unsigned int m_nbX;
    unsigned int m_nbY;
    unsigned int m_nbZ;

public:

    usScanConverter3D();
    usScanConverter3D(const usImagePreScan3D<unsigned char> &preScanImage, int down);
    virtual ~usScanConverter3D();

    void init(const usImagePreScan3D<unsigned char> &preScanImage, int down = 1);

    double getResolution() const;

    void getVolume(usImagePostScan3D<unsigned char> &V);
    usImagePostScan3D<unsigned char> getVolume();

    double getResolution() {return _resolution;}

    void SweepInZdirection(bool flag) {_SweepInZdirection = flag;}

    void convert(usImagePostScan3D<unsigned char> &postScanImage, const unsigned char *dataPreScan=NULL);

    void convertPreScanCoordToPostScanCoord(double i, double j, double k, double *x=NULL, double *y=NULL, double *z=NULL, bool sweepInZdirection=true);
    void convertPostScanCoordToPreScanCoord(double x, double y, double z, double *i=NULL, double *j=NULL, double *k=NULL, bool sweepInZdirection=true);
};

#endif // US_SCAN_CONVERTER_3D_H
