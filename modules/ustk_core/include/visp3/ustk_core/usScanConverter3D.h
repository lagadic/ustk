/****************************************************************************
 *
 * This file is part of the UsTk software.
 * Copyright (C) 2014 by Inria. All rights reserved.
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

#ifndef US_SCAN_CONVERTER_3D_H
#define US_SCAN_CONVERTER_3D_H

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
 *
 * @warning probe and motor settings are set for ultrasonix 3D probe only for the moment (in the protected attributes).
 * @todo use usTransducer and usMotor settings instead of ultrasonix default values.
 */
class VISP_EXPORT usScanConverter3D
{
protected:
    double DEFAULT_DEG_PER_LINE = 0.608767657441;
    double DEFAULT_PROBE_RADIUS = 0.04;
    double DEFAULT_MOTOR_RADIUS = 0.02725;
    double DEFAULT_CENTER_OFFSET = DEFAULT_PROBE_RADIUS - DEFAULT_MOTOR_RADIUS;
    double SPEED_OF_SOUND = 1540;
    double DEFAULT_BSAMPLE_DISTANCE = 0.000308;
    double DEFAULT_DEG_PER_FRAME = 1.463;
    double DEFAULT_RAD_PER_FRAME = M_PI / 180.0 * DEFAULT_DEG_PER_FRAME;
    double DEFAULT_RAD_PER_LINE = M_PI / 180.0 * DEFAULT_DEG_PER_LINE;

    class VoxelWeightAndIndex
    {
        friend class usScanConverter3D;
        unsigned int _outputIndex;
        unsigned int _inputIndex[8];
        double _W[8];
    };

    VoxelWeightAndIndex* _lookupTable1;
    unsigned int lut1Size;
    VoxelWeightAndIndex* _lookupTable2;
    unsigned int lut2Size;

    usImagePreScan3D<unsigned char> _VpreScan;
    usImagePostScan3D<unsigned char> _VpostScan;

    double _resolution;
    bool _SweepInZdirection;

public:

    //! Constructors, destructors

    usScanConverter3D();
    usScanConverter3D(int x, int y, int z, int down = 1);
    virtual ~usScanConverter3D();

    void init(int X, int Y, int Z, int down = 1);

    double getResolution() const;

    void setVolume(const usImagePreScan3D<unsigned char> &V);

    void getVolume(usImagePostScan3D<unsigned char> &V);
    usImagePostScan3D<unsigned char> getVolume();

    double getResolution() {return _resolution;}

    void SweepInZdirection(bool flag) {_SweepInZdirection = flag;}

    void convert();

    void convertPreScanCoordToPostScanCoord(double i, double j, double k, double *x=NULL, double *y=NULL, double *z=NULL, bool sweepInZdirection=true);
    void convertPostScanCoordToPreScanCoord(double x, double y, double z, double *i=NULL, double *j=NULL, double *k=NULL, bool sweepInZdirection=true);
};

#endif // US_SCAN_CONVERTER_3D_H
