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
 * Pedro Patlan
 * Marc Pouliquen
 *
 *****************************************************************************/

/**
 * @file usVolumeGrabber.h
 * @brief Grabber storing a volume and updating it when a new frame arrives. Using usNetworkGrabber to grab frames on the network.
 */

#ifndef __usVolumeGrabber_h_
#define __usVolumeGrabber_h_

#if 0

#include <visp3/ustk_grabber/usGrabberConfig.h>

#if defined(USTK_HAVE_QT4) || defined(USTK_HAVE_QT5)

//Qt includes
#include <QThread>
#include <QMutex>
#include <QSemaphore>
#include <QElapsedTimer>
#include <iostream>
#include <QSharedMemory>
#include <QDebug>

#include <memory>

//USTK includes
#include <visp3/ustk_core/usImagePreScan3D.h>


#define Hsize 7

class usVolumeGrabber : public QObject
{
    Q_OBJECT
public:
    explicit usVolumeGrabber(QObject *parent = 0);
    virtual ~usVolumeGrabber();
  void SetSharedBuff(std::shared_ptr<usImagePreScan3D<unsigned char> > t_sBuff);
signals:
    void mem_update();
	void changeForce();
public slots:
    void receivedArray(short *buffer, int t_Idx);
private:
    QElapsedTimer timer;
    qint64 nanoSec;
    qint64 nanoOld;
    bool m_stopped;
    bool isValChg;
    QMutex m_mutex;
    int Idx;
    int *header;
	long int m_nv;
	
  std::shared_ptr<usImagePreScan3D<unsigned char> > sBuff3d;
	
    int m_size;
    void addHeader();
};

#endif // Qt
#endif
#endif // __usVolumeGrabber_h_
