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
#if 0
#include <visp3/ustk_grabber/usVolumeGrabber.h>


usVolumeGrabber::usVolumeGrabber(QObject *parent) :
    QObject(parent)
{
    m_stopped = false;
    isValChg  = false;
    Idx       = 0;
    nanoOld = 0;
	m_nv = 0;
    timer.start();
}

usVolumeGrabber::~usVolumeGrabber()
{
    delete header;
}

void usVolumeGrabber::SetSharedBuff(std::shared_ptr<usImagePreScan3D<unsigned char> > t_sBuff)
{
    sBuff3d = std::shared_ptr<usImagePreScan3D<unsigned char> >(t_sBuff);
	header = (int*)malloc(Hsize*sizeof(int));
	addHeader();
	m_size = sizeof(short)*header[3]*header[4];
}


void usVolumeGrabber::receivedArray(short *buffer, int t_Idx)
{
  //Vector to store the rf signals
	if(m_nv%2 == 0)
    memcpy(sBuff3d.get()->getData()+(t_Idx*header[3]*header[4]), buffer, m_size);
	else
    memcpy(sBuff3d.get()->getData()+
		((2*header[2]-1-t_Idx)*header[3]*header[4]), buffer, m_size);

  //If we reach the end of the volume
	if((t_Idx+1)%header[2] == 0)//t_Idx == (header[2]-1))
	{
		m_nv++;
		emit mem_update();
  }
}

void usVolumeGrabber::addHeader()
{
  int size = 7*sizeof(int);
	memcpy(header, sBuff3d.get()->getHdr(), size);
	qDebug()<<"size" << size;
}
#endif
