/****************************************************************************
 *
 * This file is part of the UsTk software.
 * Copyright (C) 2003-2014 by Inria. All rights reserved.
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
 * Alexandre Krupa
 *
 *****************************************************************************/

#ifndef FILE_TOOLS_H
#define FILE_TOOLS_H

/*!
  \class File_tools
  \brief  The class gives tools to read a text file
  \author  Caroline Nadeau   Lagadic
*/

#include <iostream>
#include <fstream>
#include <string>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpRotationMatrix.h>


class VISP_EXPORT File_tools{
public:
    //FILE_Tools();
	//! return the number of lines of a text file
	static int nbLignes(char* filename);

	//! Convert text file into vpMatrix
	static void cvt_File2Matrix (char* filename, vpMatrix &tab);

	//! Convert text file into vpMatrix
	static void cvt_File2Matrix (char* filename, vpMatrix &tab, int max);

	//! Convert text file into vpMatrix
	static void cvt_Matrix2File (char* filename, vpMatrix &tab);



    //~FILE_Tools();

};

#endif
