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

/*!
  \file File_tools.cpp
  \brief Definition of the File_tools class
*/

#include <UsTk/usIO_tools.h>

using namespace std;


/*!
  \brief count the number of lines of a text file
  \param filename : name of the text file
  \return the number of lines in the file
*/

int File_tools::nbLignes (char* filename)
{
	// Ouverture du fichier texte
	ifstream f(filename);
	string s;

	if(f.is_open())
	{
		// Decompte du nombre de lignes
        unsigned int count = 0;
        while(getline(f,s)) ++count;
        return count;
    }
	else
	{
        cout << "Error in file opening" << endl;
    }
    f.close();
    return 0;
}


/*!
  \brief load data from a text file in a matrix of doubles
  \param filename : name of the text file 
  \param tab[][] : vector of the data
 */

void File_tools::cvt_File2Matrix (char* filename, vpMatrix &tab)
{
	
	ifstream f(filename);
	int nbL, NoL;
	double tmp;
	int max = tab.getCols();
	// Calcul du nombre de lignes du fichier texte 
	nbL = nbLignes(filename) ;

	if (!f.is_open())
		cerr<<" Impossible d'ouvrir le fichier "<<endl;

	else
	{
		// Pour chaque point
		for (int i=0 ; i<nbL ; i++)
		{
			for ( int j=0 ; j<max ; j++) 
			{
				// Remplissage du tableau avec les coordonnées 3D du point
				f >> tmp;
				tab[i][j]=tmp;
			}
		}
	}
	f.close();
}

/*!
  \brief load data from a text file in a matrix of doubles
  \param filename : name of the text file 
  \param tab[][] : vector of the data
 */

void File_tools::cvt_File2Matrix (char* filename, vpMatrix &tab, int max)
{
	
	ifstream f(filename);
	int nbL, NoL;
	double tmp;
	// Calcul du nombre de lignes du fichier texte 
	nbL = nbLignes(filename) ;

	if (!f.is_open())
		cerr<<" Impossible d'ouvrir le fichier "<<endl;

	else
	{
		// Pour chaque point
		for (int i=0 ; i<nbL ; i++)
		{
			for ( int j=0 ; j<max ; j++) 
			{
				// Remplissage du tableau avec les coordonnées 3D du point
				f >> tmp;
				tab[i][j]=tmp;
			}
		}
	}
	f.close();
}

/*!
  \brief load data from a text file in a matrix of doubles
  \param filename : name of the text file 
  \param tab[][] : vector of the data
 */

void File_tools::cvt_Matrix2File (char* filename, vpMatrix &tab)
{
  
  ofstream f(filename);
  int nbL, NoL;
  double tmp;
  int max = tab.getCols();
  nbL = tab.getRows() ;

  if (!f.is_open())
    {
      cerr<<" Impossible d'ouvrir le fichier "<<endl;
      return;
    }
	else
	  {
	    // Pour chaque point
	    for (int i=0 ; i<nbL ; i++)
	      {
		for ( int j=0 ; j<max ; j++) 
		  {
		    // Remplissage du tableau avec les coordonnées 3D du point
		    tmp=tab[i][j];
		    f << tmp;
		    f << "\t";
		  }
		f << "\n";
	      }
	  }
  f.close();
}



