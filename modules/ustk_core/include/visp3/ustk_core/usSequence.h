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
* Marc Pouliquen
*
*****************************************************************************/

/**
* @file usSequence.h
* @brief Storage of sequences of ultrasound images 
*
* This class is used to store multiple ultrasound images, and to do some basic operations such as :
* -get any image from the sequence
* -insert an image everywhere in the existing sequence
* -delete an image everywhere in the existing sequence
*/

#ifndef US_SEQUENCE_H
#define US_SEQUENCE_H

#include <cstring>
#include <iostream>
#include <vector>
#include <algorithm>

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpImage.h>

/**
* @class usSequence
* @brief Storage of sequences of ultrasound images 
* @ingroup module_ustk_core
*
* This class is used to store multiple ultrasound images, and to do some basic operations such as :
* -insert an image everywhere in the existing sequence
* -delete an image everywhere in the existing sequence
*/
template <class ImageType>
class usSequence
{
public:

  usSequence();
  
  usSequence(const usSequence<ImageType> &sequence);

  ~usSequence();
  
  //operators overload
  usSequence<ImageType> & operator=(const usSequence<ImageType> &other);
  bool operator==(const usSequence<ImageType> &other);
  //template<ImageType> friend std::ostream& operator<<(std::ostream& out, const usSequence<ImageType> &other);
  
  //attributes getters/setters
  int getSequence() const {return m_sequence;}
  int getSequenceSize() const {return m_sequence.size();}
  double getFrameRate() const {return m_frameRate;}
  std::string getSequenceName() const {return m_sequenceName;}
  
  void setFrameRate(const double &frameRate) {m_frameRate = frameRate;}
  void setSequenceName(const std::string &sequenceName) {m_sequenceName = sequenceName;}
  
  //data accessor
  ImageType at(int imageNumber) const {if(imageNumber>=0 && imageNumber<=getSequenceSize()){return m_sequence[imageNumber];}throw(vpException(vpException::badValue,"Image Number out of range !"));}
  
  //data insertion
  void insert(ImageType imageToInsert, int imageNumber);
  
  //data deletion
  void deleteImage(int imageNumber);
  
private:
  std::vector<ImageType> m_sequence;
  double m_frameRate;
  std::string m_sequenceName;
};

/****************************************************************************
* Template implementations.
****************************************************************************/

/**
* Constructor.
*/
template<class ImageType>
usSequence<ImageType>::usSequence() : m_sequence(), m_frameRate(0.0), m_sequenceName("")
{

}

/**
* Copy constructor. Performs a deep copy of every image in memory to avoid issues as concurrent modifications in the same image from 2 differents usSequences.
* TO DO
*/
template<class ImageType>
usSequence<ImageType>::usSequence(const usSequence<ImageType> &sequence) : m_sequence(sequence.getSequence()), m_frameRate(sequence.getFrameRate()), m_sequenceName(sequence.getSequenceName())
{

}

/**
* Destructor.
*/
template<class ImageType>
usSequence<ImageType>::~usSequence()
{

}

/**
* Assignement operator. TO DO
*/
template<class ImageType>
usSequence<ImageType> &usSequence<ImageType>::operator=(const usSequence<ImageType> &other)
{

  return *this;
}

/**
* Comparaison operator. TO DO
*/
template<class ImageType>
bool usSequence<ImageType>::operator==(const usSequence<ImageType> &other)
{

  return true;
}

/**
* Ostream class information printer operator. TO DO
*/
/*template<class ImageType> std::ostream& operator<<(std::ostream& out, const usSequence<ImageType> &other)
{
  return out << ""
				<< std::endl;
}*/

/**
* Insertion of a new image in the sequence.
* @param imageToInsert Image you want to insert in the sequence.
* @param imageNumber Position where the image will be inserted in the sequence.
*/
template<class ImageType>
void usSequence<ImageType>::insert(ImageType imageToInsert, int imageNumber)
{
  m_sequence.insert(m_sequence.begin()+imageNumber,imageToInsert);
}

/**
* Deletion of an image in the sequence.
* @param imageNumber Position where the image will be deleted in the sequence.
*/
template<class ImageType>
void usSequence<ImageType>::deleteImage(int imageNumber)
{
  m_sequence.erase(imageNumber);
}

#endif //US_SEQUENCE_H
