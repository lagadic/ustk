#include <visp3/ustk_core/usImageRF2D.h>
#include <cstring>

template<class T>
usImageRF2D<T>::usImageRF2D() : vpImage<T>(), usImageSettings()
{
	
}

template<class T>
usImageRF2D<T>::usImageRF2D(unsigned int AN, unsigned int LN)
  : vpImage<T>(AN, LN), usImageSettings()
{
	
}

template<class T>
usImageRF2D<T>::usImageRF2D(unsigned int AN, unsigned int LN,
		       float probeRadius, float lineAngle, float resolution,
		       float BSampleFreq, float probeElementPitch)
  : vpImage<T>(AN, LN), usImageSettings(probeRadius, lineAngle, resolution, BSampleFreq, probeElementPitch)
{
	
}

template<class T>
usImageRF2D<T>::usImageRF2D(const usImageRF2D& other)
  : vpImage<T>(other), usImageSettings()
{
	
}

template<class T>
usImageRF2D<T>::~usImageRF2D()
{

}