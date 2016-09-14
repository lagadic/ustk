#include <visp3/ustk_core/usImageRF2D.h>
#include <cstring>

usImageRF2D::usImageRF2D() : vpImage<double>(), usImageSettings()
{
	
}

usImageRF2D::usImageRF2D(unsigned int AN, unsigned int LN)
  : vpImage<double>(AN, LN), usImageSettings()
{
	
}

usImageRF2D::usImageRF2D(unsigned int AN, unsigned int LN,
		       float probeRadius, float lineAngle, float resolution,
		       float BSampleFreq, float probeElementPitch)
  : vpImage<double>(AN, LN), usImageSettings(probeRadius, lineAngle, resolution, BSampleFreq, probeElementPitch)
{
	
}

usImageRF2D::usImageRF2D(const usImageRF2D& other)
  : vpImage<double>(other), usImageSettings()
{
	
}

usImageRF2D::~usImageRF2D()
{

}

/**
* Get the number of A-samples in a line.
* @param[out] AN number of A-samples in a line.
*/
unsigned int usImageRF2D::getAN() const { return getHeight(); }

/**
* Get the number of lines.
* @param[out] LN number of lines.
*/
unsigned int  usImageRF2D::getLN() const { return getWidth(); }
