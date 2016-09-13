#include <visp3/ustk_core/usImageRF2D.h>
#include <cstring>

usImageRF2D::usImageRF2D() : vpImage<unsigned char>(), usImageSettings()
{
	
}

usImageRF2D::usImageRF2D(unsigned int AN, unsigned int LN)
  : vpImage<unsigned char>(AN, LN), usImageSettings()
{
	
}

usImageRF2D::usImageRF2D(unsigned int AN, unsigned int LN,
		       float probeRadius, float lineAngle, float resolution,
		       float BSampleFreq, float probeElementPitch)
  : vpImage<unsigned char>(AN, LN), usImageSettings(probeRadius, lineAngle, resolution, BSampleFreq, probeElementPitch)
{
	
}

usImageRF2D::usImageRF2D(const usImageRF2D& other)
  : vpImage<unsigned char>(other), usImageSettings()
{
	
}

usImageRF2D::~usImageRF2D()
{

}