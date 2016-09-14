#include <visp3/ustk_core/usImageRF2D.h>
#include <cstring>

/**unsigned char
* Constructor.
*/
usImageRF2D::usImageRF2D() : vpImage<short>(), usImageSettings()
{
	
}

/**
* Initializing constructor for image dimentions.
* @param[in] AN number of A-samples in a line.
* @param[in] LN number of lines.
*/
usImageRF2D::usImageRF2D(unsigned int AN, unsigned int LN)
  : vpImage<short>(AN, LN), usImageSettings()
{
	
}

/**
* Initializing constructor.
* @param[in] AN number of A-samples in a line.
* @param[in] LN number of lines.
* @param[in] probeRadius radius of the ultrasound probe used to acquire the RF image.
* @param[in] lineAngle angle (in radians) between 2 lines of the ultrasound probe used to acquire the RF image.
* @param[in] resolution resolution of the image.
* @param[in] BSampleFreq frequency (Hz) used for B-Mode.
* @param[in] probeElementPitch physical distance (in meters) between two piezo-electric elements in the ultrasound probe.
*/
usImageRF2D::usImageRF2D(unsigned int AN, unsigned int LN, float probeRadius, float lineAngle,
  float resolution, float BSampleFreq, float probeElementPitch)
  : vpImage<short>(AN, LN), usImageSettings(probeRadius, lineAngle, resolution, BSampleFreq, probeElementPitch)
{
	
}

/**
* Copy constructor.
* @param other usImageRF2D to copy
*/
usImageRF2D::usImageRF2D(const usImageRF2D& other)
  : vpImage<short>(other), usImageSettings(other)
{
	
}

/**
* Destructor.
*/
usImageRF2D::~usImageRF2D()
{

}

/**
* Get the number of A-samples in a line.
* @return AN number of A-samples in a line.
*/
unsigned int usImageRF2D::getAN() const { return getHeight(); }

/**
* Get the number of lines.
* @return LN number of lines.
*/
unsigned int  usImageRF2D::getLN() const { return getWidth(); }
