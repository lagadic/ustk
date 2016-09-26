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
* @param AN number of A-samples in a line.
* @param LN number of lines.
*/
usImageRF2D::usImageRF2D(unsigned int AN, unsigned int LN)
  : vpImage<short>(AN, LN), usImageSettings()
{
	
}

/**
* Initializing constructor.
* @param AN number of A-samples in a line.
* @param LN number of lines.
* @param probeRadius radius of the ultrasound probe used to acquire the RF image.
* @param scanLinePitch Angle(rad) / Distance(m) between 2 lines of the ultrasound probe used to acquire the RF image. Angle if isConvex is true, distance if it's false.
* @param isConvex Boolean to specify if the probe used was convex(true) or linear(false).
*/
usImageRF2D::usImageRF2D(unsigned int AN, unsigned int LN, double probeRadius, double scanLinePitch, bool isConvex)
  : vpImage<short>(AN, LN), usImageSettings(probeRadius, scanLinePitch, isConvex)
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

/**
* Setter for axial Resolution.
* @param Axial resolution (in meters) to set.
*/
void usImageRF2D::setAxialResolution(double axialResolution) { m_axialResolution = axialResolution; }

/**
* Getter for axial Resolution.
* @return Axial resolution (in meters).
*/
double usImageRF2D::getAxialResolution() const { return m_axialResolution; }
