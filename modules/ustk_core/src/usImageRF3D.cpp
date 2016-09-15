#include <visp3/ustk_core/usImageRF3D.h>
#include <cstring>

/**
* Basic constructor.
*/
usImageRF3D::usImageRF3D() : usImage3D<short>(), usImageSettings3D()
{

}

/**
* Initializing constructor for image dimentions.
* @param[in] AN number of A-samples in a line.
* @param[in] LN number of lines.
*/
usImageRF3D::usImageRF3D(unsigned int AN, unsigned int LN, unsigned int FN)
  : usImage3D<short>(AN, LN, FN), usImageSettings3D()
{

}

/**
* Full initializing constructor.
* @param[in] AN number of A-samples in a line.
* @param[in] LN number of lines.
* @param[in] FN number of frames.
* @param[in] probeRadius radius of the ultrasound probe used to acquire the RF image.
* @param[in] motorRadius radius of the ultrasound probe motor used to acquire the RF image.
* @param[in] lineAngle angle (in radians) between 2 lines of the ultrasound probe used to acquire the RF image.
* @param[in] frameAngle angle (in radians) between 2 successive ultrasound frames when the RF image was acquired.
* @param[in] resolution resolution of the image.
* @param[in] BSampleFreq frequency (Hz) used for B-Mode.
* @param[in] probeElementPitch physical distance (in meters) between two piezo-electric elements in the ultrasound probe.
*/
usImageRF3D::usImageRF3D(unsigned int AN, unsigned int LN, unsigned int FN, float probeRadius, float motorRadius, float lineAngle, float frameAngle,
  float resolution, float BSampleFreq, float probeElementPitch)
  : usImage3D<short>(AN, LN, FN), usImageSettings3D(probeRadius, motorRadius, lineAngle, frameAngle, resolution, BSampleFreq, probeElementPitch)
{

}

/**
* Copy constructor from usImage3D and usImageSettings
* @param image3D usImage3D to copy
* @param imageSettings usImageSettings3D to copy
*/
usImageRF3D::usImageRF3D(usImage3D<short> image3D, usImageSettings3D imageSettings) : usImage3D<short>(image3D), usImageSettings3D(imageSettings) {

}

/**
* Copy constructor from usImage3D and usImageSettings
* @param image3D usImage3D to copy
* @param imageSettings usImageSettings3D to copy
*/
usImageRF3D::usImageRF3D(usImage3D<short> image3D) : usImage3D<short>(image3D) {

}

/**
* Copy constructor from usImage3D and usImageSettings
* @param image3D usImage3D to copy
* @param imageSettings usImageSettings3D to copy
*/
usImageRF3D::usImageRF3D(usImageSettings3D imageSettings) : usImageSettings3D(imageSettings) {

}

/**
* Copy constructor.
* @param other usImageRF3D to copy
*/
usImageRF3D::usImageRF3D(const usImageRF3D& other)
  : usImage3D<short>(other), usImageSettings3D(other)
{

}

/**
* Destructor.
*/
usImageRF3D::~usImageRF3D()
{

}

/**
* Get the number of A-samples in a line.
* @return AN number of A-samples in a line.
*/
unsigned int usImageRF3D::getAN() const { return getDimX(); }

/**
* Get the number of lines.
* @return LN number of lines.
*/
unsigned int  usImageRF3D::getLN() const { return getDimY(); }

/**
* Get the number of frames.
* @return FN number of frames.
*/
unsigned int  usImageRF3D::getFN() const { return getDimZ(); }
