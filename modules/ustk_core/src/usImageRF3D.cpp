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
* @param[in] scanLinePitch angle(rad) / distance(m) between 2 lines of the ultrasound probe used to acquire the RF image.
* @param[in] framePitch angle(rad) / distance(m) between 2 lines of the ultrasound probe used to acquire the RF image.
* @param[in] isImageConvex Boolean to specyfy if the image was acquired by a convex probe(true) or by a linear probe (false).
* @param[in] isMotorConvex Boolean to specyfy if the image was acquired by a rotating  motor(true) or by a linear motor (false).
*/
usImageRF3D::usImageRF3D(unsigned int AN, unsigned int LN, unsigned int FN, float probeRadius, float motorRadius, float scanLinePitch, float framePitch,
  bool isImageConvex, bool isMotorConvex)
  : usImage3D<short>(AN, LN, FN), usImageSettings3D(probeRadius, motorRadius, scanLinePitch, framePitch, isImageConvex, isMotorConvex)
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

/**
* Setter for axial Resolution.
* @param Axial resolution (in meters) to set.
*/
void usImageRF3D::setAxialResolution(float axialResolution) { m_axialResolution = axialResolution; }

/**
* Getter for axial Resolution.
* @return Axial resolution (in meters).
*/
float usImageRF3D::getAxialResolution() const { return m_axialResolution; }
