#include <visp3/ustk_data/usDataRF3D.h>
#include <cstring>

usDataRF3D::usDataRF3D() : usVolume<short>(),
			   m_probeRadius(0.0f), m_motorRadius(0.0f),
			   m_lineAngle(0.0f), m_frameAngle(0.0f), m_resolution(0.0f),
			   m_BSampleFreq(0.0f), m_probeElementPitch(0.0f)
{
  setMode(RF_3D);
}

usDataRF3D::usDataRF3D(unsigned int AN, unsigned int LN, unsigned int FN)
  : usVolume<short>(AN, LN, FN, 1.0, 1.0, 1.0)
{
  m_probeRadius = 0.0f;
  m_motorRadius = 0.0f;
  m_lineAngle = 0.0f;
  m_frameAngle = 0.0f;
  m_resolution = 0.0f;
  m_BSampleFreq = 0.0f;
  m_probeElementPitch = 0.0f;
  setMode(RF_3D);
}

usDataRF3D::usDataRF3D(unsigned int AN, unsigned int LN, unsigned int FN,
		       float probeRadius, float motorRadius,
		       float lineAngle, float frameAngle, float resolution,
		       float BSampleFreq, float probeElementPitch)
  : usVolume<short>(AN, LN, FN, 1.0, 1.0, 1.0)
{
  m_probeRadius = probeRadius;
  m_motorRadius = motorRadius;
  m_lineAngle = lineAngle;
  m_frameAngle = frameAngle;
  m_resolution = resolution;
  m_BSampleFreq = BSampleFreq;
  m_probeElementPitch = probeElementPitch;
  setMode(RF_3D);
}

usDataRF3D::usDataRF3D(const usDataRF3D& other, const bool copy)
  : usVolume<short>(other, copy)
{
  m_probeRadius = other.m_probeRadius;
  m_motorRadius = other.m_motorRadius;
  m_lineAngle = other.m_lineAngle;
  m_frameAngle = other.m_frameAngle;
  m_resolution = other.m_resolution;
  m_BSampleFreq = other.m_BSampleFreq;
  m_probeElementPitch = other.m_probeElementPitch;
  setMode(RF_3D);
}
