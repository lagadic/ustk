#include <visp3/ustk_gui/usViper850WrapperVelocityContol.h>

#if defined(USTK_HAVE_VTK_QT) && defined(VISP_HAVE_VIPER850) && defined(VISP_HAVE_MODULE_ROBOT)

usViper850WrapperVelocityControl::usViper850WrapperVelocityControl()
{
  m_initialized = false;
  m_run = false;
  velocityProbeContact = vpColVector(6,0);

  connect(this, SIGNAL(startControlLoop()), this, SLOT(controlLoop()));
}

usViper850WrapperVelocityControl::~usViper850WrapperVelocityControl()
{
    if(viper->getPowerState())
    {
        viper->stopMotion();
        viper->powerOff();
    }
}

void usViper850WrapperVelocityControl::init()
{
    try {
      //create viper object
      viper = new vpRobotViper850();
    } catch(...) {
      std::cout << "Viper robot could not be initialized" << std::endl;
      emit(robotError());
    }

    // Transformation from end-effector frame to the probe contact frame
    vpHomogeneousMatrix eMp;
    eMp[2][3] = 0.2; // tz = 20cm

    // Build the transformation that allows to convert a velocity in the end-effector frame to the probe contact frame
    eVp.buildFrom(eMp);

    startRobot();
    m_initialized = true;
}

void usViper850WrapperVelocityControl::startRobot(void)
{
    // Test if Viper is in MANUAL contol mode
    if(viper->getControlMode() == vpRobotViper850::MANUAL)
    {
        // Power On the Viper robot
        try {
          viper->powerOn();
          viper->setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
        }
        catch (...)
        {
            viper->setRobotState(vpRobot::STATE_STOP);
            std::cout << "Viper robot could not be initialized" << std::endl;
            emit(robotError());
        }
    }
    else
        std::cout << "For security reason, the Viper robot has to be used with the dead man switch)" << std::endl;
}

void usViper850WrapperVelocityControl::run()
{
  if(m_initialized) {
    m_run = true;
    emit(startControlLoop());
  }
}

void usViper850WrapperVelocityControl::stop()
{
  m_run = false;
}

void usViper850WrapperVelocityControl::controlLoop()
{
  while(m_run) {
    //transform the velocity
    ve = eVp * velocityProbeContact;

    // Get the robot jacobian eJe
    try{
    viper->get_eJe(eJe);
    q_dot = eJe.pseudoInverse() * ve;

    // Send the joint velocities to the robot
    viper->setVelocity(vpRobot::ARTICULAR_FRAME, q_dot);
    } catch (...) {
      viper->setRobotState(vpRobot::STATE_STOP);
      std::cout << "Viper robot could not be initialized" << std::endl;
      emit(robotError());
    }

    // as we are in a loop in a slot, we have to tell to the qapplication to take in account incoming signals and execute corresponding slots
    qApp->processEvents();
  }
}

/**
 * @brief Set the linear velocity along x axis in 4DC7 probe contact frame.
 * @param xVelocity Velocity in millimeter per second.
 */
void usViper850WrapperVelocityControl::setXVelocity(int xVelocity)
{
  velocityProbeContact[0] = (double)xVelocity/1000.0;
}

/**
 * @brief Set the linear velocity along y axis in 4DC7 probe contact frame.
 * @param yVelocity Velocity in millimeter per second.
 */
void usViper850WrapperVelocityControl::setYVelocity(int yVelocity)
{
  velocityProbeContact[1] = (double)yVelocity/1000.0;
}

/**
 * @brief Set the linear velocity along z axis in 4DC7 probe contact frame.
 * @param zVelocity Velocity in millimeter per second.
 */
void usViper850WrapperVelocityControl::setZVelocity(int zVelocity)
{
  velocityProbeContact[2] = (double)zVelocity/1000.0;
}

/**
 * @brief Set the angular velocity around x axis in 4DC7 probe contact frame.
 * @param xAngularVelocity Velocity in 10-1 deg per second.
 */
void usViper850WrapperVelocityControl::setXAngularVelocity(int xAngularVelocity)
{
    velocityProbeContact[3] = vpMath::rad((double)xAngularVelocity/10.0);
}

/**
 * @brief Set the angular velocity around y axis in 4DC7 probe contact frame.
 * @param yAngularVelocity Velocity in 10-1 deg per second.
 */
void usViper850WrapperVelocityControl::setYAngularVelocity(int yAngularVelocity)
{
    velocityProbeContact[4] = vpMath::rad((double)yAngularVelocity/10.0);
}

/**
 * @brief Set the angular velocity around z axis in 4DC7 probe contact frame.
 * @param zAngularVelocity Velocity in 10-1 deg per second.
 */
void usViper850WrapperVelocityControl::setZAngularVelocity(int zAngularVelocity)
{
  velocityProbeContact[5] = vpMath::rad((double)zAngularVelocity/10.0);
}
#endif
