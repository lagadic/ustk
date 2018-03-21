#include <visp3/ustk_gui/usViper850WrapperVelocityContol.h>

#if defined(USTK_HAVE_VTK_QT) && defined(VISP_HAVE_VIPER850) && defined(VISP_HAVE_MODULE_ROBOT)

usViper850WrapperVelocityControl::usViper850WrapperVelocityControl()
{
  m_initialized = false;
  m_run = false;
  velocityProbeContact = vpColVector(6,0);

  connect(this, SIGNAL(startControlLoop()), this, SLOT(controlLoop()));
  connect(this, SIGNAL(startControlLoopAutomatic()), this, SLOT(controlLoopAutomatic()));
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
    eMp[0][0] = 0;
    eMp[1][1] = 0;
    eMp[2][2] = 0;
    eMp[0][2] = 1; // Z in force sensor becomes X in the probe control frame
    eMp[1][0] = 1; // X in force sensor becomes Y in the probe control frame
    eMp[2][1] = 1; // Y in force sensor becomes Z in the probe control frame
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

void usViper850WrapperVelocityControl::controlLoopAutomatic()
{
  // Transformation from end-effector frame to the force/torque sensor
  // Note that the end-effector frame is located on the lower part of
  // male component of the tool changer.
  vpHomogeneousMatrix eMs;
  eMs[2][3] = -0.062; // tz = -6.2cm

  // Transformation from force/torque sensor to the probe frame from where
  // we want to control the robot
  vpHomogeneousMatrix sMp;

  // Transformation from force/torque sensor to the end-effector frame
  vpHomogeneousMatrix sMe;
  eMs.inverse(sMe);

  // Build the transformation that allows to convert a velocity in the
  // end-effector frame to the FT sensor frame
  vpVelocityTwistMatrix sVe;
  sVe.buildFrom(sMe);

  vpColVector sHs(6);      // force/torque sensor measures
  vpColVector pHp_star(6); // force/torque sensor desired values in probe frame
  vpColVector gHg(6);      // force/torque due to the gravity
  vpMatrix lambdaProportionnal(6, 6); //gain of proportionnal part of the force controller
  vpMatrix lambdaDerivate(6, 6); //gain of derivate part of the force controller
  vpMatrix lambdaIntegral(6, 6); //gain of derivate part of the force controller
  vpColVector sEs(6,0);      // force/torque error
  vpColVector sEs_last(6,0);      // force/torque error
  vpColVector sEs_sum(6,0);      // force/torque sum error
  // Position of the cog of the object attached after the sensor in the sensor frame
  vpTranslationVector stg;
  vpColVector sHs_bias(6); // force/torque sensor measures for bias

  // Cartesian velocities corresponding to the force/torque control in the
  // sensor frame
  vpColVector v_s(6);

  //////////   PID gains ///////
  // Initialized the force gains, for proportionnal component
  lambdaProportionnal = 0;
  for (int i = 0; i < 3; i++)
    lambdaProportionnal[i][i] = 0.015;
  // Initialized the torque gains, for proportionnal component
  for (int i = 3; i < 6; i++)
    lambdaProportionnal[i][i] = 0;
  // Initialized the force gains, for derivate component
  lambdaDerivate = 0;
  for (int i = 0; i < 3; i++)
    lambdaDerivate[i][i] = 0.09;
  // Initialized the torque gains, for derivate component
  for (int i = 3; i < 6; i++)
    lambdaDerivate[i][i] = 0;
  // Initialized the force gains, for integral component
  lambdaIntegral = 0;
  for (int i = 0; i < 3; i++)
    lambdaIntegral[i][i] = 0;
  // Initialized the torque gains, for integral component
  for (int i = 3; i < 6; i++)
    lambdaIntegral[i][i] = 0;

  // Initialize the desired force/torque values
  pHp_star = 0;
  pHp_star[1] = 4; // Fy = 4N

  // Set the probe frame control
  sMp[0][0] = 0;
  sMp[1][1] = 0;
  sMp[2][2] = 0;
  sMp[0][2] = 1; // Z in force sensor becomes X in the probe control frame
  sMp[1][0] = 1; // X in force sensor becomes Y in the probe control frame
  sMp[2][1] = 1; // Y in force sensor becomes Z in the probe control frame
  sMp[2][3] = 0.262; // tz = 26.2cm

  // Init the force/torque due to the gravity
  gHg[2] = -(0.696 + 0.476) * 9.81; // m*g

  // Position of the cog of the object attached after the sensor in the sensor frame
  stg[0] = 0;
  stg[1] = 0;
  stg[2] = 0.088; // tz = 88.4mm

  vpRotationMatrix sRp;
  sMp.extract(sRp);
  vpTranslationVector stp;
  sMp.extract(stp);

  vpHomogeneousMatrix eMp = eMs * sMp;
  vpVelocityTwistMatrix eVp(eMp);

  // Get the position of the end-effector in the reference frame
  vpColVector q;
  vpHomogeneousMatrix fMe;
  vpHomogeneousMatrix fMs;
  vpRotationMatrix sRf;
  viper->getPosition(vpRobot::ARTICULAR_FRAME, q);
  viper->get_fMe(q, fMe);
  // Compute the position of the sensor frame in the reference frame
  fMs = fMe * eMs;
  vpHomogeneousMatrix sMf;
  fMs.inverse(sMf);
  sMf.extract(sRf);

  // Build the transformation that allows to convert the forces due to the
  // gravity in the sensor frame
  vpForceTwistMatrix sFg(sMf); // Only the rotation part is to consider
  // Modify the translational part
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      sFg[i + 3][j] = (stg.skew() * sRf)[i][j];

  // Build the transformation that allows to convert a FT expressed in the
  // FT probe frame into the sensor frame
  vpForceTwistMatrix sFp(sMp);

  // Bias the force/torque sensor
  viper->biasForceTorqueSensor();

  // Set the robot to velocity control
  viper->setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

  unsigned int iter = 0;

  //signal filtering
  unsigned int bufferSize = 80;
  double signalBuffer[bufferSize];
  for(unsigned int i=0; i<bufferSize;i++)
    signalBuffer[i]=0;

  double t0 = vpTime::measureTimeMs();
  double deltaTmilliseconds = 1; // 1ms for each loop cycle

  while(m_run) {
    try {
      t0 = vpTime::measureTimeMs();

    // Get the force/torque measures from the sensor
    sHs = viper->getForceTorque();

    // Multiply the measures by -1 to get the force/torque exerced by the
    // robot to the environment.
    sHs *= -1;

    // Update the gravity transformation matrix
    viper->getPosition(vpRobot::ARTICULAR_FRAME, q);
    viper->get_fMe(q, fMe);
    // Compute the position of the sensor frame in the reference frame
    fMs = fMe * eMs;
    // Compute the inverse transformation
    fMs.inverse(sMf);
    sMf.extract(sRf);
    // Update the transformation that allows to convert the forces due to the
    // gravity in the sensor frame
    sFg.buildFrom(sMf); // Only the rotation part is to consider
    // Modify the translational part
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        sFg[i + 3][j] = (stg.skew() * sRf)[i][j];

    if (iter == 0) {
      sHs_bias = sHs - sFg * gHg;
    }

    // save last error for derivate part of the controller
    sEs_last = sEs;


    // Compute the force/torque error in the sensor frame
    sEs = sFp * pHp_star - (sHs - sFg * gHg - sHs_bias);

    // fill filtering buffer
    signalBuffer[iter%bufferSize] = sEs[2];

    //filter
    double sum=0;
    unsigned int realBufferSize = iter+1<bufferSize ? iter+1 : bufferSize;
    for(unsigned int i=0; i<realBufferSize;i++) {
      sum+= signalBuffer[i];
    }
    sEs[2] = sum/realBufferSize;

    sEs_sum += sEs;

    // to avoid hudge derivate error at first iteration, we set the derivate error to 0
    if(iter == 0) {
      sEs_last = sEs;
    }

    // Compute the force/torque control law in the sensor frame (propotionnal + derivate controller)
    v_s = lambdaProportionnal * sEs + lambdaDerivate *(sEs - sEs_last) + lambdaIntegral * sEs_sum;

    v_s[0] = 0.0;
    v_s[1] = 0.0;
    v_s[3] = 0.0;
    v_s[4] = 0.0;
    v_s[5] = 0.0;

    vpVelocityTwistMatrix eVs;
    sVe.inverse(eVs);

    ve = eVs * v_s;

    // Get the robot jacobian eJe
    viper->get_eJe(eJe);

    // Compute the joint velocities to achieve the force/torque control
    q_dot = eJe.pseudoInverse() * ve;

    // Send the joint velocities to the robot
    viper->setVelocity(vpRobot::ARTICULAR_FRAME, q_dot);
    iter++;

    } catch (...) {
      viper->setRobotState(vpRobot::STATE_STOP);
      std::cout << "Viper robot could not be initialized" << std::endl;
      emit(robotError());
    }

    // as we are in a loop in a slot, we have to tell to the qapplication to take in account incoming signals and execute corresponding slots
    qApp->processEvents();
    vpTime::wait(t0,deltaTmilliseconds);
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


void usViper850WrapperVelocityControl::startAutomaticForceControl() {

  m_run = false; //stop the running loop

  // reset velocities vector to zero
  for(unsigned int i=0; i<velocityProbeContact.size();i++)
    velocityProbeContact[i]=0;

  if(!viper->getPowerState())
    startRobot();

  m_run = true;
  emit(startControlLoopAutomatic());
}

void usViper850WrapperVelocityControl::stopAutomaticForceControl() {

  m_run = false; //stop the running loop

  // reset velocities vector to zero
  for(unsigned int i=0; i<velocityProbeContact.size();i++)
    velocityProbeContact[i]=0;

  m_run = true;
  emit(startControlLoop()); // go back to manual mode
}
#endif
