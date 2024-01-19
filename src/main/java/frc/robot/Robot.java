// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
 
  private CANSparkMax m_steerMotor;
  private CANCoder m_steerAbsoluteEncoder;
  private SparkMaxRelativeEncoder m_steerEncoder;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    m_steerMotor = new CANSparkMax(11, CANSparkMax.MotorType.kBrushless);
    m_steerMotor.restoreFactoryDefaults();
    // m_steerMotor.setIdleMode(IdleMode.kBrake);

    m_steerAbsoluteEncoder = new CANCoder(2);

    m_steerEncoder = (SparkMaxRelativeEncoder) m_steerMotor.getEncoder();

    enableLiveWindowInTest(false);

    SmartDashboard.putNumber("Steer Relative Encoder", 0);
    SmartDashboard.putNumber("Steer Absolute Encoder", 0);

    SmartDashboard.putBoolean("Reset Relative Encoder", false);

    SmartDashboard.putNumber("Target Angle", 0);
    SmartDashboard.putBoolean("Go To Target Angle", false);

    SmartDashboard.putNumber("Rotation Speed",0);

    SmartDashboard.putBoolean("AE True/RE False", true);

    SmartDashboard.putNumber("Change in Angle Position",0);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // a change to commit to git
    // second chage
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Steer Relative Encoder", m_steerEncoder.getPosition());
    SmartDashboard.putNumber("Steer Absolute Encoder", m_steerAbsoluteEncoder.getAbsolutePosition());  


    }
 
 

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    double absoluteAngle = ReadAbsoluteAngle();
    double relativeEncoder = m_steerEncoder.getPosition();  // rotations

    SmartDashboard.putNumber("Steer Relative Encoder", relativeEncoder);
    SmartDashboard.putNumber("Steer Absolute Encoder", absoluteAngle);  

    if (SmartDashboard.getBoolean("Reset Relative Encoder", false))
    {
      m_steerEncoder.setPosition(0);
      m_steerMotor.set(0);
      SmartDashboard.putBoolean("Reset Relative Encoder", false);
      return;
    }

    if (SmartDashboard.getBoolean("Go To Target Angle", false))
    {
      double target = SmartDashboard.getNumber("Target Angle", 0);
      double currentPosition = 0;

      boolean aeEncoder = SmartDashboard.getBoolean("AE True/RE False",false);
      if (aeEncoder)
      {
        target = CorrectAngleTo180(target);
        currentPosition = absoluteAngle;
      }
      else
      {
        currentPosition = relativeEncoder;
      }

      double rotationSpeed = SmartDashboard.getNumber("Rotation Speed",0);
      double difference = target - currentPosition;
    
      if (aeEncoder)
      {
        //AE stuff
        if (Math.abs(difference) > 180)
        difference = -difference;
      }

      double deltaAngle = SmartDashboard.getNumber("Change in Angle Position", 0.5);

      if (difference > deltaAngle)
      {
        // spin clockwise
        m_steerMotor.set(rotationSpeed);
      }
      else if (difference < -deltaAngle)
      {
        // spin counter-clockwise
        
        m_steerMotor.set(-rotationSpeed);
      }
      else
      {
        // stop!
        m_steerMotor.set(0);
        SmartDashboard.putBoolean("Go To Target Angle", false);
      }
    }
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  private double ReadAbsoluteAngle()
  {
    return CorrectAngleTo180(m_steerAbsoluteEncoder.getAbsolutePosition());
  }

  private double CorrectAngleTo180(double angle)
  {
    if (angle > 180)
    {
      angle = angle - 360;
    }
    
    return angle;
  }
}
