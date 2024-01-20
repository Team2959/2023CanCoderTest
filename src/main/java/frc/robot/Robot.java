// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
 
  private static final double kSteerP = 0.4;
  private static final double kSteerI = 0.00001;
  private static final double kSteerD = 0.0;
  private static final double kSteerIZone = 1.0;
  private static final double kSteerMotorRotationsPerRevolution = 12.75;

  private CANSparkMax m_steerMotor;
  private CANCoder m_steerAbsoluteEncoder;
  private SparkMaxRelativeEncoder m_steerEncoder;
  private SparkMaxPIDController m_steerPIDController;

  private PIDController m_absoluteAngleController = new PIDController(0.3, 0.00002, kSteerD);

  private double m_targetAngleInRadians = 0.0;
  private boolean m_controlWithAbsoluteAngle = false;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    m_steerMotor = new CANSparkMax(11, CANSparkMax.MotorType.kBrushless);
    m_steerMotor.restoreFactoryDefaults();
    m_steerMotor.setIdleMode(IdleMode.kCoast);

    m_steerAbsoluteEncoder = new CANCoder(1);

    m_steerEncoder = (SparkMaxRelativeEncoder) m_steerMotor.getEncoder();
    m_steerPIDController = m_steerMotor.getPIDController();

    m_steerPIDController.setFeedbackDevice(m_steerEncoder);
    m_steerPIDController.setP(kSteerP);
    m_steerPIDController.setI(kSteerI);
    m_steerPIDController.setD(kSteerD);
    m_steerPIDController.setIZone(kSteerIZone);

    m_steerEncoder.setPositionConversionFactor(2 * Math.PI / kSteerMotorRotationsPerRevolution);

    enableLiveWindowInTest(false);

    SmartDashboard.putNumber("Steer Relative Encoder", 0);
    SmartDashboard.putNumber("Steer Absolute Encoder", 0);

    SmartDashboard.putBoolean("Reset Relative Encoder", false);

    SmartDashboard.putNumber("Target Angle", 0);
    SmartDashboard.putBoolean("Go To Target Angle", false);

    SmartDashboard.putNumber("Rotation Speed",0);

    SmartDashboard.putBoolean("AE True/RE False", true);

    SmartDashboard.putNumber("Change in Angle Position",0);

    SmartDashboard.putNumber("RE motor rotations to revolution", kSteerMotorRotationsPerRevolution);
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
    m_steerMotor.set(0);

    double targetAngleInDegress = SmartDashboard.getNumber("Target Angle", 0);
    m_targetAngleInRadians = Rotation2d.fromDegrees(targetAngleInDegress).getRadians();
    setRelativeEncoderConversion();
    m_controlWithAbsoluteAngle = SmartDashboard.getBoolean("AE True/RE False",false);
    m_absoluteAngleController.setSetpoint(m_targetAngleInRadians);
  }

  private void setRelativeEncoderConversion() {
    double x = SmartDashboard.getNumber("RE motor rotations to revolution",kSteerMotorRotationsPerRevolution);
    m_steerEncoder.setPositionConversionFactor(2 * Math.PI / x);
  }
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    double absoluteInDegrees = (m_steerAbsoluteEncoder.getAbsolutePosition());
    double absoluteInRadians = Rotation2d.fromDegrees(absoluteInDegrees).getRadians();

    SmartDashboard.putNumber("Steer Relative Encoder", Rotation2d.fromRadians(m_steerEncoder.getPosition()).getDegrees());
    SmartDashboard.putNumber("Steer Absolute Encoder", absoluteInDegrees);  
    if (m_controlWithAbsoluteAngle)
    {
      double raw = m_absoluteAngleController.calculate(absoluteInRadians);
      m_steerMotor.set(raw);
    }
    else
    {
      m_steerPIDController.setReference(m_targetAngleInRadians, CANSparkMax.ControlType.kPosition);
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    m_steerMotor.set(0);
    setRelativeEncoderConversion();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Steer Relative Encoder", Rotation2d.fromRadians(m_steerEncoder.getPosition()).getDegrees());
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
  public void testInit() {
    m_steerMotor.set(0);
    m_steerEncoder.setPositionConversionFactor(1.0);
    m_steerEncoder.setPosition(0);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    SmartDashboard.putNumber("Steer Relative Encoder", m_steerEncoder.getPosition());
    SmartDashboard.putNumber("Steer Absolute Encoder", m_steerAbsoluteEncoder.getAbsolutePosition());  

    if (SmartDashboard.getBoolean("Reset Relative Encoder", false))
    {
      m_steerEncoder.setPosition(0);
      m_steerMotor.set(0);
      SmartDashboard.putBoolean("Reset Relative Encoder", false);
      return;
    }
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
