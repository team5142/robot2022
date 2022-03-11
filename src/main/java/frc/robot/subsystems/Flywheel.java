// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;

/**
 * The Flywheel Class controls the flywheel shooter. 
 * 
 * @author Spencer Greene & Gavin Popkin
 */
public class Flywheel extends SubsystemBase {
  private final CANSparkMax m_flyLeft =
      new CANSparkMax(FlywheelConstants.kFlyLeft, MotorType.kBrushless);
  private final CANSparkMax m_flyRight =
      new CANSparkMax(FlywheelConstants.kFlyRight, MotorType.kBrushless);
  private final RelativeEncoder m_enc;
  private final RelativeEncoder m_encRight;
  private final SparkMaxPIDController m_pidLeft;
  private final SparkMaxPIDController m_pidRight;
  /** Creates a new Flywheel. */
  public Flywheel() {
    m_flyLeft.restoreFactoryDefaults();
    m_flyRight.restoreFactoryDefaults();
    m_flyRight.setInverted(true);
    m_flyLeft.setIdleMode(IdleMode.kCoast);
    m_flyRight.setIdleMode(IdleMode.kCoast);
    // m_flyRight.follow(m_flyLeft, true);
    m_enc = m_flyLeft.getEncoder();
    m_encRight = m_flyRight.getEncoder();
    m_pidLeft = m_flyLeft.getPIDController();
    m_pidLeft.setP(FlywheelConstants.kP);
    m_pidLeft.setI(FlywheelConstants.kI);
    m_pidLeft.setD(FlywheelConstants.kD);
    m_pidLeft.setIZone(FlywheelConstants.kIz);
    m_pidLeft.setFF(FlywheelConstants.kFF);
    m_pidLeft.setOutputRange(FlywheelConstants.kMinOut, FlywheelConstants.kMaxOut);
    m_flyLeft.setClosedLoopRampRate(0.5);
    m_pidRight = m_flyRight.getPIDController();
    m_pidRight.setP(FlywheelConstants.kP);
    m_pidRight.setI(FlywheelConstants.kI);
    m_pidRight.setD(FlywheelConstants.kD);
    m_pidRight.setIZone(FlywheelConstants.kIz);
    m_pidRight.setFF(FlywheelConstants.kFF);
    m_pidRight.setOutputRange(FlywheelConstants.kMinOut, FlywheelConstants.kMaxOut);
    m_flyRight.setClosedLoopRampRate(0.5);
  }

  /**
   * Gets the encoder's RPM
   * @return the encoder's RPM.
   */
  public double getRPM() {
    return m_enc.getVelocity();
  }

  /**
   * Manually set the velocity of the flywheel to full throttle. 
   */
  public void set() {
    m_flyLeft.set(0.5);
    m_flyRight.set(0.5);
  }

  public void stop() {
    m_flyLeft.stopMotor();
    m_flyRight.stopMotor();
  }

  public void setRPM() {
    m_pidLeft.setReference(5300, CANSparkMax.ControlType.kVelocity);
    m_pidRight.setReference(5300, CANSparkMax.ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("FlywheelLeftRPM", getRPM());
    SmartDashboard.putNumber("FlywheelRightRPM", m_encRight.getVelocity());
  }
}
