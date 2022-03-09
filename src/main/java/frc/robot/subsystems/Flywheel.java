// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;

public class Flywheel extends SubsystemBase {
  private final CANSparkMax m_flyLeft =
      new CANSparkMax(FlywheelConstants.kFlyLeft, MotorType.kBrushless);
  private final CANSparkMax m_flyRight =
      new CANSparkMax(FlywheelConstants.kFlyRight, MotorType.kBrushless);
  private final RelativeEncoder m_enc;
  private final RelativeEncoder m_encRight;
  private final SparkMaxPIDController m_pid;
  /** Creates a new Flywheel. */
  public Flywheel() {
    m_flyLeft.restoreFactoryDefaults();
    m_flyRight.restoreFactoryDefaults();
    // m_flyRight.follow(m_flyLeft, true);
    m_enc = m_flyLeft.getEncoder();
    m_encRight = m_flyRight.getEncoder();
    m_pid = m_flyLeft.getPIDController();
    m_pid.setP(FlywheelConstants.kP);
    m_pid.setI(FlywheelConstants.kI);
    m_pid.setD(FlywheelConstants.kD);
    m_pid.setIZone(FlywheelConstants.kIz);
    m_pid.setFF(FlywheelConstants.kFF);
    m_pid.setOutputRange(FlywheelConstants.kMinOut, FlywheelConstants.kMaxOut);
  }

  public double getRPM() {
    return m_enc.getVelocity();
  } 

  public void set() {
    // m_flyLeft.set(0.5);
    m_flyLeft.set(1);
  }

  public void stop() {
    // m_flyLeft.stopMotor();
    m_flyLeft.stopMotor();
  }

  public void setRPM() {
    m_pid.setReference(3000, CANSparkMax.ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("FlywheelLeftRPM", getRPM());
    SmartDashboard.putNumber("FlywheelRightRPM", m_encRight.getVelocity());
  }
}
