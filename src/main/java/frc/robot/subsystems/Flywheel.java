// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;

/**
 * The Flywheel Class controls the flywheel shooter.
 *
 * @author Spencer Greene & Gavin Popkin
 */
public class Flywheel extends SubsystemBase {
  private final CANSparkMax m_flyLeft;
  private final CANSparkMax m_flyRight;

  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;

  private final PIDController m_controller;
  private final SimpleMotorFeedforward m_feedforward;
  /** Creates a new Flywheel. */
  public Flywheel() {
    m_flyLeft = new CANSparkMax(FlywheelConstants.kFlyLeft, MotorType.kBrushless);
    m_flyRight = new CANSparkMax(FlywheelConstants.kFlyRight, MotorType.kBrushless);
    m_flyLeft.restoreFactoryDefaults();
    m_flyRight.restoreFactoryDefaults();
    m_flyRight.follow(m_flyLeft, true);
    m_flyLeft.setIdleMode(IdleMode.kCoast);
    m_flyRight.setIdleMode(IdleMode.kCoast);

    m_leftEncoder = m_flyLeft.getEncoder();
    m_rightEncoder = m_flyRight.getEncoder();

    m_controller =
        new PIDController(FlywheelConstants.kP, FlywheelConstants.kI, FlywheelConstants.kD);
    m_feedforward =
        new SimpleMotorFeedforward(
            FlywheelConstants.kS, FlywheelConstants.kV, FlywheelConstants.kA);
  }

  /**
   * Gets the encoder's RPM
   *
   * @return the encoder's RPM.
   */
  public double getLeftRPM() {
    return m_leftEncoder.getVelocity();
  }

  public double getRightRPM() {
    return m_rightEncoder.getVelocity();
  }

  /** Manually sets the throttle of the flywheel. */
  public void set() {
    m_flyLeft.set(0.5);
    m_flyRight.set(0.5);
  }

  /** Stops the flywheel. */
  public void stop() {
    m_flyLeft.stopMotor();
    m_flyRight.stopMotor();
  }

  /**
   * Set the RPM of the flywheel.
   *
   * @param desiredVelocity
   */
  public void setRPM(double desiredVelocity) {
    m_flyLeft.setVoltage(
        m_controller.calculate(m_leftEncoder.getVelocity(), desiredVelocity)
            + m_feedforward.calculate(desiredVelocity));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("FlywheelLeftRPM", getLeftRPM());
    SmartDashboard.putNumber("FlywheelRightRPM", getRightRPM());
  }
}
