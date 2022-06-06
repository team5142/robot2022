// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;

/**
 * The Flywheel Class controls the flywheel shooter.
 *
 * @author Spencer Greene & Gavin Popkin
 */
public class Flywheel extends SubsystemBase {
  private final CANSparkMax m_shooter;

  private final RelativeEncoder m_shooterEnc;

  private final SparkMaxPIDController m_shooterController;
  //private final SimpleMotorFeedforward m_shooterFeedForward;
  /** Creates a new Flywheel. */
  public Flywheel() {
    m_shooter = new CANSparkMax(FlywheelConstants.kFlyLeft, MotorType.kBrushless);
    m_shooter.restoreFactoryDefaults();
    m_shooter.setInverted(false);
    m_shooter.setIdleMode(IdleMode.kCoast);

    m_shooterEnc = m_shooter.getEncoder();
    m_shooterController = m_shooter.getPIDController();
    m_shooterController.setP(FlywheelConstants.kP);
    m_shooterController.setI(FlywheelConstants.kI);
    m_shooterController.setD(FlywheelConstants.kD);
    m_shooterController.setFF(FlywheelConstants.kFF);
  }

  /**
   * Gets the encoder's RPM
   *
   * @return the encoder's RPM.
   */
  public double getShooterRPM() {
    return m_shooterEnc.getVelocity();
  }

  /** Manually sets the throttle of the flywheel. */
  public void set() {
    m_shooter.set(0.5);
  }

  /** Stops the flywheel. */
  public void stopFlywheel() {
    m_shooter.stopMotor();
  }

  public void flywheelRPM(double flyVel) {
    m_shooterController.setReference(flyVel, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("FlywheelRPM", getShooterRPM());
  }
}
