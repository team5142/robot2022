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
  private final CANSparkMax m_shooter;
  private final CANSparkMax m_bumper;

  private final RelativeEncoder m_shooterEnc;
  private final RelativeEncoder m_bumperEnc;

  private final PIDController m_shooterController;
  private final SimpleMotorFeedforward m_shooterFeedForward;
  private final PIDController m_bumperController;
  private final SimpleMotorFeedforward m_bumperFeedForward;
  /** Creates a new Flywheel. */
  public Flywheel() {
    m_shooter = new CANSparkMax(FlywheelConstants.kFlyLeft, MotorType.kBrushless);
    m_bumper = new CANSparkMax(FlywheelConstants.kFlyRight, MotorType.kBrushless);
    m_shooter.restoreFactoryDefaults();
    m_bumper.restoreFactoryDefaults();
    m_shooter.setInverted(false);
    m_bumper.setInverted(false);
    m_shooter.setIdleMode(IdleMode.kCoast);
    m_bumper.setIdleMode(IdleMode.kCoast);

    m_shooterEnc = m_shooter.getEncoder();
    m_bumperEnc = m_bumper.getEncoder();

    m_shooterController =
        new PIDController(FlywheelConstants.kP, FlywheelConstants.kI, FlywheelConstants.kD);
    m_shooterFeedForward =
        new SimpleMotorFeedforward(
            FlywheelConstants.kS, FlywheelConstants.kV, FlywheelConstants.kA);
    m_bumperController =
        new PIDController(FlywheelConstants.kP, FlywheelConstants.kI, FlywheelConstants.kD);
    m_bumperFeedForward =
        new SimpleMotorFeedforward(
            FlywheelConstants.kS, FlywheelConstants.kV, FlywheelConstants.kA);
  }

  /**
   * Gets the encoder's RPM
   *
   * @return the encoder's RPM.
   */
  public double getShooterRPM() {
    return m_shooterEnc.getVelocity();
  }

  public double getBumperRPM() {
    return m_bumperEnc.getVelocity();
  }

  /** Manually sets the throttle of the flywheel. */
  public void set() {
    m_shooter.set(0.5);
    // m_flyRight.set(0.5);
  }

  /** Stops the flywheel. */
  public void stop() {
    m_shooter.stopMotor();
    m_bumper.stopMotor();
  }

  /**
   * Set the RPM of the flywheel.
   *
   * @param desiredVelocity
   */
  public void setRPM(double desiredVelocity) {
    m_shooter.setVoltage(
        m_shooterController.calculate(m_shooterEnc.getVelocity(), desiredVelocity)
            + m_shooterFeedForward.calculate(desiredVelocity));
    // m_bumper.setVoltage(
    //     m_bumperController.calculate(m_bumperEnc.getVelocity(), desiredVelocity)
    //         + m_bumperFeedForward.calculate(desiredVelocity));
    m_bumper.set(0.75);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("FlywheelLeftRPM", getShooterRPM());
    SmartDashboard.putNumber("FlywheelRightRPM", getBumperRPM());
  }
}
