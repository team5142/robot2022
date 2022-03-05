// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {
  private final CANSparkMax m_turret =
      new CANSparkMax(TurretConstants.kTurret, MotorType.kBrushless);
  private final SparkMaxPIDController m_pidController;
  private final RelativeEncoder m_encoder;
  public double desiredPos;
  private final PhotonCamera m_camera = new PhotonCamera("photonvision");
  /** Creates a new Turret. */
  public Turret() {
    m_turret.restoreFactoryDefaults();
    // m_turret.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    // m_turret.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    m_pidController = m_turret.getPIDController();
    m_encoder = m_turret.getEncoder();
    m_pidController.setP(TurretConstants.kP);
    m_pidController.setI(TurretConstants.kI);
    m_pidController.setD(TurretConstants.kD);
    m_pidController.setIZone(TurretConstants.kIz);
    m_pidController.setFF(TurretConstants.kFF);
    m_pidController.setOutputRange(TurretConstants.kMinOut, TurretConstants.kMaxOut);
    m_camera.setLED(VisionLEDMode.kOn);
  }

  public PhotonTrackedTarget getResult() {
    return m_camera.getLatestResult().getBestTarget();
  }
  
  public void stopMotor() {
    m_turret.stopMotor();
  }

  public void turnRight() {
    m_turret.set(0.25);
  }

  public void turnLeft() {
    m_turret.set(-0.25);
  }

  public void zeroEncoder() {
    m_encoder.setPosition(0);
  }

  public double getPos() {
    return m_encoder.getPosition();
  }

  public double getDeg() {
    return m_encoder.getPosition() * 1.93;
  }

  public void setTarget(double target) {
    m_pidController.setReference(target, CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Enc Pos", getPos());
    SmartDashboard.putNumber("Enc Deg", getDeg());
  }
}
