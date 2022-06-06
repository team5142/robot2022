// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;
import edu.wpi.first.wpilibj.Timer;

public class Conveyor extends SubsystemBase {
  private final WPI_TalonSRX m_conveyor = new WPI_TalonSRX(ConveyorConstants.kConveyorSRX);
  private final CANSparkMax m_bumper =
      new CANSparkMax(ConveyorConstants.kBumper, MotorType.kBrushless);
  private final RelativeEncoder m_bumperEnc;
  // private final PIDController m_bumperController;
  // private final SimpleMotorFeedforward m_bumperFeedForward;
  // private double m_bumperPer = 0;
  private double m_conveyorPer = 0;
  private int m_conveyorState = 2;
  private double m_shooterVel = 0;
  private double fireTime = 0;
  /** 0=do nothing 1=fire 2=defualt/run conveyor normally */

  /** Creates a new Coveyor. */
  public Conveyor() {
    m_conveyor.configFactoryDefault();
    m_conveyor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    m_bumper.restoreFactoryDefaults();
    m_bumper.setInverted(false);
    m_bumper.setIdleMode(IdleMode.kCoast);
    m_bumperEnc = m_bumper.getEncoder();
    // m_bumperController =
    //     new PIDController(
    //         ConveyorConstants.kPBump, ConveyorConstants.kIBump, ConveyorConstants.kDBump);
    // m_bumperFeedForward =
    //     new SimpleMotorFeedforward(
    //         ConveyorConstants.kSBump, ConveyorConstants.kVBump, ConveyorConstants.kABump);

  }

  public void fire(double shooterVel) {
    m_conveyorState = 1;
    m_shooterVel = shooterVel;
  }

  public void endfire() {
    m_conveyorState = 2;
  }

  public void stopConveyor() {
    m_conveyorPer = 0;
  }

  public void manual(double speed) {
    m_conveyorPer = speed;
  }

  public double getBumperRPM() {
    return m_bumperEnc.getVelocity();
  }

  public void opposeIntake() {
    m_bumper.set(-0.5);
  }

  public void stopBumper() {
    m_bumper.stopMotor();
  }

  // public void bumperOut(double bumpVel) {
  //   m_bumperPer=bumpVel;
  // }

  // public void bumperRPM(double bumpVel) {
  //   m_bumper.setVoltage(
  //       m_bumperController.calculate(m_bumperEnc.getVelocity(), bumpVel)
  //           + m_bumperFeedForward.calculate(bumpVel));
  // }

  @Override
  public void periodic() {
    double conveyorOut = 0;
    double bumperOut = 0;
    switch (m_conveyorState) {
      case 0: // Stop
        conveyorOut = 0;
        bumperOut = 0;
        break;
      case 1: // Fire
        if (m_shooterVel >= 2000) {
          conveyorOut = 0.5;
          fireTime++;
          if (fireTime>=80) {
            bumperOut = .60;
          }
        } else {
          bumperOut = -0.5;
          conveyorOut = 0;
          fireTime = 0;
        }
        break;
      case 2: // Run Conveyor
        conveyorOut = m_conveyorPer;
        if (Math.abs(m_conveyorPer) > 0.1) {
          bumperOut = -0.5;
        } else {
          bumperOut = 0;
        }
        break;
    }
    m_bumper.set(bumperOut);
    m_conveyor.set(conveyorOut);
    //SmartDashboard.putNumber("ShooterVel",);
  }
}
