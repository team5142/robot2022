// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class Conveyor extends SubsystemBase {
  private final WPI_TalonSRX m_conveyor = new WPI_TalonSRX(ConveyorConstants.kConveyorSRX);
  public final DigitalInput m_photoBase = new DigitalInput(ConveyorConstants.kPhotoBase);
  public final DigitalInput m_photoMid = new DigitalInput(ConveyorConstants.kPhotoMid);
  public final DigitalInput m_photoTop = new DigitalInput(ConveyorConstants.kPhotoTop);
  public final Counter m_counter = new Counter(ConveyorConstants.kCounter);
  private static int m_pos = 0;
  private static int m_count = 0;
  /** Creates a new Coveyor. */
  public Conveyor() {
    m_conveyor.configFactoryDefault();
    m_conveyor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
  }

  public void increment() {
    // run motion magic
    m_pos = 1;
    m_count = 1;
  }

  public void intakeFirst() {
    if (!m_photoBase.get() && m_count == 0) {
      m_conveyor.set(ControlMode.PercentOutput, 0.5);
    }
  }

  public boolean getBase() {
    return m_photoBase.get();
  }

  public boolean getMid() {
    return m_photoMid.get();
  }

  public boolean getTop() {
    return m_photoTop.get();
  }

  public void stop() {
    m_conveyor.stopMotor();
  }

  public void manual(double speed) {
    m_conveyor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
