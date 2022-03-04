// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;

public class Flywheel extends SubsystemBase {
  private final CANSparkMax m_flyLeft =
      new CANSparkMax(FlywheelConstants.kFlyLeft, MotorType.kBrushless);
  private final CANSparkMax m_flyRight =
      new CANSparkMax(FlywheelConstants.kFlyRight, MotorType.kBrushless);
  /** Creates a new Flywheel. */
  public Flywheel() {
    m_flyRight.follow(m_flyLeft, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
