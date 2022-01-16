// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private WPI_TalonSRX m_rightMaster;
  private WPI_TalonSRX m_rightSlave;
  private WPI_TalonSRX m_leftMaster;
  private WPI_TalonSRX m_leftSlave;

  private DifferentialDrive drive;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    m_rightMaster = new WPI_TalonSRX(Constants.DriveConstants.kRightMaster);
    m_rightSlave = new WPI_TalonSRX(Constants.DriveConstants.kRightSlave);
    m_leftMaster = new WPI_TalonSRX(Constants.DriveConstants.kLeftMaster);
    m_leftSlave = new WPI_TalonSRX(Constants.DriveConstants.kLeftSlave);

    m_rightSlave.follow(m_rightMaster);
    m_leftSlave.follow(m_rightMaster);
    m_rightMaster.setInverted(true);

    drive = new DifferentialDrive(m_leftMaster, m_rightMaster);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
