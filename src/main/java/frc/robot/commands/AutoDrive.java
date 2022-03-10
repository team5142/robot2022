// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AutoDrive extends CommandBase {
  /** Creates a new AutoDrive. */
  private final Drivetrain m_drive;

  public AutoDrive(Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.arcadeDrive(1, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double avg = (m_drive.getLeftEncoder() + m_drive.getRightEncoder()) / 2;
    if (avg > 30000 || Timer.getMatchTime() < 10) {
      return true;
    }
    return false;
  }
}
