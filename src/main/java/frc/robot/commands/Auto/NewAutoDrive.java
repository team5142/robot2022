// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class NewAutoDrive extends CommandBase {
  /** Creates a new NewAutoDrive. */
  private final Drivetrain m_drive;
  private double m_forward;
  private double m_rotation;
  private String m_type;
  public NewAutoDrive(Drivetrain drive, double forward, double rotation, String type) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_forward = forward;
    m_rotation = rotation;
    m_type = type;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    m_drive.arcadeDrive(m_forward, m_rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
    m_drive.resetEncoders();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_type.equals("AutoRev")) {
      if(m_drive.getAverageEncoderDistance() < -1.6) {
        return true;
      }
      }
    else if(m_type.equals("AutoRev2")) {
      if(m_drive.getAverageEncoderDistance() < 2) {
        return true;
      }
    } else if (m_type.equals("AutoFwd")) {
      if(m_drive.getAverageEncoderDistance() > 1.1) {
        return true;
      }
    }
    return false;
  }
}
