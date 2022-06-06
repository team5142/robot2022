// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import java.util.function.DoubleSupplier;

public class ArcadeDrive extends CommandBase {
  private final Drivetrain m_drive;
  private DoubleSupplier m_forward, m_rotation;
  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(Drivetrain drive, DoubleSupplier forward, DoubleSupplier rotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_forward = forward;
    m_rotation = rotation;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_drive.test();
    // if (m_grab.readUltrasound() > 1600) {
    //   m_drive.arcadeDrive(m_forward.getAsDouble(), m_rotation.getAsDouble());
    // }
    m_drive.arcadeDrive(-m_forward.getAsDouble(), m_rotation.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
