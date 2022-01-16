// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.nio.ReadOnlyBufferException;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Grabber;

public class ArcadeDrive extends CommandBase {
  private final Drivetrain m_drive;
  private final Grabber m_grab;
  private DoubleSupplier m_z, m_x;
  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(Drivetrain drive, Grabber grab, DoubleSupplier z, DoubleSupplier x) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_grab = grab;
    m_z = z;
    m_x = x;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_grab.readUltrasound() > 1600) {
      m_drive.arcadeDrive(m_x.getAsDouble(), m_z.getAsDouble());
    }
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
