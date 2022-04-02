// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Grabber;
import java.util.function.DoubleSupplier;

public class Grab extends CommandBase {
  private final Grabber m_grabber;
  private DoubleSupplier m_speed;
  /** Creates a new Grab. */
  public Grab(Grabber grabber, DoubleSupplier speed) {
    m_grabber = grabber;
    m_speed = speed;
    addRequirements(grabber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_grabber.startGrab(m_speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_grabber.stopGrab();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
