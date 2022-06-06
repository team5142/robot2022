// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Flywheel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Flywheel;

public class LowShot extends CommandBase {
  /** Creates a new LowShot. */
  private final Flywheel m_fly;
  private final Conveyor m_conv;
  public LowShot(Flywheel fly, Conveyor conv) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_fly = fly;
    m_conv = conv;
    addRequirements(m_fly, m_conv);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_fly.flywheelRPM(800);
    m_conv.fire(3200);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_fly.stopFlywheel();
    m_conv.endfire();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
