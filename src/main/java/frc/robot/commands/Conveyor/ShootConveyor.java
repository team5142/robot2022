// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Conveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Flywheel;

public class ShootConveyor extends CommandBase {
  /** Creates a new ManualConveyor. */
  private final Conveyor m_conveyor;

  private final Flywheel m_flywheel;

  public ShootConveyor(Conveyor conveyor, Flywheel flywheel) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_conveyor = conveyor;
    m_flywheel = flywheel;

    addRequirements(conveyor, flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_conveyor.fire(m_flywheel.getShooterRPM());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_conveyor.endfire();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
