// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Flywheel;

public class AutoFire extends CommandBase {
  /** Creates a new AutoFire. */
  private final Conveyor m_conveyor;

  private final Flywheel m_fly;

  public AutoFire(Conveyor conv, Flywheel fly) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_conveyor = conv;
    m_fly = fly;
    addRequirements(m_conveyor, m_fly);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_fly.flywheelRPM(2200);
    m_conveyor.fire(m_fly.getShooterRPM());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_conveyor.endfire();
    m_fly.stopFlywheel();
  }

  // Returns true when the command should end.
  public boolean isFinished() {
    if (Timer.getMatchTime() < 4) {
      return true;
    }
    return false;
  }
}
