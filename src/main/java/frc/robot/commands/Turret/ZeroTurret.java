// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class ZeroTurret extends CommandBase {
  /** Creates a new ZeroTurret. */
  private final Turret m_turret;

  public ZeroTurret(Turret turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turret;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turret.zeroEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
