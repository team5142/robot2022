// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import java.lang.Math;
import java.util.function.DoubleSupplier;

public class TurretPID extends CommandBase {
  /** Creates a new TurretPID. */
  private final Turret m_turret;
  private static DoubleSupplier target;

  public TurretPID(Turret turret, DoubleSupplier setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turret;
    addRequirements(turret);
    target=setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_turret.setTarget(target.getAsDouble()*90);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
