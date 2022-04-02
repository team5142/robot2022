// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AutoTurn extends CommandBase {
  /** Creates a new AutoTurn. */
  private Drivetrain m_drive;

  private PIDController m_controller;
  private Timer m_timer = new Timer();

  public AutoTurn(Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_controller = new PIDController(0.1, 0, 0);
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.zeroHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = m_controller.calculate(m_drive.getHeading(), 20);
    if (output > .4) {
      output = .4;
    } else if (output < -.4) {
      output = -.4;
    }
    m_drive.arcadeDrive(0, output);
    if (m_drive.getHeading() >= 15 && m_drive.getHeading() <= 25) {
      if (m_timer.get() == 0) {
        m_timer.start();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_timer.get() > 2 || Timer.getMatchTime() < 7) {
      return true;
    }
    return false;
  }
}
