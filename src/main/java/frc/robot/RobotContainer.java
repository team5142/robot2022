// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.AutoDriveForward;
import frc.robot.commands.AutoFire;
import frc.robot.commands.AutoHarvest;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.BrakeClimber;
import frc.robot.commands.ExtendGrabber;
import frc.robot.commands.FlywheelSpool;
import frc.robot.commands.Grab;
import frc.robot.commands.LiftClimber;
import frc.robot.commands.LowerClimber;
import frc.robot.commands.RetractGrabber;
import frc.robot.commands.Shoot;
import frc.robot.commands.SpinConveyor;
import frc.robot.commands.TurretLeft;
import frc.robot.commands.TurretPID;
import frc.robot.commands.TurretRight;
import frc.robot.commands.UnbrakeClimber;
import frc.robot.commands.ZeroClimber;
import frc.robot.commands.ZeroTurret;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Turret;
import java.util.function.DoubleSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Joysticks
  private final Joystick m_joystick = new Joystick(0);
  private final Joystick m_joystickR = new Joystick(2);
  private final Joystick m_operator = new Joystick(1);

  private final DoubleSupplier m_forwardAxis = () -> m_joystick.getRawAxis(1);
  //private final DoubleSupplier m_rotationAxis = () -> m_joystick.getRawAxis(0);
  private final DoubleSupplier m_rotationAxis = () -> m_joystickR.getRawAxis(0);

  private final DoubleSupplier m_opForward = () -> m_operator.getRawAxis(3);
  private final DoubleSupplier m_opLeftHorizontal = () -> m_operator.getRawAxis(0);

  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drive = new Drivetrain();
  private final Conveyor m_conveyor = new Conveyor();
  private final Grabber m_grabber = new Grabber();
  private final Turret m_turret = new Turret();
  private final Climber m_climber = new Climber();
  private final Flywheel m_fly = new Flywheel();

  private final AutoDrive m_autoDrive = new AutoDrive(m_drive);
  private final AutoDriveForward m_autoForward = new AutoDriveForward(m_drive);
  private final AutoFire m_autoFire = new AutoFire(m_conveyor, m_fly);
  private final AutoHarvest m_autoHarv = new AutoHarvest(m_grabber);
  private final AutoTurn m_autoTurn = new AutoTurn(m_drive);
  private final ExtendGrabber m_extendGrabber = new ExtendGrabber(m_grabber);
  private final RetractGrabber m_retractGrabber = new RetractGrabber(m_grabber);
  private final Grab m_grab = new Grab(m_grabber, m_opForward);
  private final TurretRight m_turrRight = new TurretRight(m_turret);
  private final TurretLeft m_turrLeft = new TurretLeft(m_turret);
  private final TurretPID m_turrPID = new TurretPID(m_turret, m_opLeftHorizontal);
  private final ZeroTurret m_turrZero = new ZeroTurret(m_turret);
  private final LiftClimber m_liftClimber = new LiftClimber(m_climber);
  private final LowerClimber m_lowerClimber = new LowerClimber(m_climber);
  private final SpinConveyor m_manConv = new SpinConveyor(m_conveyor, m_opForward);
  private final FlywheelSpool m_flySpool = new FlywheelSpool(m_fly);
  private final ZeroClimber m_climberZero = new ZeroClimber(m_climber);
  private final Shoot m_shoot = new Shoot(m_conveyor);
  private final BrakeClimber m_brakeClimb = new BrakeClimber(m_climber);
  private final UnbrakeClimber m_unbrakeClimb = new UnbrakeClimber(m_climber);
  private final InstantCommand m_driveFlip = new InstantCommand(m_drive::toggleDriveDirction, m_drive);
  private final InstantCommand m_resetNav = new InstantCommand(m_drive::resetNav, m_drive);
  private final ParallelCommandGroup m_autoHarvDrive = new ParallelCommandGroup(m_autoHarv, m_autoDrive);
  // private final SequentialCommandGroup m_auto = new SequentialCommandGroup(m_autoForward, m_autoFire, m_autoHarvDrive);
  private final SequentialCommandGroup m_auto = new SequentialCommandGroup(m_autoFire, m_resetNav, m_autoTurn, m_autoHarvDrive);
  // private final SequentialCommandGroup m_auto = new SequentialCommandGroup(m_autoFire, m_autoHarvDrive);

  private final ArcadeDrive m_arcadeDrive =
      new ArcadeDrive(m_drive, m_forwardAxis, m_rotationAxis);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings 
    configureButtonBindings();
  }

  // Button -> Command Mapping
  private void configureButtonBindings() {
    new JoystickButton(m_joystick, 5).whenPressed(m_turrZero);
    new JoystickButton(m_joystick, 10).whenPressed(m_climberZero);
    new JoystickButton(m_joystick, 1).whenPressed(m_driveFlip);

    new JoystickButton(m_operator, 6).whenPressed(m_retractGrabber.withTimeout(2));
    new JoystickButton(m_operator, 8).whenPressed(m_extendGrabber.withTimeout(2));
    new JoystickButton(m_operator, 1).whileHeld(m_turrPID);
    // new JoystickButton(m_operator, 5).whileHeld(autoAim);
    new JoystickButton(m_operator, 5).whileHeld(m_flySpool);
    new JoystickButton(m_operator, 7).whileHeld(m_shoot);
    new POVButton(m_operator, 0).whileHeld(m_liftClimber);
    new POVButton(m_operator, 180).whileHeld(m_lowerClimber);
    new POVButton(m_operator, 90).whenPressed(m_brakeClimb.withTimeout(2));
    new POVButton(m_operator, 270).whenPressed(m_unbrakeClimb.withTimeout(2));
    m_drive.setDefaultCommand(m_arcadeDrive);
    m_grabber.setDefaultCommand(m_grab);
    m_conveyor.setDefaultCommand(m_manConv);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_auto;
  }
}
