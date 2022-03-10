// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ExtendGrabber;
import frc.robot.commands.FlywheelSpool;
import frc.robot.commands.Grab;
import frc.robot.commands.LiftClimber;
import frc.robot.commands.LowerClimber;
import frc.robot.commands.SpinConveyor;
import frc.robot.commands.RetractGrabber;
import frc.robot.commands.TurretLeft;
import frc.robot.commands.TurretRight;
import frc.robot.commands.TurretPID;
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
  private final Joystick m_operator = new Joystick(1);

  private final DoubleSupplier m_forwardAxis = () -> m_joystick.getRawAxis(1);
  private final DoubleSupplier m_rotationAxis = () -> m_joystick.getRawAxis(0);

  private final DoubleSupplier m_opForward = () -> m_operator.getRawAxis(5);
  private final DoubleSupplier m_opLeftHorizontal = () -> m_operator.getRawAxis(0);

  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drive = new Drivetrain();
  private final Conveyor m_conveyor = new Conveyor();
  private final Grabber m_grabber = new Grabber(m_conveyor);
  private final Turret m_turret = new Turret();
  private final Climber m_climber = new Climber();
  private final Flywheel m_fly = new Flywheel();

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

  private final ArcadeDrive m_arcadeDrive =
      new ArcadeDrive(m_drive, m_grabber, m_forwardAxis, m_rotationAxis);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  // Button -> Command Mapping
  private void configureButtonBindings() {
    new JoystickButton(m_joystick, 3).whileHeld(m_turrLeft);
    new JoystickButton(m_joystick, 4).whileHeld(m_turrRight);
    new JoystickButton(m_joystick, 5).whileHeld(m_liftClimber);
    new JoystickButton(m_joystick, 10).whileHeld(m_lowerClimber);
    new JoystickButton(m_joystick, 1).whenPressed(m_turrZero);
    new JoystickButton(m_operator, 5).whileHeld(m_turrPID);
    new JoystickButton(m_joystick, 7).whenPressed(m_grab);
    new JoystickButton(m_operator, 1).whileHeld(m_grab.alongWith(m_manConv));
    new JoystickButton(m_operator, 2).whileHeld(m_grab);   
    new JoystickButton(m_joystick, 13).whenPressed(m_extendGrabber.withTimeout(2));
    new JoystickButton(m_joystick, 14).whenPressed(m_retractGrabber.withTimeout(2));

    new JoystickButton(m_operator, 6).whenPressed(m_retractGrabber.withTimeout(2));
    new JoystickButton(m_operator, 8).whenPressed(m_retractGrabber.withTimeout(2));
    // new JoystickButton(m_operator, 5).whileHeld(autoAim);
    new JoystickButton(m_operator, 7).whileHeld(m_flySpool);
    new POVButton(m_operator, 0).whileHeld(m_liftClimber);
    new POVButton(m_operator, 180).whileHeld(m_lowerClimber);
    m_drive.setDefaultCommand(m_arcadeDrive);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An ExampleCommand will run in autonomous
  //   // return m_autoCommand;
  // }
}
