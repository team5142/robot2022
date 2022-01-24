// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ExtendGrabber;
import frc.robot.commands.Grab;
import frc.robot.commands.RetractGrabber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Grabber;
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
  private final DoubleSupplier m_forwardAxis = () -> m_joystick.getRawAxis(1);
  private final DoubleSupplier m_rotationAxis = () -> m_joystick.getRawAxis(0);

  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drive = new Drivetrain();
  private final Grabber m_grabber = new Grabber();

  private final ExtendGrabber m_extendGrabber = new ExtendGrabber(m_grabber);
  private final RetractGrabber m_retractGrabber = new RetractGrabber(m_grabber);
  private final Grab m_grab = new Grab(m_grabber);

  private final ArcadeDrive m_arcadeDrive =
      new ArcadeDrive(m_drive, m_grabber, m_forwardAxis, m_rotationAxis);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  // Button -> Command Mapping
  private void configureButtonBindings() {
    new JoystickButton(m_joystick, 1).whenHeld(m_grab);
    new JoystickButton(m_joystick, 3).whenPressed(m_extendGrabber.withTimeout(2));
    new JoystickButton(m_joystick, 4).whenPressed(m_retractGrabber.withTimeout(2));

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
