// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.AutoDriveForward;
import frc.robot.commands.AutoFire;
import frc.robot.commands.AutoHarvest;
import frc.robot.commands.AutoTurn;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Turret;
import java.util.List;
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
  // private final DoubleSupplier m_rotationAxis = () -> m_joystick.getRawAxis(0);
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
  private final InstantCommand m_resetNav = new InstantCommand(m_drive::resetNav, m_drive);
  private final ParallelCommandGroup m_autoHarvDrive =
      new ParallelCommandGroup(m_autoHarv, m_autoDrive);
  // private final SequentialCommandGroup m_auto = new SequentialCommandGroup(m_autoForward,
  // m_autoFire, m_autoHarvDrive);
  private final SequentialCommandGroup m_auto =
      new SequentialCommandGroup(m_autoFire, m_resetNav, m_autoTurn, m_autoHarvDrive);
  // private final SequentialCommandGroup m_auto = new SequentialCommandGroup(m_autoFire,
  // m_autoHarvDrive);

  private final ArcadeDrive m_arcadeDrive = new ArcadeDrive(m_drive, m_forwardAxis, m_rotationAxis);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  // Button -> Command Mapping
  private void configureButtonBindings() {

    new JoystickButton(m_joystick, 5).whenPressed(new InstantCommand(m_turret::zeroEncoder));
    new JoystickButton(m_joystick, 10).whenPressed(new InstantCommand(m_climber::zeroEncoder));

    new JoystickButton(m_operator, 6)
        .whenPressed(
            new StartEndCommand(m_grabber::retractGrabber, m_grabber::offGrabber, m_grabber)
                .withTimeout(2));
    new JoystickButton(m_operator, 8)
        .whenPressed(
            new StartEndCommand(m_grabber::extendGrabber, m_grabber::offGrabber, m_grabber)
                .withTimeout(2));
    new JoystickButton(m_operator, 1)
        .whileHeld(() -> m_turret.setTarget(m_opLeftHorizontal.getAsDouble() * 90));
    // new JoystickButton(m_operator, 5).whileHeld(autoAim);
    new JoystickButton(m_operator, 5).whileHeld(() -> m_fly.setRPM(3000), m_fly);
    new JoystickButton(m_operator, 7).whileHeld(() -> m_conveyor.manual(-0.5), m_conveyor);
    new POVButton(m_operator, 0)
        .whileHeld(
            new StartEndCommand(m_climber::liftClimber, m_climber::stop, m_climber)
                .withInterrupt(m_climber::getTop));
    new POVButton(m_operator, 180)
        .whileHeld(
            new StartEndCommand(m_climber::lowerClimber, m_climber::stop, m_climber)
                .withInterrupt(m_climber::getBottom));
    new POVButton(m_operator, 90)
        .whenPressed(
            new StartEndCommand(m_climber::brakeOn, m_climber::brakeOff, m_climber).withTimeout(5));
    new POVButton(m_operator, 270)
        .whenPressed(
            new StartEndCommand(m_climber::brakeRev, m_climber::brakeOff, m_climber)
                .withTimeout(5));
    m_drive.setDefaultCommand(
        new RunCommand(
            () -> {
              m_drive.arcadeDrive(m_forwardAxis.getAsDouble(), m_rotationAxis.getAsDouble());
            },
            m_drive));
    m_grabber.setDefaultCommand(
        new RunCommand(
            () -> {
              m_grabber.startGrab(m_opForward.getAsDouble());
            },
            m_grabber));
    m_conveyor.setDefaultCommand(
        new RunCommand(
            () -> {
              m_conveyor.manual(-m_opForward.getAsDouble());
            },
            m_conveyor));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return m_auto;
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.ksVoltsSecondsPerMeter,
                DriveConstants.kaVoltsSecondsSquaredPerMeter),
            DriveConstants.kKinematics,
            10);

    TrajectoryConfig config =
        new TrajectoryConfig(DriveConstants.kMaxSpeedMetersSecond, DriveConstants.kMaxAcceleration)
            .setKinematics(DriveConstants.kKinematics)
            .addConstraint(autoVoltageConstraint);

    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
              new Translation2d(1, 1), 
              new Translation2d(2, -1)
            ),
            // List.of(new Translation2d(1, 0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            m_drive::getPose,
            new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.ksVoltsSecondsPerMeter,
                DriveConstants.kaVoltsSecondsSquaredPerMeter),
            DriveConstants.kKinematics,
            m_drive::getWheelSpeeds,
            new PIDController(DriveConstants.kPDrive, 0, 0),
            new PIDController(DriveConstants.kPDrive, 0, 0),
            m_drive::tankDriveVolts,
            m_drive);
    m_drive.resetOdometry(exampleTrajectory.getInitialPose());
    return ramseteCommand.andThen(() -> m_drive.tankDriveVolts(0, 0));
  }
}
