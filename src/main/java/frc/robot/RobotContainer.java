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
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Auto.AutoFire;
import frc.robot.commands.Auto.AutoHarvest;
import frc.robot.commands.Auto.AutoLimeTurr;
import frc.robot.commands.Auto.AutoTurn;
import frc.robot.commands.Auto.AutoTurrPID;
import frc.robot.commands.Auto.AutoTurrPID2;
import frc.robot.commands.Auto.NewAutoDrive;
import frc.robot.commands.Climber.BrakeClimber;
import frc.robot.commands.Climber.LiftClimber;
import frc.robot.commands.Climber.LowerClimber;
import frc.robot.commands.Climber.UnbrakeClimber;
import frc.robot.commands.Conveyor.ShootConveyor;
import frc.robot.commands.Conveyor.SpinConveyor;
import frc.robot.commands.Drivetrain.ArcadeDrive;
import frc.robot.commands.Flywheel.FlywheelSpool;
import frc.robot.commands.Flywheel.LowShot;
import frc.robot.commands.Grabber.ExtendGrabber;
import frc.robot.commands.Grabber.Grab;
import frc.robot.commands.Grabber.RetractGrabber;
import frc.robot.commands.Turret.LimelightTurret;
import frc.robot.commands.Turret.TurretPID;
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
  private final Joystick m_driverRight = new Joystick(0);
  private final Joystick m_driverLeft = new Joystick(1);
  private final Joystick m_operator = new Joystick(2);

  private final DoubleSupplier m_driverForward = () -> m_driverRight.getRawAxis(1);
  private final DoubleSupplier m_driverRotation = () -> m_driverLeft.getRawAxis(0);

  private final DoubleSupplier m_opForward = () -> m_operator.getRawAxis(3);
  private final DoubleSupplier m_opLeftHorizontal = () -> m_operator.getRawAxis(0);

  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drive = new Drivetrain();
  private final Conveyor m_conveyor = new Conveyor();
  private final Grabber m_grabber = new Grabber();
  private final Turret m_turret = new Turret();
  private final Climber m_climber = new Climber();
  private final Flywheel m_flywheel = new Flywheel();

  private final RetractGrabber m_retractGrabber = new RetractGrabber(m_grabber);
  private final ExtendGrabber m_extendGrabber = new ExtendGrabber(m_grabber);
  private final FlywheelSpool m_flySpool = new FlywheelSpool(m_flywheel);
  private final ShootConveyor m_shootConveyor = new ShootConveyor(m_conveyor, m_flywheel);
  private final LiftClimber m_liftClimber = new LiftClimber(m_climber);
  private final LowerClimber m_lowerClimber = new LowerClimber(m_climber);
  private final BrakeClimber m_brakeClimber = new BrakeClimber(m_climber);
  private final UnbrakeClimber m_unbrakeClimber = new UnbrakeClimber(m_climber);
  private final Grab m_grab = new Grab(m_grabber, m_opForward);
  private final SpinConveyor m_manualConveyor = new SpinConveyor(m_conveyor, m_opForward);
  private final TurretPID m_turretPID = new TurretPID(m_turret, m_opLeftHorizontal);
  private final LimelightTurret m_limeTurr = new LimelightTurret(m_turret);
  private final LowShot m_lowShot = new LowShot(m_flywheel, m_conveyor);
  private final AutoHarvest m_autoHarv = new AutoHarvest(m_grabber);
  private final AutoHarvest m_autoHarv2 = new AutoHarvest(m_grabber);
  private final AutoFire m_autoFire = new AutoFire(m_conveyor, m_flywheel);
  private final AutoFire m_autoFire2 = new AutoFire(m_conveyor, m_flywheel);
  private final NewAutoDrive m_autoRev = new NewAutoDrive(m_drive, -0.5, 0, "AutoRev");
  private final NewAutoDrive m_autoFwd = new NewAutoDrive(m_drive, 0.5, 0, "AutoFwd");
  private final NewAutoDrive m_autoRev2 = new NewAutoDrive(m_drive, -0.5, 0, "AutoRev2");
  private final NewAutoDrive m_autoFwd2 = new NewAutoDrive(m_drive, 0.5, 0, "AutoFwd");
  private final AutoLimeTurr m_autoLimeTurr = new AutoLimeTurr(m_turret);
  private final AutoTurrPID m_autoTurrPID = new AutoTurrPID(m_turret);
  private final AutoLimeTurr m_autoLimeTurr2 = new AutoLimeTurr(m_turret);
  private final AutoTurrPID2 m_autoTurrPID2 = new AutoTurrPID2(m_turret);
  private final AutoTurn m_autoTurn = new AutoTurn(m_drive);
  // private final BumperOppose m_opposeIntake = new BumperOppose(m_conveyor, m_opForward);
  // private final BumperSpool m_spoolBumper = new BumperSpool(m_conveyor);

  private final ArcadeDrive m_arcadeDrive =
      new ArcadeDrive(m_drive, m_driverForward, m_driverRotation);


  //   private final AutoDrive m_autoDrive = new AutoDrive(m_drive);
  //   private final AutoDriveForward m_autoForward = new AutoDriveForward(m_drive);
  //   private final AutoFire m_autoFire = new AutoFire(m_conveyor, m_fly);
  //   private final AutoHarvest m_autoHarv = new AutoHarvest(m_grabber);
  //   private final AutoTurn m_autoTurn = new AutoTurn(m_drive);
  //   private final InstantCommand m_resetNav = new InstantCommand(m_drive::zeroHeading, m_drive);
    private final ParallelDeadlineGroup m_autoHarvDrive =
        new ParallelDeadlineGroup(m_autoRev, m_autoHarv);
    private final ParallelDeadlineGroup m_autoHarvDrive2 =
        new ParallelDeadlineGroup(m_autoRev2, m_autoHarv2);
    private final ParallelRaceGroup m_trackFire = new ParallelRaceGroup(m_autoLimeTurr, m_autoFire);
    private final ParallelRaceGroup m_trackFire2 = new ParallelRaceGroup(m_autoLimeTurr2, m_autoFire2);
    private final SequentialCommandGroup m_autoMain = new SequentialCommandGroup(m_autoHarvDrive, m_autoTurrPID, m_autoFwd,  m_trackFire);
    // private final SequentialCommandGroup m_threeBall = new SequentialCommandGroup(m_autoHarvDrive, m_autoTurrPID, m_autoFwd, m_trackFire, m_autoTurn, m_autoHarvDrive2, m_autoFwd2, m_autoTurrPID2, m_trackFire2);
  //   private final SequentialCommandGroup m_auto =
  //       new SequentialCommandGroup(m_autoFire, m_resetNav, m_autoTurn, m_autoHarvDrive);
  //   // private final SequentialCommandGroup m_auto = new SequentialCommandGroup(m_autoFire,
  //   // m_autoHarvDrive);

  //   private final ArcadeDrive m_arcadeDrive = new ArcadeDrive(m_drive, m_forwardAxis,
  // m_rotationAxis);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  // Button -> Command Mapping
  private void configureButtonBindings() {

    new JoystickButton(m_driverRight, 2).whenPressed(new InstantCommand(m_turret::zeroEncoder));
    new JoystickButton(m_driverRight, 1).whenPressed(new InstantCommand(m_climber::zeroEncoder));

    new JoystickButton(m_operator, 6).whenPressed(m_retractGrabber.withTimeout(2));
    new JoystickButton(m_operator, 8).whenPressed(m_extendGrabber.withTimeout(2));
    new JoystickButton(m_operator, 5).whileHeld(m_flySpool);
    new JoystickButton(m_operator, 7).whileHeld(m_shootConveyor);
    new POVButton(m_operator, 0).whileHeld(m_liftClimber);
    new POVButton(m_operator, 180).whileHeld(m_lowerClimber);
    new POVButton(m_operator, 90).whenPressed(m_brakeClimber.withTimeout(2));
    new POVButton(m_operator, 270).whenPressed(m_unbrakeClimber.withTimeout(2));
    new JoystickButton(m_operator, 2).whileHeld(m_limeTurr);
    new JoystickButton(m_operator, 3).whileHeld(m_lowShot);

    m_drive.setDefaultCommand(m_arcadeDrive);
    m_grabber.setDefaultCommand(m_grab);
    m_conveyor.setDefaultCommand(m_manualConveyor);
    m_turret.setDefaultCommand(m_turretPID);
    // m_conveyor.setDefaultCommand(m_opposeIntake);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoMain;
    // var autoVoltageConstraint =
    //     new DifferentialDriveVoltageConstraint(
    //         new SimpleMotorFeedforward(
    //             DriveConstants.ksVolts,
    //             DriveConstants.kvVoltsSecondsPerMeter,
    //             DriveConstants.kaVoltsSecondsSquaredPerMeter),
    //         DriveConstants.kKinematics,
    //         10);

    // TrajectoryConfig config =
    //     new TrajectoryConfig(DriveConstants.kMaxSpeedMetersSecond, DriveConstants.kMaxAcceleration)
    //         .setKinematics(DriveConstants.kKinematics)
    //         .addConstraint(autoVoltageConstraint);

    // Trajectory exampleTrajectory =
    //     TrajectoryGenerator.generateTrajectory(
    //         // Start at the origin facing the +X direction
    //         new Pose2d(0, 0, new Rotation2d(0)),
    //         // Pass through these two interior waypoints, making an 's' curve path
    //         List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //         // List.of(new Translation2d(1, 0)),
    //         // End 3 meters straight ahead of where we started, facing forward
    //         new Pose2d(3, 0, new Rotation2d(0)),
    //         // Pass config
    //         config);

    // RamseteCommand ramseteCommand =
    //     new RamseteCommand(
    //         exampleTrajectory,
    //         m_drive::getPose,
    //         new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
    //         new SimpleMotorFeedforward(
    //             DriveConstants.ksVolts,
    //             DriveConstants.kvVoltsSecondsPerMeter,
    //             DriveConstants.kaVoltsSecondsSquaredPerMeter),
    //         DriveConstants.kKinematics,
    //         m_drive::getWheelSpeeds,
    //         new PIDController(DriveConstants.kPDrive, 0, 0),
    //         new PIDController(DriveConstants.kPDrive, 0, 0),
    //         m_drive::tankDriveVolts,
    //         m_drive);
    // m_drive.resetOdometry(exampleTrajectory.getInitialPose());
    // return ramseteCommand.andThen(() -> m_drive.tankDriveVolts(0, 0));
  }
}
