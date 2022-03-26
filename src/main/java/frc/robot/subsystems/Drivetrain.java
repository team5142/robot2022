package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

/**
 * Drivetrain Subsystem. Instantiates the drive motors, the encoders, various sensors and
 * navigational units, and simulation techniques.
 *
 * @author Spencer Greene
 */
public class Drivetrain extends SubsystemBase {
  private final WPI_TalonSRX m_rightMaster, m_rightSlave, m_leftMaster, m_leftSlave;

  private final DifferentialDrive m_drive;
  private final AHRS m_nav;
  public final DifferentialDriveOdometry m_odometry;
  private boolean isFlipped = false;
  // private boolean quickTurn = false;

  /** Crreates a new Drivetrain Subsystem. */
  public Drivetrain() {
    // Construct motors
    m_rightMaster = new WPI_TalonSRX(DriveConstants.kRightMaster);
    m_rightSlave = new WPI_TalonSRX(DriveConstants.kRightSlave);
    m_leftMaster = new WPI_TalonSRX(DriveConstants.kLeftMaster);
    m_leftSlave = new WPI_TalonSRX(DriveConstants.kLeftSlave);

    // Clear config
    m_rightMaster.configFactoryDefault();
    m_rightSlave.configFactoryDefault();
    m_leftMaster.configFactoryDefault();
    m_leftMaster.configFactoryDefault();

    // Drivebase configuration: master/follower & right-side inversion
    m_rightSlave.follow(m_rightMaster);
    m_leftSlave.follow(m_leftMaster);
    m_rightMaster.setInverted(InvertType.InvertMotorOutput);
    m_rightSlave.setInverted(InvertType.FollowMaster);
    m_leftMaster.setInverted(InvertType.None);
    m_leftSlave.setInverted(InvertType.FollowMaster);

    // Neutral Mode
    m_rightMaster.setNeutralMode(NeutralMode.Brake);
    m_leftMaster.setNeutralMode(NeutralMode.Brake);

    // Encoder selection & configuration
    m_rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    m_leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    // Construct a drivetrain;
    m_drive = new DifferentialDrive(m_leftMaster, m_rightMaster);

    // Odometry Management
    m_nav = new AHRS(SerialPort.Port.kMXP);
    m_nav.reset();
    m_odometry = new DifferentialDriveOdometry(m_nav.getRotation2d());
  }

  /**
   * Method for driving the robot teleoperated in an arcade manner.
   *
   * @param forward the supplied speed along the x axis.
   * @param rotation the supplied speed along the z axis.
   */
  public void arcadeDrive(double forward, double rotation) {
    if (isFlipped) {
      m_drive.arcadeDrive(forward, rotation);
    } else {
      m_drive.arcadeDrive(-forward, rotation);
    }
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMaster.setVoltage(leftVolts);
    m_rightMaster.setVoltage(rightVolts);
    m_drive.feed();
  }

  public void resetNav() {
    m_nav.reset();
  }

  public void resetEncoders() {
    m_leftMaster.setSelectedSensorPosition(0);
    m_rightMaster.setSelectedSensorPosition(0);
  }

  public void toggleDriveDirction() {
    isFlipped = !isFlipped;
  }

  public double getLeftPos() {
    return m_leftMaster.getSensorCollection().getQuadraturePosition();
  }

  public double getRightPos() {
    return m_rightMaster.getSensorCollection().getQuadraturePosition();
  }

  public double getHeading() {
    return m_nav.getAngle();
  }

  public double getTurnRate() {
    return -m_nav.getRate();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        nativeUnitsToDistanceMeters(m_leftMaster.getSensorCollection().getQuadratureVelocity()),
        nativeUnitsToDistanceMeters(m_rightMaster.getSensorCollection().getQuadratureVelocity()));
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_nav.getRotation2d());
  }

  /** UTILITY FUNCTIONS * */
  private int distanceToNativeUnits(double positionMeters) {
    double wheelRotations = positionMeters / (2 * Math.PI * Units.inchesToMeters(3));
    double motorRotations = wheelRotations * 10.71;
    int sensorCounts = (int) (motorRotations * 4096);
    return sensorCounts;
  }

  private int velocityToNativeUnits(double velocityMetersPerSecond) {
    double wheelRotationsPerSecond =
        velocityMetersPerSecond / (2 * Math.PI * Units.inchesToMeters(3));
    double motorRotationsPerSecond = wheelRotationsPerSecond * 10.71;
    double motorRotationsPer100ms = motorRotationsPerSecond / 10;
    int sensorCountsPer100ms = (int) (motorRotationsPer100ms * 4096);
    return sensorCountsPer100ms;
  }

  private double nativeUnitsToDistanceMeters(double sensorCounts) {
    double motorRotations = (double) sensorCounts / 4096;
    double wheelRotations = motorRotations / 10.71;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(3));
    return positionMeters;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(
        m_nav.getRotation2d(),
        nativeUnitsToDistanceMeters(getLeftPos()),
        nativeUnitsToDistanceMeters(getRightPos()));
    SmartDashboard.putNumber("LeftEnc", getLeftPos());
    SmartDashboard.putNumber("RightEnc", getRightPos());
    SmartDashboard.putNumber("Match Time", Timer.getMatchTime());
  }
}
