package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
  private final WPI_TalonSRX m_rightMaster, m_leftMaster;
  private final WPI_VictorSPX m_rightSlave, m_leftSlave;

  private final DifferentialDrive m_drive;
  private final AHRS m_nav;
  private final Field2d m_field = new Field2d();
  private final DifferentialDriveOdometry m_odometry;

  /** Crreates a new Drivetrain Subsystem. */
  public Drivetrain() {
    // Construct motors
    m_rightMaster = new WPI_TalonSRX(DriveConstants.kRightMaster);
    m_rightSlave = new WPI_VictorSPX(DriveConstants.kRightSlave);
    m_leftMaster = new WPI_TalonSRX(DriveConstants.kLeftMaster);
    m_leftSlave = new WPI_VictorSPX(DriveConstants.kLeftSlave);

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

    m_nav = new AHRS(SerialPort.Port.kMXP);
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_nav.getRotation2d());
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        nativeUnitsToDistanceMeters(m_leftMaster.getSelectedSensorVelocity()),
        nativeUnitsToDistanceMeters(m_rightMaster.getSelectedSensorVelocity()));
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_nav.getRotation2d());
  }

  public void arcadeDrive(double forward, double rotation) {
    m_drive.arcadeDrive(forward, rotation);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMaster.setVoltage(leftVolts);
    m_rightMaster.setVoltage(rightVolts);
    m_drive.feed();
  }

  public void resetEncoders() {
    m_leftMaster.setSelectedSensorPosition(0);
    m_rightMaster.setSelectedSensorPosition(0);
  }

  public double getAverageEncoderDistance() {
    return (getLeftDistance() + getRightDistance()) / 2.0;
  }

  public double getLeftDistance() {
    return nativeUnitsToDistanceMeters(m_leftMaster.getSelectedSensorPosition());
  }

  public double getRightDistance() {
    return nativeUnitsToDistanceMeters(m_rightMaster.getSelectedSensorPosition());
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    m_nav.reset();
  }

  public double getHeading() {
    return m_nav.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return -m_nav.getRate();
  }

  private double nativeUnitsToDistanceMeters(double sensorCounts) {
    double motorRotations = (double) sensorCounts / 4096;
    double wheelRotations = motorRotations / 10.71;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(6));
    return positionMeters;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(
        m_nav.getRotation2d(),
        nativeUnitsToDistanceMeters(getLeftDistance()),
        nativeUnitsToDistanceMeters(getRightDistance()));
    m_field.setRobotPose(getPose());
    SmartDashboard.putData(m_field);
  }
}
