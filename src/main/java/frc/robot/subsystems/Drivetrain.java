package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Drivetrain Subsystem. Instantiates the drive motors, the encoders,
 * various sensors and navigational units, and simulation techniques. 
 * 
 * @author Spencer Greene
 */
public class Drivetrain extends SubsystemBase {
  private WPI_TalonSRX m_rightMaster;
  private WPI_TalonSRX m_rightSlave;
  private WPI_TalonSRX m_leftMaster;
  private WPI_VictorSPX m_leftSlave;
  private WPI_CANCoder m_rightEncoder;
  private WPI_CANCoder m_leftEncoder;

  // private Encoder m_leftEncoder;
  // private Encoder m_rightEncoder;
  // private EncoderSim m_leftEncoderSim;
  // private EncoderSim m_rightEncoderSim;
  private DifferentialDrive m_drive;
  // private DifferentialDrivetrainSim m_driveSim;
  // private Field2d m_field;
  // private DifferentialDriveOdometry m_odometry;
  // private AnalogGyro m_gyro;
  // private AnalogGyroSim m_gyroSim;

  /**
   * Crreates a new Drivetrain Subsystem.
   */
  public Drivetrain() {
    m_rightMaster = new WPI_TalonSRX(Constants.DriveConstants.kRightMaster);
    m_rightSlave = new WPI_TalonSRX(Constants.DriveConstants.kRightSlave);
    m_leftMaster = new WPI_TalonSRX(Constants.DriveConstants.kLeftMaster);
    m_leftSlave = new WPI_VictorSPX(Constants.DriveConstants.kLeftSlave);
    
    m_rightEncoder = new WPI_CANCoder(Constants.DriveConstants.kRightEncoder);
    m_leftEncoder = new WPI_CANCoder(Constants.DriveConstants.kLeftEncoder);
    // m_leftEncoder = new Encoder(0, 1);
    // m_rightEncoder = new Encoder(2, 3);
    // m_leftEncoderSim = new EncoderSim(m_leftEncoder);
    // m_rightEncoderSim = new EncoderSim(m_rightEncoder);
    // m_gyro = new AnalogGyro(0);
    // m_gyroSim = new AnalogGyroSim(m_gyro);

    // m_leftEncoder.setDistancePerPulse(2 * Math.PI * 3 / 4096);
    // m_rightEncoder.setDistancePerPulse(2 * Math.PI * 3 / 4096);
    // m_leftEncoder.reset();
    // m_rightEncoder.reset();

    m_rightMaster.setInverted(true);
    m_rightSlave.follow(m_rightMaster);
    m_rightSlave.setInverted(InvertType.FollowMaster);
    m_leftSlave.follow(m_leftMaster);
    m_drive = new DifferentialDrive(m_leftMaster, m_rightMaster);

    // m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

    // m_field = new Field2d();
    // SmartDashboard.putData("Field", m_field);

    // m_driveSim = DifferentialDrivetrainSim.createKitbotSim(
    //   KitbotMotor.kDualCIMPerSide,
    //   KitbotGearing.k10p71,
    //   KitbotWheelSize.kSixInch,
    //   null
    // );
  }

  /**
   * Method for driving the robot teleoperated in an arcade manner.
   * @param x the supplied speed along the x axis.
   * @param z the supplied speed along the z axis.
   */
  public void arcadeDrive(double x, double z) {
    m_drive.arcadeDrive(x, z);
  }

  // public void updateOdometry() {
  //   m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  // }

  // public Pose2d getPose() {
  //   return m_odometry.getPoseMeters();  
  // }

  // public void simulationPeriodic() {
  //   m_driveSim.setInputs(m_leftMaster.get() * RobotController.getInputVoltage(), m_rightMaster.get() * RobotController.getInputVoltage());
  //   m_driveSim.update(0.02);
  //   // m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
  //   // m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
  //   // m_rightEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
  //   // m_rightEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
  //   m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    // m_field.setRobotPose(m_odometry.getPoseMeters());
  }
}
