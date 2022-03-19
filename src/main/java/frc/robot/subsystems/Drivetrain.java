package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.kauailabs.navx.frc.AHRS;

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
  private final WPI_TalonSRX m_rightMaster = new WPI_TalonSRX(DriveConstants.kRightMaster);
  private final WPI_VictorSPX m_rightSlave = new WPI_VictorSPX(DriveConstants.kRightSlave);
  private final WPI_TalonSRX m_leftMaster = new WPI_TalonSRX(DriveConstants.kLeftMaster);
  private final WPI_VictorSPX m_leftSlave = new WPI_VictorSPX(DriveConstants.kLeftSlave);
  private final WPI_CANCoder m_rightEncoder = new WPI_CANCoder(DriveConstants.kRightEncoder);
  private final WPI_CANCoder m_leftEncoder = new WPI_CANCoder(DriveConstants.kLeftEncoder);
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMaster, m_rightMaster);
  private final AHRS m_nav = new AHRS(SerialPort.Port.kMXP);
  private boolean isFlipped = false;
  public double kP = 0;
  //private boolean quickTurn = false;

  /** Crreates a new Drivetrain Subsystem. */
  public Drivetrain() {
    m_rightMaster.configFactoryDefault();
    m_rightSlave.configFactoryDefault();
    m_leftMaster.configFactoryDefault();
    m_leftMaster.configFactoryDefault();

    m_rightSlave.follow(m_rightMaster);
    m_leftSlave.follow(m_leftMaster);

    m_rightMaster.setInverted(InvertType.InvertMotorOutput);
    m_rightSlave.setInverted(InvertType.FollowMaster);
    m_leftMaster.setInverted(InvertType.None);
    m_leftSlave.setInverted(InvertType.FollowMaster);
    m_rightMaster.setNeutralMode(NeutralMode.Coast);
    m_leftMaster.setNeutralMode(NeutralMode.Coast);
    m_rightEncoder.configSensorDirection(true);

    m_nav.reset();
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
    } else{
      m_drive.arcadeDrive(-forward, rotation);
    }
  }

  public void resetNav() {
    m_nav.reset();
  }

  public void toggleDriveDirction() {
    isFlipped = !isFlipped;
  }

  public double getLeftEncoder() {
    return m_leftEncoder.getPosition();
  }

  public double getRightEncoder() {
    return m_rightEncoder.getPosition();
  }

  public void zeroEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public double getHeading() {
    return m_nav.getAngle();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("LeftEnc", getLeftEncoder());
    SmartDashboard.putNumber("RightEnc", getRightEncoder());
    SmartDashboard.putNumber("Match Time", Timer.getMatchTime());
    SmartDashboard.putNumber("kP", kP);
  }
}
