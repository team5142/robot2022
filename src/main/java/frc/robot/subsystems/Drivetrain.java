package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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
  }

  /**
   * Method for driving the robot teleoperated in an arcade manner.
   *
   * @param forward the supplied speed along the x axis.
   * @param rotation the supplied speed along the z axis.
   */
  public void arcadeDrive(double forward, double rotation) {
    m_drive.arcadeDrive(forward, rotation);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
