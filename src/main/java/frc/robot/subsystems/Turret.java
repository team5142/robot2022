package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * The Turret Class controls the rotational turret to shoot the balls.
 *
 * @author Spencer Greene & Gavin Popkin
 */
public class Turret extends SubsystemBase {
  private final CANSparkMax m_turret =
      new CANSparkMax(TurretConstants.kTurret, MotorType.kBrushless);
  private final SparkMaxPIDController m_pidController;
  private final RelativeEncoder m_encoder;
  public double desiredPos;
  private final double loopspeed = 0.01;
  private final PhotonCamera m_camera = new PhotonCamera("gloworm");

  /** Constructor for a turret. */
  public Turret() {
    m_turret.restoreFactoryDefaults();
    // m_turret.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    // m_turret.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    m_pidController = m_turret.getPIDController();
    m_encoder = m_turret.getEncoder();
    m_encoder.setPosition(0);
    m_pidController.setP(TurretConstants.kP);
    m_pidController.setI(TurretConstants.kI);
    m_pidController.setD(TurretConstants.kD);
    m_pidController.setIZone(TurretConstants.kIz);
    m_pidController.setFF(TurretConstants.kFF);
    m_pidController.setOutputRange(TurretConstants.kMinOut, TurretConstants.kMaxOut);
    m_camera.setLED(VisionLEDMode.kOff);
  }

  public double getOffset() {
    PhotonTrackedTarget target = m_camera.getLatestResult().getBestTarget();
    if (target == null) {
      return 0;
    }
    return target.getYaw() + 3;
  }

  public void ledOn() {
    m_camera.setLED(VisionLEDMode.kOn);
  }

  public void ledOff() {
    m_camera.setLED(VisionLEDMode.kOff);
  }

  public void ledBlink() {
    m_camera.setLED(VisionLEDMode.kBlink);
  }

  public void setTargetPos() {
    double offset = getOffset();
    double currPos = getDeg();
    desiredPos = currPos - offset;
  }

  /** Stop the turret motor. */
  public void stopMotor() {
    m_turret.stopMotor();
  }

  /** Manual method to turn the turret motor to the right at a quarter speed. */
  public void turnRight() {
    m_turret.set(0.25);
  }

  /** Manual method to turn the turret motor to the left at a quarter speed. */
  public void turnLeft() {
    m_turret.set(-0.25);
  }

  /** Method to zero the encoder's position. */
  public void zeroEncoder() {
    m_encoder.setPosition(0);
  }

  /**
   * Get's the turret's encoder position.
   *
   * @return the encoder's position in native units.
   */
  public double getPos() {
    return m_encoder.getPosition();
  }

  /**
   * Get's the turret's degrees.
   *
   * @return the encoder's position calculated in degrees.
   */
  public double getDeg() {
    return m_encoder.getPosition() * 1.93;
  }

  /**
   * Sets the position of the turret to a positional reference.
   *
   * @param[in] target the desired position of the turret, in degrees.
   */
  public void setTarget(double target) {
    desiredPos = target;
    // m_pidController.setReference(target / 1.93, CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Enc Pos", getPos());
    SmartDashboard.putNumber("Enc Deg", getDeg());
    double targetPos = 0;
    if (desiredPos < 10) {
      targetPos = 10;
    } else if (desiredPos > 170) {
      targetPos = 170;
    } else {
      targetPos = desiredPos;
    }
    m_pidController.setReference(targetPos / 1.93, ControlType.kPosition);
  }
}
