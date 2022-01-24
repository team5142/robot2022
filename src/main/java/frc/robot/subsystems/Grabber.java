package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

/**
 * Grabber Subsystem. Contains methods for pneumatic extension of
 * the grabber mechanism, operation of the grabber motor, and ultrasonic readings.
 * 
 * @author Spencer Greene
 */
public class Grabber extends SubsystemBase {
  private static Boolean isExtended = false;
  private static Boolean isClose = false;

  private final DoubleSolenoid m_sol = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, GrabberConstants.kSolForward, GrabberConstants.kSolReverse);
  private final WPI_VictorSPX m_spx = new WPI_VictorSPX(GrabberConstants.kGrabberSPX);
  private final AnalogInput m_ultrasound;

  /**
   * Creates a new Grabber Subsystem.
   */
  public Grabber() {
    m_ultrasound = new AnalogInput(Globals.kUltrasound);
    m_ultrasound.setAverageBits(2);
  }

  /** 
   * Extends the pneumatics to the grabber subsystem.
   */
  public void extendGrabber() {
    m_sol.set(Value.kForward);
    isExtended = true;
  }

  /** 
   * Retracts the pneumatics to the grabber subsystem.
   */
  public void retractGrabber() {
    m_sol.set(Value.kReverse);
    isExtended = false;
  }

  /** 
   * Toggles the grabber pneumatics off.
   */
  public void offGrabber() {
    m_sol.set(Value.kOff);
  }

  /**
   * Turns the grabber motor on.
   */
  public void startGrab() {
    m_spx.set(1);
  }

  /** 
   * Turns the grabber motor off.
   */
  public void stopGrab() {
    m_spx.stopMotor();
  }

  /**
   * Reads the ultrasound sensor.
   * @return the reading from the ultrasound. 
   */
  public int readUltrasound() {
    return m_ultrasound.getValue();
  }

  @Override
  public void periodic() {
    isClose = (readUltrasound() < 1600) ? true : false;
    SmartDashboard.putBoolean("Grabber State", isExtended);
    SmartDashboard.putBoolean("Range State", isClose);
    if(isExtended && isClose) {
      retractGrabber();
    }
  }
}
