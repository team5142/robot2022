// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Grabber extends SubsystemBase {
  private static Boolean isExtended = false;
  private static Boolean isClose = false;

  private DoubleSolenoid m_sol;
  private WPI_VictorSPX m_spx;
  private AnalogInput m_ultrasound;
  /** Creates a new Grabber. */
  public Grabber() {
    m_sol = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.GrabberConstants.kSolForward, Constants.GrabberConstants.kSolReverse);
    m_spx = new WPI_VictorSPX(Constants.GrabberConstants.kGrabberSPX);
    m_ultrasound = new AnalogInput(Constants.Globals.kUltrasound);
    m_ultrasound.setAverageBits(2);
  }

  public void extendGrabber() {
    m_sol.set(Value.kForward);
    isExtended = true;
  }

  public void retractGrabber() {
    m_sol.set(Value.kReverse);
    isExtended = false;
  }

  public void offGrabber() {
    m_sol.set(Value.kOff);
  }

  public void startGrab() {
    m_spx.set(1);
  }

  public void stopGrab() {
    m_spx.stopMotor();
  }

  public int readUltrasound() {
    return m_ultrasound.getValue();
  }

  @Override
  public void periodic() {
    isClose = (readUltrasound() < 1600) ? true : false;
    // if(readUltrasound() < 1600) {
    //   isClose = true;
    // } else {
    //   isClose = false;
    // }
    SmartDashboard.putBoolean("Grabber State", isExtended);
    SmartDashboard.putBoolean("Range State", isClose);
    if(isExtended && isClose) {
      retractGrabber();
    }
  }
}
