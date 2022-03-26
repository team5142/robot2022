// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  // private final DoubleSolenoid m_sol = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ClimberConstants.kSolForward, ClimberConstants.kSolReverse);
  private WPI_TalonSRX m_master = new WPI_TalonSRX(ClimberConstants.kClimberMasterSRX);
  private WPI_TalonSRX m_follower = new WPI_TalonSRX(ClimberConstants.kClimberFollowerSRX);
  private WPI_CANCoder m_encoder = new WPI_CANCoder(ClimberConstants.kEncoder);

  public Climber() {
    m_follower.follow(m_master);
    m_master.setNeutralMode(NeutralMode.Brake);
    m_follower.setNeutralMode(NeutralMode.Brake);
    m_encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
  }

  public void liftClimber() {
    m_master.set(ControlMode.PercentOutput, -1);
  }

  public void lowerClimber() {
    m_master.set(ControlMode.PercentOutput, 1);
  }

  public void zeroEncoder() {
    m_encoder.setPosition(0);
  }

  public void brakeOn() {
    // m_sol.set(Value.kReverse);
  }

  public void brakeRev() {
    // m_sol.set(Value.kForward);
  }

  public void brakeOff() {
    // m_sol.set(Value.kOff);
  }

  public double getPos() {
    return m_encoder.getPosition();
  }

  public void stop() {
    m_master.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ClimberPos", getPos());
  }
}
