// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.util.Units;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class DriveConstants {
    public static int kLeftMaster = 4;
    public static int kLeftSlave = 5;
    public static int kRightMaster = 2;
    public static int kRightSlave = 3;
    public static int kLeftEncoder = 7;
    public static int kRightEncoder = 6;
  }

  public static class GrabberConstants {
    public static int kSolForward = 0;
    public static int kSolReverse = 1;
    public static int kGrabberSPX = 12;
  }

  public static class ConveyorConstants {
    public static int kConveyorSRX = 8;
    public static int kPhotoBase = 1;
    public static int kPhotoMid = 2;
    public static int kPhotoTop = 3;
    public static int kCounter = 4;
  }

  public static class FlywheelConstants {
    public static int kFlyLeft = 40;
    public static int kFlyRight = 41;
    public static double kP = 0.0004;
    public static double kI = 0;
    public static double kD = 0.008;
    public static double kIz = 0;
    public static double kFF = 0.000195;
    public static double kMaxOut = 1;
    public static double kMinOut = -1;
  }

  public static class TurretConstants {
    public static int kTurret = 20;
    public static double kP = 0.03;
    public static double kI = 0;
    public static double kD = 0.2;
    public static double kIz = 0;
    public static double kFF = 0.00015;
    public static double kMaxOut = 0.3;
    public static double kMinOut = -0.3;
    public static double kCameraHeight = 0.55;
    public static double kCameraPitch = Units.degreesToRadians(4);
    public static double kGoalHeight = 0.89;
    public static double kGoalDistance = 3;
  }

  public static class ClimberConstants {
    public static int kClimberMasterSRX = 10;
    public static int kClimberFollowerSRX = 11;
    public static int kEncoder = 9;
  }

  public static class Globals {
    public static int kUltrasound = 1;
  }
}
