// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public class DriveValues {
    public static int driveLeftID = 2;
    public static int followerLeft = 13;
    public static boolean driveLeftInverted = false;
    public static boolean followerLeftInverted = false;

    public static int driveRightID = 4;
    public static int followerRight = 14;
    public static boolean driveRightInverted = true;
    public static boolean followerRightInverted = true;

    public static double maxSpeed = 0.5;

    public class driveLeftPID {
      public static double kP = 0.45;
      public static double kI = 0;
      public static double kD = 0;
      public static double kF = 0.225;

      // Which controller to use, 1 = primary, 0 = aux (I know it's weird just go with
      // it)
      public static int leftMotorPIDController = 0;
    }

    public class driveRightPID {
      public static double kP = 0.45;
      public static double kI = 0;
      public static double kD = 0;
      public static double kF = 0;

      // Which controller to use, 1 = primary, 0 = aux (I know it's weird just go with
      // it)
      public static int rightMotorPIDController = 0;
    }

  }

  public class Shooter {
    public class Tilt {
      public static int tilt = 6;
      public static double kP = 0.01;
      public static double kI = 0;
      public static double kD = 0;
    }

    public class Propulsion {
      public static int shooterRight = 7;
      public static int shooterLeft = 8;

      public static boolean shooterRightInverted = false;
      public static boolean shooterLeftInverted = true;

      public static int feedBackSensor = 0;

      public static double kP = 0.2;
      public static double kI = 0;
      public static double kD = 0;
      public static double kF = 0.04;
    }

  }

  public class Conveyor {
    public static int conveyor = 3;
    public static int beaterBar = 11;
  }

  public class LimitSwitches {
    public static int topSwitch = 3;
    public static int middleSwitch = 2;
    public static int bottomSwitch = 1;
  }
}
