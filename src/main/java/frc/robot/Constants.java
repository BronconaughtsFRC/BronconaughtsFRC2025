// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Shooter {
    public static final int leftMotorID = 13;
    public static final int rightMotorID = 14;
  }

  public static class Arm {
    public static final int armMotorID = 15;

    public static final double kp = 0.02;
    public static final double ki = 0.0;
    public static final double kd = 0.0;

    public static final double encoderToAngleCoefficent = 0.0;

    public static final double maxAngle = 1.0;
    public static final double minAngle = 0.0;
  }

  public static class SwerveDrive {
    public static final double MAX_SPEED = 2.0; //YGASL uses 4.4196

    public static final double MOI = 1.803;
  }

  public static class Vision {
    public static final double turnKp = 0.01;
    public static final double distanceKp = 0.4; //0.8 for carpet maybe
  }
}
