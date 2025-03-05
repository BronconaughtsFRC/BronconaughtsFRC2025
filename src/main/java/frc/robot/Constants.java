// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    public static final double deadband = 0.2;
  }

  public static class Shooter {
    public static final int leftMotorID = 13;
    public static final int rightMotorID = 14;

    public static final int iterationsPerSecond = 20;

    public static final double secondsToShoot = 1.0;

    public static final double speedToPickup = -0.2;
  }

  public static class Arm {
    public static final int armMotorID = 15;

    public static final double kp = 0.04;
    public static final double ki = 0.0;
    public static final double kd = 0.0;

    public static final double startingAngle = 180;

    public static final double angleToGrabOffReef = 45;

    public static final double maxSetpointWhileDown = 28;

    public static final double minSlideHightWhenMoving = Units.inchesToMeters(50);

    public static final double encoderToAngleCoefficent = 12.43583;

    public static final double maxAngle = 180.0;
    public static final double minAngle = -10.0;

    public static final double maxSpeed = 0.3;
  }

  public static class LinearSlide {
    public static final int slideMotorID = 16;
    
    public static final double kp = 15.0; //0.6
    public static final double ki = 0.0;
    public static final double kd = 0.0;

    public static final double slideRampSpeed = 0.01;

    public static final double maxArmAngleWhenMoving = 28;

    public static final double slideHightFromFloor = Units.inchesToMeters(10.25);
    public static final double startingArmHight = Units.inchesToMeters(28);

    public static final double encoderToMetersCoefficent = (1.475 / 117);

    public static final double maxHight = 1.475 - slideHightFromFloor; //1.5194
    public static final double minHight = 0.0;
  }

  public static class Lift {
    public static final int liftMotorID = 17;

    public static final double maxHight = 1.0; //112
    public static final double minHight = -110.0;
  }

  public static class SwerveDrive {
    public static final double MAX_SPEED = 2.0; //YGASL uses 4.4196
    public static final double maxRotationalSpeed = 360; //Degrees Per Second

    public static final double driveMotorMaxVoltage = 12;
    public static final double turnMotorMaxVoltage = 12;

    public static final double MOI = 1.803;

    public static final double circleDegreesTolerence = 3; //Can not be zero.

    public static final double interationsPerSecond = 20;

    public static final double testEndVelocity = 1.5;
    public static final double testEndAngularVelocity = 4;
    public static final double testRampRate = 0.0025;
    public static final int testDelayTime = 60; //in iterations

    public static final double metersPerSecondWhileTesting = 1.503731;
    public static final double voltageWhileTestingMotion = 4.349653;

    public static final double degreesPerSecondWhileTesting = 4;
    public static final double voltageWhileTestingAngularRotation = 5.215897;

    public static double RoFtMBasedVolts(double targetMetersPerSecond) {
      return Math.pow(voltageWhileTestingMotion, (targetMetersPerSecond / metersPerSecondWhileTesting)); 
    } //Ratio of Force to Motion. Something I made up for my math.
    public static double RoFtABasedVolts(double targetDegreesPerSecond) {
      return Math.pow(voltageWhileTestingAngularRotation, (targetDegreesPerSecond / degreesPerSecondWhileTesting)); 
    } //Ratio of Force to Angular motion. Something I made up for my math.
  }

  public static class Vision {
    public static final double turnKp = 0.018;
    public static final double driveToDistanceKp = 0.2; //0.8 for carpet maybe. 
    //public static final double distanceKp = 5.735; //Kp to convert area to meters.
    public static final double slideMovementKp = 0.01;

    public static final double largestPossibleTagVisionArea = 1.6;
    public static final double largestPossibleBallVisionArea = 2;
    public static final double areaDeadband = 0.05;
    public static final double areaDedectionDeadband = 0.005; //Zero for no deadband.

    public static final double shooterShotKp = 0.05;
    public static final double maxBallVelocity = 2.0; //m/s of the ball.

    public static final double pickUpTargetSlideHightTolerence = 0.2;

    public static final double distanceWhileTestingShot = 5.0; //Meters, how far away you were.
    public static final double voltageWhileTestingShot = 4.0; //Voltage, to hit where you want to.

    public static final double maxMotorVoltage = 12;

    public static double RoFtDBasedVolts(double currentDistance) {
      return Math.pow(voltageWhileTestingShot, (currentDistance / distanceWhileTestingShot)); 
    } //Ratio of Force to Distance. Something I made up for my math.

    public static final double targetHightRampSpeed = 0.1; //In meters per second. Higher will make the shot less accurate, but shoot sooner if there is delay.

    public static final double maxTurnSpeedForTurnToPoint = 0.3;

    public static InterpolatingDoubleTreeMap distanceInterpolater = new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap verticleDistanceInterpolater = new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap algaeDistanceInterpolater = new InterpolatingDoubleTreeMap();


    public static double GetAprilTagDistance(double value) {
      distanceInterpolater.put(6.552953, Units.inchesToMeters(24));
      distanceInterpolater.put(1.682014, Units.inchesToMeters(54));
      distanceInterpolater.put(0.720026, Units.inchesToMeters(84));
      //distanceInterpolater.put(null, null);

      return distanceInterpolater.get(value);
    }
    public static double GetAprilTagVerticleDistance(double value) {
      verticleDistanceInterpolater.put(null, null);
      verticleDistanceInterpolater.put(null, null);
      verticleDistanceInterpolater.put(null, null);

      return verticleDistanceInterpolater.get(value);
    }
    public static double GetAlgaeDistance(double value) {
      algaeDistanceInterpolater.put(16.0, 15.0);
      algaeDistanceInterpolater.put(12.0, 24.0);
      algaeDistanceInterpolater.put(2.8, 40.0);
      algaeDistanceInterpolater.put(1.7, 60.0);

      return algaeDistanceInterpolater.get(value);
    }
  }

  public static class FieldConstants {
    public static final double hightOfBottemAlgaeInReef = 0.9188704;
  }

  public static class PhysicsConstants {
    public static final double gravitationalConstant = 9.81;
  }
}
