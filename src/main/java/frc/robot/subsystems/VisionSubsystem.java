// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.mathExtras;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  public double x, y, area, oldX, oldY, oldArea, speedmultiplier;

  public int pipelineNumber = 0;

  NetworkTable table1, table2;
  NetworkTableEntry tx, ty, ta;

  int count = 0;

  double mountAngle, lensHeight, goalHeight = 0.0;

  public VisionSubsystem() {
    pipelineNumber = 1;
    count = 0;
  }

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("Limelight TX Pipeline 0 ", m_visionSubsystem.getTx(0));
    //SmartDashboard.putNumber("Limelight TX Pipeline 1 ", m_visionSubsystem.getTx(1));
    //SmartDashboard.putNumber("Limelight TX Pipeline 2 ", m_visionSubsystem.getTx(2));

    // This method will be called once per scheduler run
    table1 = NetworkTableInstance.getDefault().getTable("limelight-broncoa");
    table2 = NetworkTableInstance.getDefault().getTable("limelight-broncob");

    table1.getEntry("pipeline").setNumber(pipelineNumber);
    table2.getEntry("pipeline").setNumber(pipelineNumber);

    SmartDashboard.putNumber("Broncoa Area ", table1.getEntry("ta").getDouble(0.0));
    SmartDashboard.putNumber("Broncob Area ", table2.getEntry("ta").getDouble(0.0));

    if (table1.getEntry("ta").getDouble(0.0) >= table2.getEntry("ta").getDouble(0.0)) {
      tx = table1.getEntry("tx");
      ty = table1.getEntry("ty");
      ta = table1.getEntry("ta");

      y = ty.getDouble(0.0);
      x = tx.getDouble(0.0);
      area = ta.getDouble(0.0);

      speedmultiplier = 1.0;

      mountAngle = 27; //75.3;
      lensHeight = Units.inchesToMeters(9);
    } else {
      tx = table2.getEntry("tx");
      ty = table2.getEntry("ty");
      ta = table2.getEntry("ta");

      y = (ty.getDouble(0.0));
      x = (tx.getDouble(0.0));
      area = ta.getDouble(0.0);

      speedmultiplier = 1.0;

      mountAngle = 27; //75.3;
      lensHeight = Units.inchesToMeters(6.5);
    }

    if (area == 0.0) {
      if (!(oldArea == 0.0) && count < 5) {
        count++;
        area = oldArea;
        x = oldX;
        y = oldY;
      }
    }

    SmartDashboard.putNumber("area ", area);
    SmartDashboard.putNumber("tx ", x);

    if (area < Constants.Vision.areaDeadband) {
      area = 0;
    }

    if (area < Constants.Vision.areaDedectionDeadband) {
      x = 0.0;
      y = 0.0;
      area = 0.0;
    }

    SmartDashboard.putNumber("Ty ", y);
    SmartDashboard.putNumber("Limelight Distance ", getVerticleDistanceFromTarget(1));
  }

  public void setPipeline(int number) {
    pipelineNumber = number;
    
    table1 = NetworkTableInstance.getDefault().getTable("limelight-broncoa");
    table2 = NetworkTableInstance.getDefault().getTable("limelight-broncob");
    
    table1.getEntry("pipeline").setNumber(pipelineNumber);
    table2.getEntry("pipeline").setNumber(pipelineNumber);

    if (table1.getEntry("ta").getDouble(0.0) >= table2.getEntry("ta").getDouble(0.0)) {
      tx = table1.getEntry("tx");
      ty = table1.getEntry("ty");
      ta = table1.getEntry("ta");

      y = ty.getDouble(0.0);
      x = tx.getDouble(0.0);
      area = ta.getDouble(0.0);

      
      speedmultiplier = 1.0;

      mountAngle = 31.3;
      lensHeight = Units.inchesToMeters(9.0);

    } else {
      tx = table2.getEntry("tx");
      ty = table2.getEntry("ty");
      ta = table2.getEntry("ta");

      y = (ty.getDouble(0.0));
      x = (tx.getDouble(0.0));
      area = ta.getDouble(0.0);

      speedmultiplier = 1.0;

      mountAngle = 31.3;
      lensHeight = Units.inchesToMeters(6.5);
    }

    if (area == 0.0) {
      if (!(oldArea == 0.0) && count < 5) {
        count++;
        area = oldArea;
        x = oldX;
        y = oldY;
      }
    }

    if (area < Constants.Vision.areaDeadband) {
      area = 0;
    }

    if (area < Constants.Vision.areaDedectionDeadband) {
      x = 0.0;
      y = 0.0;
      area = 0.0;
    }

    oldArea = area;
    oldX = x;
    oldY = y;
  }

  public Pose2d getBargePosition() {
    var allience = DriverStation.getAlliance();
    if (allience.isPresent()) {
      if (allience.equals(Alliance.Red)) {
        return new Pose2d(8.75, 2, new Rotation2d(0));
      }
    }
    return new Pose2d(8.75, 6.2, new Rotation2d(0));
  }

  public double getTx(int pipeline) {
    setPipeline(pipeline);
    return x;
  }

  public double getDistanceFromTarget(int pipeline) {
    var allience = DriverStation.getAlliance();
    if (allience.isPresent()) {
      if (!allience.equals(Alliance.Blue)) {
        if (pipeline == 2) {
          setPipeline(3);
        }
      }
    } else {
      setPipeline(pipeline);
    }

    if (pipeline == 0) {
      goalHeight = 0.3048;
    } if (pipeline == 1) {
      goalHeight = 0.2032;
    } else {
      goalHeight = 1.7145;
    }

    /* 
    double angleToGoalDegrees = mountAngle + y;

    double angleToGoalRadians = Units.degreesToRadians(angleToGoalDegrees);

    SmartDashboard.putNumber("Distance Via Limelight ", (goalHeight - lensHeight) / Math.tan(angleToGoalRadians));
    return (goalHeight - lensHeight) / Math.tan(Math.abs(angleToGoalRadians));
    */

    if (pipeline == 1) {
      SmartDashboard.putBoolean("Getting Algae ", true);
      return Constants.Vision.GetAlgaeDistance(area);
    } else {
      SmartDashboard.putBoolean("Getting Algae ", false);
      return Constants.Vision.GetAprilTagDistance(area);
    }
  }

  public double getVerticleDistanceFromTarget(int pipeline) {
    var allience = DriverStation.getAlliance();
    if (allience.isPresent()) {
      if (!allience.equals(Alliance.Blue)) {
        if (pipeline == 2) {
          setPipeline(3);
        }
      }
    } else {
      setPipeline(pipeline);
    }

    if (pipeline == 0) {
      goalHeight = 0.3048;
    } if (pipeline == 1) {
      goalHeight = 0.2032;
    } else {
      goalHeight = 1.7145;
    }

    return Constants.Vision.GetAlgaeVerticleDistance(y);
  }

  public Command turnTowardAprilTag(SwerveSubsystem swerve) {
    return run(()-> {
        setPipeline(0);

        swerve.setHeadingCorrection(false);

        swerve.setFrontLeft((x * Constants.Vision.turnKp), -45.0);
        swerve.setFrontRight(-(x * Constants.Vision.turnKp), 45.0);
        swerve.setBackLeft((x * Constants.Vision.turnKp), 45.0);
        swerve.setBackRight(-(x * Constants.Vision.turnKp), -45.0);
      }
    );
  }

  public Command moveTowardAprilTag(SwerveSubsystem swerve) {
    return run(()-> {
        setPipeline(0);

        if (!(area == 0)) {
        swerve.setHeadingCorrection(false);

        swerve.setFrontLeft((getDistanceFromTarget(0) * speedmultiplier) * Constants.Vision.driveToDistanceKp, 0.0);
        swerve.setFrontRight((getDistanceFromTarget(0) * speedmultiplier) * Constants.Vision.driveToDistanceKp, 0.0);
        swerve.setBackLeft((getDistanceFromTarget(0) * speedmultiplier) * Constants.Vision.driveToDistanceKp, 0.0);
        swerve.setBackRight((getDistanceFromTarget(0) * speedmultiplier) * Constants.Vision.driveToDistanceKp, 0.0);
        }
      }
    );
  }

  public Command moveAndAlignTowardAprilTag(SwerveSubsystem swerve, double targetDistance) {
    return run(()-> {
        setPipeline(0);

        if (!(area == 0)) {
          swerve.setHeadingCorrection(false);

        swerve.setFrontLeft((((targetDistance - getDistanceFromTarget(0)) * speedmultiplier) * -Constants.Vision.driveToDistanceKp) + (x * Constants.Vision.turnKp), 0.0);
        swerve.setFrontRight((((targetDistance - getDistanceFromTarget(0)) * speedmultiplier) * -Constants.Vision.driveToDistanceKp) + (-(x * Constants.Vision.turnKp)), 0.0);
        swerve.setBackLeft((((targetDistance - getDistanceFromTarget(0)) * speedmultiplier) * -Constants.Vision.driveToDistanceKp) + (x * Constants.Vision.turnKp), 0.0);
        swerve.setBackRight((((targetDistance - getDistanceFromTarget(0)) * speedmultiplier) * -Constants.Vision.driveToDistanceKp) + (-(x * Constants.Vision.turnKp)), 0.0);
        }     
      }
    );
  }

  public Command strafeTowardAlgaeBall(SwerveSubsystem swerve) {
    return run(()-> {
        setPipeline(1);

        swerve.setHeadingCorrection(false);

        swerve.setFrontLeft((x * Constants.Vision.driveToDistanceKp), 90.0);
        swerve.setFrontRight((x * Constants.Vision.driveToDistanceKp), 90.0);
        swerve.setBackLeft((x * Constants.Vision.driveToDistanceKp), 90.0);
        swerve.setBackRight((x * Constants.Vision.driveToDistanceKp), 90.0);
      }
    );
  }

  public Command pickUpTarget(SwerveSubsystem swerve, LinearSlideSubsystem linearSlide, ArmSubsystem arm, ShooterSubsystem shooter) {
    return run(()-> {
        setPipeline(1);

        if (!(area == 0)) {
          //May be able to be improved by averaging the turn need for the angle, like in the AdvancedSwerveDriveCommand.
          swerve.setHeadingCorrection(false);

          //Might need speed multiplier
          swerve.setFrontLeft(((getDistanceFromTarget(1) * speedmultiplier) * -Constants.Vision.driveToDistanceKp) + (x * Constants.Vision.turnKp), 0.0);
          swerve.setFrontRight(((getDistanceFromTarget(1) * speedmultiplier) * -Constants.Vision.driveToDistanceKp) + (-(x * Constants.Vision.turnKp)), 0.0);
          swerve.setBackLeft(((getDistanceFromTarget(1) * speedmultiplier) * -Constants.Vision.driveToDistanceKp) + (x * Constants.Vision.turnKp), 0.0);
          swerve.setBackRight(((getDistanceFromTarget(1) * speedmultiplier) * -Constants.Vision.driveToDistanceKp) + (-(x * Constants.Vision.turnKp)), 0.0);

          linearSlide.setSetpoint(getVerticleDistanceFromTarget(1) - Constants.LinearSlide.slideHightFromFloor, arm.getCurrentAngle());

          if ((linearSlide.getSetpoint() + Constants.LinearSlide.slideHightFromFloor) >= (Constants.FieldConstants.hightOfBottemAlgaeInReef - Constants.Vision.pickUpTargetSlideHightTolerence)) {
            arm.setToReefGrab(linearSlide.getHight());
          } else {
            arm.setToFloorGrab(linearSlide.getHight());
          }

          shooter.setBothMotors(Constants.Shooter.speedToPickup);
        }
      }
    );
  }

  public Command turnTowardPoint(SwerveSubsystem swerve, double targetX, double targetY) {
    return run(()-> {
        Pose2d position = swerve.getPose();

        double positionX = position.getX();
        double positionY = position.getY();

        SmartDashboard.putNumber("Pos X ", positionX);
        SmartDashboard.putNumber("Pos Y ", positionY);

        double xSide = (targetX - positionX) / Units.inchesToMeters(690.875);
        double ySide = (targetY - positionY) / Units.inchesToMeters(317);

        SmartDashboard.putNumber("xSide ", xSide);
        SmartDashboard.putNumber("ySide ", ySide);

        double hypotenuse = Math.sqrt(Math.pow(xSide, 2) + Math.pow(ySide, 2)); //xSide + ySide

        double neededAngle = Math.acos(xSide/hypotenuse);

        double turningAngle = swerve.getCurrentAngle() + neededAngle;

        SmartDashboard.putNumber("Current Swerve Angle ", swerve.getCurrentAngle());
        SmartDashboard.putNumber("Needed Swerve Angle ", neededAngle);
        SmartDashboard.putNumber("Turning Angle ", turningAngle);

        swerve.setFrontLeft(mathExtras.codeStop((turningAngle * Constants.Vision.turnKp), 0, Constants.Vision.maxTurnSpeedForTurnToPoint), -45.0);
        swerve.setFrontRight(-mathExtras.codeStop((turningAngle * Constants.Vision.turnKp), 0, Constants.Vision.maxTurnSpeedForTurnToPoint), 45.0);
        swerve.setBackLeft(mathExtras.codeStop((turningAngle * Constants.Vision.turnKp), 0, Constants.Vision.maxTurnSpeedForTurnToPoint), 45.0);
        swerve.setBackRight(-mathExtras.codeStop((turningAngle * Constants.Vision.turnKp), 0, Constants.Vision.maxTurnSpeedForTurnToPoint), -45.0);
      }
    );
  }

  public double calculatePerfectShot(double targetHight, double range, double vi, ArmSubsystem arm, LinearSlideSubsystem slide) {
    double theta = (Math.pow(Math.sin((range * Constants.PhysicsConstants.gravitationalConstant) / Math.pow(vi, 2)), -1)) / 2;

    double speed = vi / Constants.Vision.maxBallVelocity;

    arm.setSetpoint(theta, slide.getHight());

    return speed;
  }

  public double calculateShot(double targetHight, double targetDistance, ArmSubsystem arm, LinearSlideSubsystem slide) {
    double realTargetHight = targetHight - Constants.LinearSlide.slideHightFromFloor;
    double t = 2 * (realTargetHight/ Constants.PhysicsConstants.gravitationalConstant);
    double vx = 2 * (realTargetHight/ t);
    double vi = vx / Math.sin(arm.getCurrentAngle());
    double vy = vi*Math.sin(arm.getCurrentAngle());

    double bigT = (vy - Math.sqrt(Math.pow(vy, 2) - ((2 * Constants.PhysicsConstants.gravitationalConstant) * realTargetHight))) / Constants.PhysicsConstants.gravitationalConstant;
    double distance = vx * bigT;

    while (distance < targetDistance) {
      realTargetHight = realTargetHight + Constants.Vision.targetHightRampSpeed;

      t = 2 * (realTargetHight/ Constants.PhysicsConstants.gravitationalConstant);
      vx = 2 * (realTargetHight/ t);
      vi = vx / Math.sin(arm.getCurrentAngle());
      vy = vi*Math.sin(arm.getCurrentAngle());
      bigT = (vy - Math.sqrt(Math.pow(vy, 2) - ((2 * Constants.PhysicsConstants.gravitationalConstant) * realTargetHight))) / Constants.PhysicsConstants.gravitationalConstant;
      distance = vx * bigT;
    }

    double speed = vi / Constants.Vision.maxBallVelocity;

    return speed;
  }

  public double calculateShotByVolts() {
    if (Constants.Vision.RoFtDBasedVolts(getDistanceFromTarget(2)) <= Constants.Vision.maxMotorVoltage) {
      return Constants.Vision.RoFtDBasedVolts(getDistanceFromTarget(2)) / Constants.Vision.maxMotorVoltage;
    } 
    else {
      return 1;
    }
  }
}
