// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  public double x, y, area;

  public int pipelineNumber = 0;

  public VisionSubsystem() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    NetworkTable table1 = NetworkTableInstance.getDefault().getTable("limelight-bronco");
    NetworkTable table2 = NetworkTableInstance.getDefault().getTable("limelight-bronco2");
    table1.getEntry("pipeline").setNumber(pipelineNumber);
    table2.getEntry("pipeline").setNumber(pipelineNumber);

    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;

    if (table1.getEntry("ta").getDouble(0.0) >= table2.getEntry("ta").getDouble(0.0)) {
      tx = table1.getEntry("tx");
      ty = table1.getEntry("ty");
      ta = table1.getEntry("ta");
    } else {
      tx = table2.getEntry("tx");
      ty = table2.getEntry("ty");
      ta = table2.getEntry("ta");
    }

    //read values periodically
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);

    if (area < Constants.Vision.areaDeadband) {
      area = 0;
    }

    if (area < Constants.Vision.areaDedectionDeadband) {
      x = 0.0;
      y = 0.0;
      area = 0.0;
    }

    SmartDashboard.putNumber("area ", area);
  }

  public void setPipeline(int number) {
    pipelineNumber = number;
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

        swerve.setFrontLeft((Constants.Vision.largestPossibleVisionArea - area) * Constants.Vision.distanceKp, 0.0);
        swerve.setFrontRight((Constants.Vision.largestPossibleVisionArea - area) * Constants.Vision.distanceKp, 0.0);
        swerve.setBackLeft((Constants.Vision.largestPossibleVisionArea - area) * Constants.Vision.distanceKp, 0.0);
        swerve.setBackRight((Constants.Vision.largestPossibleVisionArea - area) * Constants.Vision.distanceKp, 0.0);
        }
      }
    );
  }

  public Command moveAndAlignTowardAprilTag(SwerveSubsystem swerve) {
    return run(()-> {
        setPipeline(0);

        if (!(area == 0)) {
          swerve.setHeadingCorrection(false);

        swerve.setFrontLeft((Constants.Vision.largestPossibleVisionArea - area) * Constants.Vision.driveToDistanceKp + (x * Constants.Vision.turnKp), 0.0);
        swerve.setFrontRight((Constants.Vision.largestPossibleVisionArea - area) * Constants.Vision.driveToDistanceKp + (-(x * Constants.Vision.turnKp)), 0.0);
        swerve.setBackLeft((Constants.Vision.largestPossibleVisionArea - area) * Constants.Vision.driveToDistanceKp + (x * Constants.Vision.turnKp), 0.0);
        swerve.setBackRight((Constants.Vision.largestPossibleVisionArea - area) * Constants.Vision.driveToDistanceKp + (-(x * Constants.Vision.turnKp)), 0.0);
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

          swerve.setFrontLeft((Constants.Vision.largestPossibleVisionArea - area) * Constants.Vision.driveToDistanceKp + (x * Constants.Vision.turnKp), 0.0);
          swerve.setFrontRight((Constants.Vision.largestPossibleVisionArea - area) * Constants.Vision.driveToDistanceKp + (-(x * Constants.Vision.turnKp)), 0.0);
          swerve.setBackLeft((Constants.Vision.largestPossibleVisionArea - area) * Constants.Vision.driveToDistanceKp + (x * Constants.Vision.turnKp), 0.0);
          swerve.setBackRight((Constants.Vision.largestPossibleVisionArea - area) * Constants.Vision.driveToDistanceKp + (-(x * Constants.Vision.turnKp)), 0.0);

          if (y > 0) {
            linearSlide.increaseSetpoint(arm.getCurrentAngle());
          } else {
            linearSlide.decreaseSetpoint(arm.getCurrentAngle());
          }

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
    return Math.pow(area, 2) * Constants.Vision.distanceKp; //Might not be squared
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
