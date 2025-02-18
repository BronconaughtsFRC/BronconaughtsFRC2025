// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

  public double deadbandDistance = 1.1;

  public VisionSubsystem() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    //read values periodically
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);

    SmartDashboard.putNumber("area ", area);
  }

  public Command turnTowardAprilTag(SwerveSubsystem swerve) {
    return run(()-> {
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
        swerve.setHeadingCorrection(false);

        swerve.setFrontLeft((deadbandDistance - area) * Constants.Vision.distanceKp, 0.0);
        swerve.setFrontRight((deadbandDistance - area) * Constants.Vision.distanceKp, 0.0);
        swerve.setBackLeft((deadbandDistance - area) * Constants.Vision.distanceKp, 0.0);
        swerve.setBackRight((deadbandDistance - area) * Constants.Vision.distanceKp, 0.0);
      }
    );
  }

  public Command moveAndAlignTowardAprilTag(SwerveSubsystem swerve) {
    return run(()-> {
        swerve.setHeadingCorrection(false);

        swerve.setFrontLeft((deadbandDistance - area) * Constants.Vision.distanceKp + (x * Constants.Vision.turnKp), 0.0);
        swerve.setFrontRight((deadbandDistance - area) * Constants.Vision.distanceKp + (-(x * Constants.Vision.turnKp)), 0.0);
        swerve.setBackLeft((deadbandDistance - area) * Constants.Vision.distanceKp + (x * Constants.Vision.turnKp), 0.0);
        swerve.setBackRight((deadbandDistance - area) * Constants.Vision.distanceKp + (-(x * Constants.Vision.turnKp)), 0.0);
      }
    );
  }

  public double getTx() {
    return x;
  }
}
