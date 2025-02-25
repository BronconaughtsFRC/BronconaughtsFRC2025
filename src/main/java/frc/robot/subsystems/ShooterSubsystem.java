// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  PWMSparkMax leftMotor = new PWMSparkMax(Constants.Shooter.leftMotorID);
  PWMSparkMax rightMotor  = new PWMSparkMax(Constants.Shooter.rightMotorID);

  public ShooterSubsystem() {
    
  }

  public void setLeftMotor(double speed) {
    leftMotor.set(speed);
  }

  public void setRightMotor(double speed) {
    rightMotor.set(speed);
  }

  public void setBothMotors(double speed) {
    leftMotor.set(speed);
    rightMotor.set(speed);
  }

  public void stopBothMotors() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public void setBothMotorsForTime(double speed, double time) {
    time = time * Constants.Shooter.iterationsPerSecond;

    leftMotor.set(0.0);
    rightMotor.set(0.0);

    while(time > 0) {
      time--;

      leftMotor.set(speed);
      rightMotor.set(speed);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
