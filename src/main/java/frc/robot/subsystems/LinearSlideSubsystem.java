// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.mathExtras;
import frc.robot.Constants;

public class LinearSlideSubsystem extends SubsystemBase {
  /** Creates a new LinearSlideSubsystem. */
  SparkMax slideMotor = new SparkMax(Constants.LinearSlide.slideMotorID, MotorType.kBrushless);

  PIDController pid = new PIDController(Constants.LinearSlide.kp, Constants.LinearSlide.ki, Constants.LinearSlide.kd);

  double slideSetpoint, slideCurrent = 0.0;

  public LinearSlideSubsystem() {
    slideSetpoint = 0.0;
    slideCurrent = 0.0;
  }
  //was <
  public void setSetpoint(double setpoint, double armAngle) {
    if ((armAngle < Constants.LinearSlide.maxArmAngleWhenMoving) || (armAngle > 90 && setpoint > Constants.LinearSlide.startingArmHight)) {
      slideSetpoint = mathExtras.codeStop(setpoint, Constants.LinearSlide.minHight, Constants.LinearSlide.maxHight);
    }
  }
  public void increaseSetpoint(double armAngle) {
    if ((armAngle < Constants.LinearSlide.maxArmAngleWhenMoving) || (armAngle > 90 && slideSetpoint + Constants.LinearSlide.slideRampSpeed > Constants.LinearSlide.startingArmHight)) {
      slideSetpoint = mathExtras.codeStop(slideSetpoint + Constants.LinearSlide.slideRampSpeed, Constants.LinearSlide.minHight, Constants.LinearSlide.maxHight);
    }
  }
  public void decreaseSetpoint(double armAngle) {
    if ((armAngle < Constants.LinearSlide.maxArmAngleWhenMoving) || (armAngle > 90 && slideSetpoint - Constants.LinearSlide.slideRampSpeed > Constants.LinearSlide.startingArmHight)) {
      slideSetpoint = mathExtras.codeStop(slideSetpoint - Constants.LinearSlide.slideRampSpeed, Constants.LinearSlide.minHight, Constants.LinearSlide.maxHight);
    }
  }
  public void rampSetpoint(double armAngle, double ramp) {
    if ((armAngle < Constants.LinearSlide.maxArmAngleWhenMoving) || (armAngle > 90 && slideSetpoint + ramp > Constants.LinearSlide.startingArmHight)) {
      slideSetpoint = mathExtras.codeStop(slideSetpoint + ramp, Constants.LinearSlide.minHight, Constants.LinearSlide.maxHight);
    }
  }

  public void setMotors(double speed) {
    slideMotor.set(speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Slide Setpoint ", getSetpoint());

    // This method will be called once per scheduler run
    slideCurrent = slideMotor.getEncoder().getPosition() * Constants.LinearSlide.encoderToMetersCoefficent;
    SmartDashboard.putNumber("Slide Current  ", slideCurrent);
    SmartDashboard.putNumber("Slide Motor Output ", slideMotor.get());

    slideMotor.set(pid.calculate(slideCurrent, slideSetpoint));
  }

  public double getHight() {
    return (slideMotor.getEncoder().getPosition() * Constants.LinearSlide.encoderToMetersCoefficent) + Constants.LinearSlide.slideHightFromFloor;
  }

  public double getEncoderValue() {
    return slideMotor.getEncoder().getPosition();
  }

  public double getSetpoint() {
    return slideSetpoint;
  }
}
