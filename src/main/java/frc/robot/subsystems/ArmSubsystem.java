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

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  SparkMax armMotor = new SparkMax(Constants.Arm.armMotorID, MotorType.kBrushless);

  PIDController pid = new PIDController(Constants.Arm.kp, Constants.Arm.ki, Constants.Arm.kd);

  double angleSetpoint, angleCurrent = 0.0;

  double usedAngleCurrent = 0.0;

  public ArmSubsystem() {
    angleSetpoint = 0.0 + Constants.Arm.startingAngle;
    angleCurrent = 0.0;

    usedAngleCurrent = 0.0;
  }

  public void setSetpoint(double setpoint, double linearSlideHight) {
    if ((linearSlideHight > Constants.Arm.minSlideHightWhenMoving || setpoint <= Constants.Arm.maxSetpointWhileDown)) {
      angleSetpoint = mathExtras.codeStop(setpoint, Constants.Arm.minAngle, Constants.Arm.maxAngle);
    }
  }

  public void setToReefGrab(double linearSlideHight) {
    if ((linearSlideHight > Constants.Arm.minSlideHightWhenMoving || Constants.Arm.angleToGrabOffReef <= Constants.Arm.maxSetpointWhileDown)) {
      angleSetpoint = mathExtras.codeStop(Constants.Arm.angleToGrabOffReef, Constants.Arm.minAngle, Constants.Arm.maxAngle);
    }
  }

  public void setToFloorGrab(double linearSlideHight) {
    if ((linearSlideHight > Constants.Arm.minSlideHightWhenMoving || 0 <= Constants.Arm.maxSetpointWhileDown)) {
      angleSetpoint = mathExtras.codeStop(0, Constants.Arm.minAngle, Constants.Arm.maxAngle);
    }
  }

  public void rampSetpoint(double linearSlideHight, double rampRate) {
    if ((linearSlideHight > Constants.Arm.minSlideHightWhenMoving || angleSetpoint + rampRate <= Constants.Arm.maxSetpointWhileDown)) {
      angleSetpoint = mathExtras.codeStop(angleSetpoint + rampRate, Constants.Arm.minAngle, Constants.Arm.maxAngle);
    }
  }

  public void setMotor(double speed) {
    armMotor.set(speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Encoder ", getEncoderValue());

    // This method will be called once per scheduler run
    angleCurrent = armMotor.getEncoder().getPosition() * Constants.Arm.encoderToAngleCoefficent;
    double usedAngleCurrent = angleCurrent + Constants.Arm.startingAngle;
    armMotor.set(mathExtras.codeStop(pid.calculate(usedAngleCurrent, angleSetpoint), -Constants.Arm.maxSpeed, Constants.Arm.maxSpeed));

    SmartDashboard.putNumber("Arm Angle ", usedAngleCurrent);
    SmartDashboard.putNumber("Arm Setpoint ", angleSetpoint);
    SmartDashboard.putNumber("Arm Motor Output ", armMotor.get());
  }

  public double getCurrentAngle() {
    return usedAngleCurrent;
  }
  public double getEncoderValue() {
    return armMotor.getEncoder().getPosition();
  }
}
