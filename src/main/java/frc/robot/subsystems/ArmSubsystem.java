// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.mathExtras;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  PWMSparkMax armMotor = new PWMSparkMax(Constants.Arm.armMotorID);

  PIDController pid = new PIDController(Constants.Arm.kp, Constants.Arm.ki, Constants.Arm.kd);

  public double angleSetpoint, angleCurrent = 0.0;

  public ArmSubsystem() {
    angleSetpoint = 0.0;
    angleCurrent = 0.0;
  }

  public void setSetpoint(double setpoint, double linearSlideHight) {
    if (!(linearSlideHight > Constants.Arm.minSlideHightWhenMoving)) {
      angleSetpoint = mathExtras.codeStop(setpoint, Constants.Arm.minAngle, Constants.Arm.maxAngle);
    }
  }

  public void setToReefGrab(double linearSlideHight) {
    if (!(linearSlideHight > Constants.Arm.minSlideHightWhenMoving)) {
      angleSetpoint = mathExtras.codeStop(Constants.Arm.angleToGrabOffReef, Constants.Arm.minAngle, Constants.Arm.maxAngle);
    }
  }

  public void setToFloorGrab(double linearSlideHight) {
    if (!(linearSlideHight > Constants.Arm.minSlideHightWhenMoving)) {
      angleSetpoint = mathExtras.codeStop(0, Constants.Arm.minAngle, Constants.Arm.maxAngle);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    angleCurrent = armMotor.get() * Constants.Arm.encoderToAngleCoefficent;
    armMotor.set(pid.calculate(angleCurrent, angleSetpoint));
  }

  public double getCurrentAngle() {
    return angleCurrent;
  }
  public double getEncoderValue() {
    return armMotor.get();
  }
}
