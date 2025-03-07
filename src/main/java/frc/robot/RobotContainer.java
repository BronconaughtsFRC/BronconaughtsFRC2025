// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.mathExtras;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.LinearSlideSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();

  
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final LinearSlideSubsystem m_linearSlideSubsystem = new LinearSlideSubsystem();
  private final LiftSubsystem m_liftSubsystem = new LiftSubsystem();
  

  private final SendableChooser<Command> autoChooser;

  double turnSpeedX, turnSpeedY = 0.0;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Joystick driverJoystick =
      new Joystick(OperatorConstants.kDriverControllerPort);

  private final XboxController operatorController = new XboxController(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_visionSubsystem.turnTowardAprilTag(m_swerveSubsystem).addRequirements(m_swerveSubsystem);
    m_visionSubsystem.moveTowardAprilTag(m_swerveSubsystem).addRequirements(m_swerveSubsystem);
    m_visionSubsystem.moveAndAlignTowardAprilTag(m_swerveSubsystem, 0.0).addRequirements(m_swerveSubsystem);
    m_visionSubsystem.strafeTowardAlgaeBall(m_swerveSubsystem).addRequirements(m_swerveSubsystem);
    m_visionSubsystem.pickUpTarget(m_swerveSubsystem, m_linearSlideSubsystem, m_armSubsystem, m_shooterSubsystem).addRequirements(m_swerveSubsystem, m_linearSlideSubsystem, m_armSubsystem, m_shooterSubsystem);
    m_visionSubsystem.turnTowardPoint(m_swerveSubsystem, 0, 0).addRequirements(m_swerveSubsystem);

    m_swerveSubsystem.setDefaultCommand(m_swerveSubsystem.driveCommand(
      ()-> -mathExtras.deadband(driverJoystick.getY(), Constants.OperatorConstants.deadband),
      ()-> mathExtras.deadband(driverJoystick.getX(), Constants.OperatorConstants.deadband),
      ()-> -mathExtras.deadband(driverJoystick.getZ(), Constants.OperatorConstants.deadband)));
    //in pidf, d was 0.0
    
    m_linearSlideSubsystem.setDefaultCommand(new RunCommand(()->
      m_linearSlideSubsystem.rampSetpoint(m_armSubsystem.getCurrentAngle(), mathExtras.deadband(operatorController.getRightY(), Constants.OperatorConstants.deadband)  * -0.01), m_linearSlideSubsystem));
    m_armSubsystem.setDefaultCommand(new RunCommand(()->
      m_armSubsystem.rampSetpoint(m_linearSlideSubsystem.getHight(), mathExtras.deadband(operatorController.getLeftY(), Constants.OperatorConstants.deadband)  * -1.0), m_armSubsystem));
    

    Pose2d bargePosition = m_visionSubsystem.getBargePosition();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    NamedCommands.registerCommand("StrafeToAlgaeBall", m_visionSubsystem.strafeTowardAlgaeBall(m_swerveSubsystem));
    //Test if PickupTarget is inverted
    NamedCommands.registerCommand("PickUpTarget", m_visionSubsystem.pickUpTarget(m_swerveSubsystem, m_linearSlideSubsystem, m_armSubsystem, m_shooterSubsystem));
    NamedCommands.registerCommand("MoveToAprilTag", m_visionSubsystem.moveAndAlignTowardAprilTag(m_swerveSubsystem, 0.5));
    NamedCommands.registerCommand("Shoot", new InstantCommand(()-> m_shooterSubsystem.setBothMotorsForTime(
      m_visionSubsystem.calculateShot(
      2.262048,
      m_visionSubsystem.getDistanceFromTarget(2),
      m_armSubsystem,
      m_linearSlideSubsystem), Constants.Shooter.secondsToShoot),
      m_shooterSubsystem)); //Change to other shooter command if the others work better.

    //NamedCommands.registerCommand("AlignThenShoot", somethingIDK);

    configureBindings();
  }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    /* 
    SmartDashboard.putNumber("Joystick X ", mathExtras.deadband(driverJoystick.getX(), Constants.OperatorConstants.deadband));
    SmartDashboard.putNumber("Joystick Y ", mathExtras.deadband(driverJoystick.getY(), Constants.OperatorConstants.deadband));
    SmartDashboard.putNumber("Joystick Z ", mathExtras.deadband(driverJoystick.getZ(), Constants.OperatorConstants.deadband));
    */

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    if (driverJoystick.getPOV() == 1) {
      turnSpeedX = 1.0;
    } else if (driverJoystick.getPOV() == 3) {
      turnSpeedY = 1.0;
    } else if (driverJoystick.getPOV() == 5) { 
      turnSpeedX = -1.0;
    } else if (driverJoystick.getPOV() == 7) { 
      turnSpeedY = -1.0;
    }

    //new JoystickButton(driverJoystick, 16).whileTrue(m_visionSubsystem.moveAndAlignTowardAprilTag(m_swerveSubsystem));

    //new JoystickButton(driverJoystick, 15).whileTrue(m_visionSubsystem.turnTowardPoint(m_swerveSubsystem, 0.0, 2.0));
    //new JoystickButton(driverJoystick, 16).whileTrue(new RunCommand(()-> m_swerveSubsystem.setAll(1, 0), m_swerveSubsystem));

    //SmartDashboard.putNumber("Distance Limelight ", m_visionSubsystem.getDistanceFromTarget(0));

    //new JoystickButton(driverJoystick, 16).whileTrue(m_visionSubsystem.moveAndAlignTowardAprilTag(m_swerveSubsystem));
    
    new Trigger(()-> (operatorController.getRightTriggerAxis() > 0.2)).whileTrue(new RunCommand(()->m_shooterSubsystem.setBothMotors(0.9))).onFalse(new RunCommand(()->m_shooterSubsystem.setBothMotors(0.0)));
    new Trigger(()-> (operatorController.getLeftTriggerAxis() > 0.2)).whileTrue(new RunCommand(()->m_shooterSubsystem.setBothMotors(Constants.Shooter.speedToPickup))).onFalse(new RunCommand(()->m_shooterSubsystem.setBothMotors(0.0)));


    new JoystickButton(driverJoystick, 2).whileTrue(m_visionSubsystem.pickUpTarget(m_swerveSubsystem, m_linearSlideSubsystem, m_armSubsystem, m_shooterSubsystem));

    new JoystickButton(driverJoystick, 7).whileTrue(new RunCommand (()->m_armSubsystem.rampSetpoint(m_linearSlideSubsystem.getHight(), 0.5)));
    new JoystickButton(driverJoystick, 6).whileTrue(new RunCommand (()->m_armSubsystem.rampSetpoint(m_linearSlideSubsystem.getHight(), -0.5)));


    //new JoystickButton(driverJoystick, 7).onTrue(new InstantCommand (()->m_armSubsystem.setSetpoint(45, m_linearSlideSubsystem.getHight())));
    //new JoystickButton(driverJoystick, 6).onTrue(new InstantCommand (()->m_armSubsystem.setSetpoint(0, m_linearSlideSubsystem.getHight())));

    new JoystickButton(driverJoystick, 15).whileTrue(new RunCommand (()->m_linearSlideSubsystem.decreaseSetpoint(m_armSubsystem.getCurrentAngle())));
    new JoystickButton(driverJoystick, 16).whileTrue(new RunCommand (()->m_linearSlideSubsystem.increaseSetpoint(m_armSubsystem.getCurrentAngle())));
    
    new JoystickButton(driverJoystick, 8).whileTrue(new RunCommand (()->m_shooterSubsystem.setBothMotors(Constants.Shooter.speedToPickup))).onFalse(new RunCommand (()-> m_shooterSubsystem.setBothMotors(0)));
    new JoystickButton(driverJoystick, 9).whileTrue(new RunCommand (()->m_shooterSubsystem.setBothMotors(1.0))).onFalse(new RunCommand (()-> m_shooterSubsystem.setBothMotors(0)));
    

    new JoystickButton(driverJoystick, 11).whileTrue(new RunCommand (()->m_liftSubsystem.unsafeSpeed(0.3))).onFalse(new RunCommand (()-> m_liftSubsystem.setSpeed(0)));
    new JoystickButton(driverJoystick, 12).whileTrue(new RunCommand (()->m_liftSubsystem.unsafeSpeed(-0.3))).onFalse(new RunCommand (()-> m_liftSubsystem.setSpeed(0)));


    //new JoystickButton(driverJoystick, 15).whileTrue(new RunCommand (()->m_armSubsystem.setMotor(0.3))).onFalse(new RunCommand (()->m_armSubsystem.setMotor(0.0)));

    //new JoystickButton(driverJoystick, 15).whileTrue(new RunCommand (()->m_liftSubsystem.unsafeSpeed(-0.3))).onFalse(new RunCommand (()->m_liftSubsystem.unsafeSpeed(0.0)));
    //new JoystickButton(driverJoystick, 14).whileTrue(new RunCommand (()->m_liftSubsystem.unsafeSpeed(0.3))).onFalse(new RunCommand (()->m_liftSubsystem.unsafeSpeed(0.0)));


    //new JoystickButton(driverJoystick, 15).whileTrue(new RunCommand (()->m_shooterSubsystem.setBothMotors(0.6))).onFalse(new RunCommand (()->m_shooterSubsystem.setBothMotors(0.0)));
    //new JoystickButton(driverJoystick, 14).whileTrue(new RunCommand (()->m_shooterSubsystem.setBothMotors(-0.6))).onFalse(new RunCommand (()->m_shooterSubsystem.setBothMotors(0.0)));


    //new JoystickButton(driverJoystick, 15).onTrue(new InstantCommand(()-> m_linearSlideSubsystem.setSetpoint(1, m_armSubsystem.getCurrentAngle()))).onFalse(new InstantCommand(()-> m_linearSlideSubsystem.setSetpoint(0, m_armSubsystem.getCurrentAngle())));

    //new JoystickButton(driverJoystick, 15).whileTrue(new RunCommand (()->m_linearSlideSubsystem.setMotors(0.3))).onFalse(new RunCommand (()->m_linearSlideSubsystem.setMotors(0.0)));
    //new JoystickButton(driverJoystick, 14).whileTrue(new RunCommand (()->m_linearSlideSubsystem.setMotors(-0.3))).onFalse(new RunCommand (()->m_linearSlideSubsystem.setMotors(0.0)));


    /*
    new JoystickButton(driverJoystick, 1).onTrue(
      new InstantCommand(()-> m_shooterSubsystem.setBothMotorsForTime(
        m_visionSubsystem.calculateShot(
        2.262048,
        m_visionSubsystem.getDistanceFromTarget(2),
        m_armSubsystem,
        m_linearSlideSubsystem), Constants.Shooter.secondsToShoot),
        m_shooterSubsystem));
    new JoystickButton(driverJoystick, 3).onTrue(
      new InstantCommand(()-> m_shooterSubsystem.setBothMotorsForTime(
        m_visionSubsystem.calculatePerfectShot(
        2.262048,
        (m_visionSubsystem.getDistanceFromTarget(2) * 1.75),
        m_visionSubsystem.getDistanceFromTarget(2),
        m_armSubsystem,
        m_linearSlideSubsystem), Constants.Shooter.secondsToShoot),
        m_shooterSubsystem));
    new JoystickButton(driverJoystick, 2).onTrue(
      new InstantCommand(()-> m_shooterSubsystem.setBothMotorsForTime(
        m_visionSubsystem.calculateShotByVolts(), Constants.Shooter.secondsToShoot),
        m_shooterSubsystem));
    */

    /* 
    new JoystickButton(driverJoystick, 3).whileTrue(
      new RunCommand(()-> m_linearSlideSubsystem.increaseSetpoint(
        m_armSubsystem.getCurrentAngle()),
         m_linearSlideSubsystem));
    new JoystickButton(driverJoystick, 4).whileTrue(
      new RunCommand(()-> m_linearSlideSubsystem.decreaseSetpoint(
        m_armSubsystem.getCurrentAngle()),
          m_linearSlideSubsystem));
    new JoystickButton(driverJoystick, 2).whileTrue(
      new RunCommand(()-> m_visionSubsystem.pickUpTarget(
        m_swerveSubsystem,
        m_linearSlideSubsystem,
        m_armSubsystem,
        m_shooterSubsystem)));
    */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
