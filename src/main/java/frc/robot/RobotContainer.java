// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ChangeDriveMode;
import frc.robot.commands.Drive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ShooterIntegratedPID;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public final XboxController driveController = new XboxController(Constants.driveConstants.DRIVE_CONTROLLER);

  // shooter stuff
  public final WPI_TalonFX shooterMotor = new WPI_TalonFX(Constants.IntegratedShooterPID.SHOOTIE_ID);
  public final WPI_TalonFX shooterMotor2 = new WPI_TalonFX(Constants.IntegratedShooterPID.LOADIE_ID);
  public final ShooterIntegratedPID shooter = new ShooterIntegratedPID(shooterMotor, shooterMotor2);

  public final JoystickButton limey = new JoystickButton(driveController, 2);

  // tank drive motors
  private static MotorController upperLeftMotor = new WPI_TalonFX(Constants.driveConstants.UPPER_LEFT_MOTOR);
  private final MotorController lowerLeftMotor = new WPI_TalonFX(Constants.driveConstants.LOWER_LEFT_MOTOR);

  private static MotorController upperRightMotor = new WPI_TalonFX(Constants.driveConstants.UPPER_RIGHT_MOTOR);
  private final MotorController lowerRightMotor = new WPI_TalonFX(Constants.driveConstants.LOWER_RIGHT_MOTOR);

  // tank drive motor groups
  private final MotorControllerGroup leftMotors = new MotorControllerGroup(upperLeftMotor, lowerLeftMotor);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(upperRightMotor, lowerRightMotor);

  // initing the Joysticks so that we can pass them to the Drive command
  public final JoystickButton encoderButton = new JoystickButton(driveController, Constants.gamepadButtons.ENCODER_BUTTON); // pressing the button will ONLY enable driving with encoders. It will toggle itself off after running the comman

  // subsystems
  // public final DriveTrain driveSubsystem = new DriveTrain(leftMotors, rightMotors, driveController, upperLeftMotor, upperRightMotor);
  public final DriveTrain driveSubsystem = new DriveTrain(leftMotors, rightMotors, driveController);

  // commands
  public final Drive driveCommand = new Drive(driveSubsystem);
  public final ChangeDriveMode changeDriveMode = new ChangeDriveMode(driveSubsystem, Constants.driveModeConstants.JOYSTICK_DRIVE); // default drive mode is manual joystick

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    leftMotors.setInverted(true);
    rightMotors.setInverted(false);
    driveSubsystem.setDefaultCommand(driveCommand);
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    encoderButton.whenPressed(new ChangeDriveMode(driveSubsystem, Constants.driveModeConstants.ENCODER_DRIVE)); 
    limey.whenPressed(new ChangeDriveMode(driveSubsystem, Constants.driveModeConstants.LIMELIGHT_DRIVE));
  }
}
