// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.invoke.ConstantBootstraps;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
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
  // shooter stuff
  public final WPI_TalonFX shooterMotor = new WPI_TalonFX(Constants.IntegratedShooterPID.ShooterID);
  public final WPI_TalonFX shooterMotor2 = new WPI_TalonFX(Constants.IntegratedShooterPID.ShooterID2);
  public final ShooterIntegratedPID shooter = new ShooterIntegratedPID(shooterMotor, shooterMotor2);

  public final JoystickButton joyEnable = new JoystickButton(driveController, 2);
  public final JoystickButton joyDisable = new JoystickButton(driveController, 3);

  // tank drive motors
  private static MotorController upperLeftMotor = new WPI_TalonFX(Constants.DriveConstants.UPPER_LEFT_MOTOR);
  private final MotorController lowerLeftMotor = new WPI_TalonFX(Constants.DriveConstants.LOWER_LEFT_MOTOR);

  private static MotorController upperRightMotor = new WPI_TalonFX(Constants.DriveConstants.UPPER_RIGHT_MOTOR);
  private final MotorController lowerRightMotor = new WPI_TalonFX(Constants.DriveConstants.LOWER_RIGHT_MOTOR);

  // tank drive motor groups
  private final MotorControllerGroup leftMotors = new MotorControllerGroup(upperLeftMotor, lowerLeftMotor);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(upperRightMotor, lowerRightMotor);

  // initing the Joysticks so that we can pass them to the Drive command
  public final XboxController driveController = new XboxController(Constants.DriveConstants.DRIVE_CONTROLLER);
  public final JoystickButton encoderButton = new JoystickButton(driveController, Constants.GamepadButtons.ENCODER_BUTTON); // pressing the button will ONLY enable driving with encoders. It will toggle itself off after running the comman

  // subsystems
  public final DriveTrain driveSubsystem = new DriveTrain(leftMotors, rightMotors, driveController, upperLeftMotor, upperRightMotor);

  // commands
  public final Drive driveCommand = new Drive(driveSubsystem);
  public final ChangeDriveMode changeDriveMode = new ChangeDriveMode(driveSubsystem, Constants.DriveModeConstants.JOYSTICK_DRIVE); // default drive mode is manual joystick

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
    joyEnable.whenPressed(new InstantCommand(shooter::enable, shooter));
    joyDisable.whenPressed(new InstantCommand(shooter::disable, shooter));
    encoderButton.whenPressed(new ChangeDriveMode(driveSubsystem, Constants.DriveModeConstants.ENCODER_DRIVE)); // change drive mode to encoder
  }
}
