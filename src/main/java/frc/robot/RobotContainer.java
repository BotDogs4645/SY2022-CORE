// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ChangeDriveMode;
import frc.robot.commands.Drive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ShooterIntegratedPID;
import frc.robot.subsystems.Climber;

public class RobotContainer {
 // tank drive motors
  private static WPI_TalonFX upperLeftMotor = new WPI_TalonFX(Constants.driveConstants.UPPER_LEFT_MOTOR);
  private final WPI_TalonFX lowerLeftMotor = new WPI_TalonFX(Constants.driveConstants.LOWER_LEFT_MOTOR);

  private static WPI_TalonFX upperRightMotor = new WPI_TalonFX(Constants.driveConstants.UPPER_RIGHT_MOTOR);
  private final WPI_TalonFX lowerRightMotor = new WPI_TalonFX(Constants.driveConstants.LOWER_RIGHT_MOTOR);
 // climber motors
  public final CANSparkMax rightClimberMotor = new CANSparkMax(Constants.climberConstants.LEFT_CLIMBER_ID, MotorType.kBrushed);
  public final CANSparkMax leftClimberMotor = new CANSparkMax(Constants.climberConstants.RIGHT_CLIMBER_ID, MotorType.kBrushed);
// tank drive motor groups
  private final MotorControllerGroup leftMotors = new MotorControllerGroup(upperLeftMotor, lowerLeftMotor);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(upperRightMotor, lowerRightMotor);

  public final XboxController driveController = new XboxController(Constants.driveConstants.DRIVE_CONTROLLER);
  // buttons
  public final JoystickButton encoderButton = new JoystickButton(driveController, Constants.gamepadButtons.ENCODER_DRIVE); // pressing the button will ONLY enable driving with encoders. It will toggle itself off after running the comman
  public final JoystickButton limelightButton = new JoystickButton(driveController, Constants.gamepadButtons.LIMELIGHT_DRIVE);
  public final JoystickButton joyDisable = new JoystickButton(driveController, Constants.gamepadButtons.CLIMBER_BUTTON);
  public final JoystickButton joyEnable = new JoystickButton(driveController, Constants.gamepadButtons.PID_BUTTON);
  public final JoystickButton gripButton = new JoystickButton(driveController, Constants.gamepadButtons.GRIP_BUTTON);
  // fix these button constants, they overlap -
   // shooter  --> COMMENTED OUT BC MOTORS ARE MISSING FROM CHASSIS
  public final WPI_TalonFX shooterMotor = new WPI_TalonFX(Constants.IntegratedShooterPID.SHOOTIE_ID);
  public final WPI_TalonFX shooterMotor2 = new WPI_TalonFX(Constants.IntegratedShooterPID.LOADIE_ID);
 
  // subsystems
  public final DriveTrain driveSubsystem = new DriveTrain(leftMotors, rightMotors, driveController, upperLeftMotor, upperRightMotor);
  public final ShooterIntegratedPID shooter = new ShooterIntegratedPID(shooterMotor, shooterMotor2);
  public final Climber climberSubsystem = new Climber(rightClimberMotor, leftClimberMotor, driveController);

  // commands
   public final Drive driveCommand = new Drive(driveSubsystem);
   public final ChangeDriveMode changeDriveMode = new ChangeDriveMode(driveSubsystem, Constants.gamepadButtons.JOYSTICK_DRIVE); // default drive mode is manual joystick

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
    //pidButton.whenPressed(new ConditionalCommand(new InstantCommand(shooter::enable), new InstantCommand(shooter::disable), shooter::getOnOffFlag));
    //climbButton.whenPressed(new ConditionalCommand(new InstantCommand(climberSubsystem::climberUp), new InstantCommand(climberSubsystem::climberDown), climberSubsystem::getUpFlag));
    //encoderButton.whenPressed(new ChangeDriveMode(driveSubsystem, Constants.gamepadButtons.ENCODER_DRIVE)); // change drive mode to encoder
    //limelightButton.whenPressed(new ChangeDriveMode(driveSubsystem, Constants.gamepadButtons.LIMELIGHT_DRIVE));
    
  }
}
