package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ChangeDriveMode;
import frc.robot.commands.Drive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ShooterPID;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // tank drive motors
  //Left motors
  private static MotorController upperLeftMotor = new WPI_TalonFX(Constants.driveConstants.UPPER_LEFT_MOTOR);
  private final MotorController lowerLeftMotor = new WPI_TalonFX(Constants.driveConstants.LOWER_LEFT_MOTOR);
  private final WPI_TalonFX left = new WPI_TalonFX(Constants.DriveConstantsLOWER_LEFT_MOTOR);

  // right motors
  private static MotorController upperRightMotor = new WPI_TalonFX(Constants.driveConstants.UPPER_RIGHT_MOTOR);
  private final MotorController lowerRightMotor = new WPI_TalonFX(Constants.driveConstants.LOWER_RIGHT_MOTOR);

  // tank drive motor groups
  private final MotorControllerGroup leftMotors = new MotorControllerGroup(upperLeftMotor, lowerLeftMotor);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(upperRightMotor, lowerRightMotor);

  // Initing the Joysticks so that we can pass them to the Drive command
  public final XboxController driveController = new XboxController(Constants.DriveConstants.DRIVE_CONTROLLER);
  public final JoystickButton encoderButton = new JoystickButton(driveController, Constants.encoderConstants.ENCODER_BUTTON); // pressing the button will ONLY enable driving with encoders. It will toggle itself off after running the comman
  
  public final JoystickButton runShooterRPM = new JoystickButton(driveController, 2);
  public final JoystickButton stopShooterRPM = new JoystickButton(driveController, 3);

  // Initing the Shooter motors and their CANCoders
  private final WPI_TalonFX shooterMotor = new WPI_TalonFX(Constants.ShooterConstants.shootMotor);
  private final WPI_TalonFX loaderMotor = new WPI_TalonFX(Constants.ShooterConstants.loaderMotor);

  // Initing the Shooter subsystem
  private final ShooterPID shooterSub = new ShooterPID(shooterMotor, loaderMotor);

  // Initing the PID Command to demand the shooter's RPM based on it's current RPM

  // Drive subsystem
  public final DriveTrain driveSubsystem = new DriveTrain(leftMotors, rightMotors, driveController, upperLeftMotor, upperRightMotor, left);
  
  // commands
  public final Drive driveCommand = new Drive(driveSubsystem);
  public final Drive changeDriveMode = new ChangeDriveMode(driveSubsystem, Constants.driveModeConstants.JOYSTICK_DRIVE); // default drive mode is manual joystick
  
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
    encoderButton.whenPressed(new ChangeDriveMode(driveSubsystem, Constants.driveModeConstants.ENCODER_DRIVE)); // change drive mode to encoder

  }
}
