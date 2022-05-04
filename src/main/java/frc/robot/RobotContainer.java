package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.commands.ChangeDriveMode;
import frc.robot.commands.Drive;
import frc.robot.subsystems.DriveTrain;

public class RobotContainer {
 // tank drive motors
 public static WPI_TalonFX upperLeftMotor = new WPI_TalonFX(Constants.DriveConstants.UPPER_LEFT_MOTOR);
 public static WPI_TalonFX lowerLeftMotor = new WPI_TalonFX(Constants.DriveConstants.LOWER_LEFT_MOTOR);

  public static WPI_TalonFX upperRightMotor = new WPI_TalonFX(Constants.DriveConstants.UPPER_RIGHT_MOTOR);
  public static WPI_TalonFX lowerRightMotor = new WPI_TalonFX(Constants.DriveConstants.LOWER_RIGHT_MOTOR);

  // tank drive motor groups
  private final static MotorControllerGroup leftMotors = new MotorControllerGroup(upperLeftMotor, lowerLeftMotor);
  private final static MotorControllerGroup rightMotors = new MotorControllerGroup(upperRightMotor, lowerRightMotor);

  // Controllers
  public final static Joystick driveController = new Joystick(Constants.DriveConstants.DRIVE_CONTROLLER);
  public final static XboxController buttonController  = new XboxController(Constants.DriveConstants.BUTTON_CONTROLLER); 

  // joy buttons
  //public final JoystickButton enableLimey = new JoystickButton(driveController, Constants.JoystickButtons.LIMEY_TOGGLE);

  // buttons

  // shooter motors
 
  // subsystems
  public static final DriveTrain driveSubsystem = new DriveTrain(leftMotors, rightMotors, driveController, upperLeftMotor, upperRightMotor);

  //Limit Switch
  
  // commands
  public final Drive driveCommand = new Drive(driveSubsystem);
  public final ChangeDriveMode changeDriveMode = new ChangeDriveMode(driveSubsystem, Constants.DriveModes.JOYSTICK_DRIVE); // default drive mode is manual joystick
  //public final Temp tempCommand = new Temp();

  public RobotContainer() {
    leftMotors.setInverted(false);
    rightMotors.setInverted(true);
    driveSubsystem.setDefaultCommand(driveCommand);
    configureButtonBindings();
  }
  
  private void configureButtonBindings() {
  }

}
