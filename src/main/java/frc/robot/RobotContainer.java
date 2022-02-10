package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Drive;
import frc.robot.subsystems.DriveTrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private int driveMode = Constants.driveModeConstants.joystickDrive; 

  // tank drive motors
  private static MotorController upperLeftMotor = new WPI_TalonFX(Constants.driveConstants.upperLeftMotor);
  private final MotorController lowerLeftMotor = new WPI_TalonFX(Constants.driveConstants.lowerLeftMotor);

  private static MotorController upperRightMotor = new WPI_TalonFX(Constants.driveConstants.upperRightMotor);
  private final MotorController lowerRightMotor = new WPI_TalonFX(Constants.driveConstants.lowerRightMotor);

  // tank drive motor groups
  private final MotorControllerGroup leftMotors = new MotorControllerGroup(upperLeftMotor, lowerLeftMotor);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(upperRightMotor, lowerRightMotor);

  // initing the Joysticks so that we can pass them to the Drive command
  public final XboxController driveController = new XboxController(Constants.driveConstants.driveController);
  public final JoystickButton encoderButton = new JoystickButton(driveController, Constants.encoderConstants.encoderButton); // pressing the button will ONLY enable driving with encoders. It will toggle itself off after running the comman

  // subsystems
  public final DriveTrain driveSubsystem = new DriveTrain(leftMotors, rightMotors, driveController, upperLeftMotor, upperRightMotor);

  // commands
  public final Drive driveCommand = new Drive(driveSubsystem, driveMode);
  
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
    encoderButton.whenPressed(new Drive(driveSubsystem, Constants.driveModeConstants.encoderDrive)); 
  }
}
