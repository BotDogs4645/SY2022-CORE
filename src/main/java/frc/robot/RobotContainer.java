package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import frc.robot.commands.Drive;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.commands.ChangeDriveMode;
import frc.robot.subsystems.ShooterIntegratedPID;

public class RobotContainer {
 // tank drive motors
 private static WPI_TalonFX upperLeftMotor = new WPI_TalonFX(Constants.DriveConstants.UPPER_LEFT_MOTOR);
 private final static WPI_TalonFX lowerLeftMotor = new WPI_TalonFX(Constants.DriveConstants.LOWER_LEFT_MOTOR);

  private static WPI_TalonFX upperRightMotor = new WPI_TalonFX(Constants.DriveConstants.UPPER_RIGHT_MOTOR);
  private final static WPI_TalonFX lowerRightMotor = new WPI_TalonFX(Constants.DriveConstants.LOWER_RIGHT_MOTOR);
 
  // climber motors
  private final CANSparkMax rightClimberMotor = new CANSparkMax(Constants.ClimberConstants.LEFT_CLIMBER_ID, MotorType.kBrushed);
  private final CANSparkMax leftClimberMotor = new CANSparkMax(Constants.ClimberConstants.RIGHT_CLIMBER_ID, MotorType.kBrushed);
  private final AHRS ahrs = new AHRS();

  // indexer motors
  private final WPI_TalonFX verticalIndexerMotor = new WPI_TalonFX(Constants.IndexerConstants.VERTICAL_INDEXER_MOTOR);
  private final WPI_TalonFX horizontalIndexerMotor = new WPI_TalonFX(Constants.IndexerConstants.HORIZONTAL_INDEXER_MOTOR);
  
  // tank drive motor groups
  private final static MotorControllerGroup leftMotors = new MotorControllerGroup(upperLeftMotor, lowerLeftMotor);
  private final static MotorControllerGroup rightMotors = new MotorControllerGroup(upperRightMotor, lowerRightMotor);

  // Controllers
  public final static Joystick driveController = new Joystick(Constants.DriveConstants.DRIVE_CONTROLLER);
  public final XboxController buttonController  = new XboxController(Constants.DriveConstants.BUTTON_CONTROLLER); 

  // xbox buttons
  public final JoystickButton encoderButton = new JoystickButton(buttonController, Constants.GamepadButtons.ENCODER_DRIVE); // pressing the button will ONLY enable driving with encoders. It will toggle itself off after running the comman
  public final JoystickButton climbButton = new JoystickButton(buttonController, Constants.GamepadButtons.CLIMBER_BUTTON);
  public final JoystickButton shooterButton = new JoystickButton(buttonController, Constants.GamepadButtons.SHOOTER);

  // joy buttons
<<<<<<< Updated upstream
  public final JoystickButton shootBall = new JoystickButton(driveController, Constants.JoystickButtons.FIRE_SHOOTER);
  public final JoystickButton enableLimey = new JoystickButton(driveController, Constants.JoystickButtons.LIMEY_TOGGLE);
=======
  public final JoystickButton shootBall = new JoystickButton(driveController, Constants.GamepadButtons.FIRE_SHOOTER);
  public final JoystickButton enableLimey = new JoystickButton(driveController, Constants.GamepadButtons.LIMEY_TOGGLE); 
>>>>>>> Stashed changes

  public final WPI_TalonFX shooterMotor = new WPI_TalonFX(Constants.IntegratedShooterPID.SHOOTIE_ID);
  public final WPI_TalonFX shooterMotor2 = new WPI_TalonFX(Constants.IntegratedShooterPID.LOADIE_ID);
 
  // subsystems
  public static final DriveTrain driveSubsystem = new DriveTrain(leftMotors, rightMotors, driveController, upperLeftMotor, upperRightMotor);
  public final ShooterIntegratedPID shooterSubsystem = new ShooterIntegratedPID(shooterMotor, shooterMotor2, verticalIndexerMotor, horizontalIndexerMotor);
  public final Climber climberSubsystem = new Climber(rightClimberMotor, leftClimberMotor, buttonController, ahrs);
  
  // commands
  public final Drive driveCommand = new Drive(driveSubsystem);
  public final ChangeDriveMode changeDriveMode = new ChangeDriveMode(driveSubsystem, Constants.GamepadButtons.JOYSTICK_DRIVE); // default drive mode is manual joystick

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    leftMotors.setInverted(true);
    rightMotors.setInverted(false);
    driveSubsystem.setDefaultCommand(driveCommand);
    configureButtonBindings();
  }
  
  private void configureButtonBindings() {
    enableLimey.whenPressed(new InstantCommand(shooterSubsystem::limeyToggle, shooterSubsystem)); // Requests the opposite mode, to disable or reenable.

    // shootBall.whenPressed(new InstantCommand(shooterSubsystem::toggleOn, shooterSubsystem)); // Toggle on while button is held
    // shootBall.whenReleased(new InstantCommand(shooterSubsystem::toggleOff, shooterSubsystem));
    // shootBall.whileHeld(new InstantCommand(shooterSubsystem::indexCargo, shooterSubsystem));




    climbButton.whenPressed(new InstantCommand(climberSubsystem::climberToggle, climberSubsystem)); // Requests the opposite mode, to disable or reenable.
    encoderButton.whenPressed(new ChangeDriveMode(driveSubsystem, Constants.GamepadButtons.ENCODER_DRIVE)); // change drive mode to encoder
  }
}
