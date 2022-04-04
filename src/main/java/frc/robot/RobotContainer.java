package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import frc.robot.commands.Drive;
import frc.robot.commands.LimelightAlignToLower;
import frc.robot.commands.ToClosestPlottedPosition;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimelightMath;
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
  public final JoystickButton climberUp = new JoystickButton(buttonController, Constants.GamepadButtons.CLIMBER_UP);
  public final JoystickButton climberDown = new JoystickButton(buttonController, Constants.GamepadButtons.CLIMBER_DOWN);
  public final JoystickButton shooterButton = new JoystickButton(buttonController, Constants.GamepadButtons.SHOOTER);
  public final JoystickButton lowerIntake = new JoystickButton(buttonController, Constants.GamepadButtons.LOWER_INTAKE);
  public final JoystickButton reverseIndexers = new JoystickButton(buttonController, Constants.GamepadButtons.REVERSE_LOWER);
  public final JoystickButton raiseIntake = new JoystickButton(buttonController, Constants.GamepadButtons.RAISE_INTAKE);
  public final JoystickButton intakeAndIndex = new JoystickButton(buttonController, Constants.GamepadButtons.INTAKE);
  public final JoystickButton topIndexer = new JoystickButton(buttonController, Constants.GamepadButtons.UPPER_BELTS);

  // joy buttons
  public final JoystickButton enableLimey = new JoystickButton(driveController, Constants.JoystickButtons.LIMEY_TOGGLE);

  public final WPI_TalonFX shooterMotor = new WPI_TalonFX(Constants.IntegratedShooterPID.SHOOTIE_ID);
  public final WPI_TalonFX shooterMotor2 = new WPI_TalonFX(Constants.IntegratedShooterPID.LOADIE_ID);
 
  // subsystems
  public static final DriveTrain driveSubsystem = new DriveTrain(leftMotors, rightMotors, driveController, upperLeftMotor, upperRightMotor);
  public final ShooterIntegratedPID shooterSubsystem = new ShooterIntegratedPID(shooterMotor, shooterMotor2, verticalIndexerMotor, horizontalIndexerMotor);
  public final Climber climberSubsystem = new Climber(rightClimberMotor, leftClimberMotor, buttonController, ahrs);
  public static LimelightMath LimeMath = new LimelightMath();
  
  // commands
  public final Drive driveCommand = new Drive(driveSubsystem);
  public final ChangeDriveMode changeDriveMode = new ChangeDriveMode(driveSubsystem, Constants.DriveConstants.JOYSTICK_DRIVE); // default drive mode is manual joystick

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    leftMotors.setInverted(true);
    rightMotors.setInverted(false);
    driveSubsystem.setDefaultCommand(driveCommand);
    configureButtonBindings();
  }
  
  private void configureButtonBindings() {
    enableLimey.whenPressed(
      new SequentialCommandGroup(
        new InstantCommand(shooterSubsystem::limeOn, shooterSubsystem),
        new LimelightAlignToLower(driveSubsystem),
        new ToClosestPlottedPosition(driveSubsystem)
      )); // Enables limey, then aligns to lower, once aligned to lower it then moves to the closest plotted position to have precise RPM control.

    // shootBall.whenPressed(new InstantCommand(shooterSubsystem::toggleOn, shooterSubsystem)); // Toggle on while button is held
    // shootBall.whenReleased(new InstantCommand(shooterSubsystem::toggleOff, shooterSubsystem));
    // shootBall.whileHeld(new InstantCommand(shooterSubsystem::indexCargo, shooterSubsystem));

    

    // climbButton.whenPressed(new InstantCommand(climberSubsystem::climberToggle, climberSubsystem)); // Requests the opposite mode, to disable or reenable.
    // encoderButton.whenPressed(new ChangeDriveMode(driveSubsystem, Constants.DriveConstants.ENCODER_DRIVE)); // change drive mode to encoder
  }
}
