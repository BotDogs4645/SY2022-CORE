package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import frc.robot.commands.Drive;
import frc.robot.commands.LimelightAlignToLower;
import frc.robot.commands.ToClosestPlottedPosition;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LimelightMath;
import frc.robot.commands.AutoProfile1;
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
  private final static WPI_TalonFX verticalIndexerMotor = new WPI_TalonFX(Constants.IndexerConstants.VERTICAL_INDEXER_MOTOR);
  private final static WPI_TalonFX horizontalIndexerMotor = new WPI_TalonFX(Constants.IndexerConstants.HORIZONTAL_INDEXER_MOTOR);
  
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

  public final static WPI_TalonFX shooterMotor = new WPI_TalonFX(Constants.IntegratedShooterPID.SHOOTIE_ID);
  public final static WPI_TalonFX shooterMotor2 = new WPI_TalonFX(Constants.IntegratedShooterPID.LOADIE_ID);
 
  // subsystems
  public static final DriveTrain driveSubsystem = new DriveTrain(leftMotors, rightMotors, driveController, upperLeftMotor, upperRightMotor);
  public static final ShooterIntegratedPID shooterSubsystem = new ShooterIntegratedPID(shooterMotor, shooterMotor2, verticalIndexerMotor, horizontalIndexerMotor);
  public final Climber climberSubsystem = new Climber(rightClimberMotor, leftClimberMotor, buttonController, ahrs);
  public static LimelightMath LimeMath = new LimelightMath();
  public static Indexer indexerSubsystem = new Indexer(verticalIndexerMotor, horizontalIndexerMotor);
  
  // commands
  public final Drive driveCommand = new Drive(driveSubsystem);
  public final ChangeDriveMode changeDriveMode = new ChangeDriveMode(driveSubsystem, Constants.DriveConstants.JOYSTICK_DRIVE); // default drive mode is manual joystick

  public SendableChooser<Command> chooser = new SendableChooser<>();
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
        new ToClosestPlottedPosition(driveSubsystem))
      ); 
      chooser.addOption("Main", new InstantCommand());
      chooser.addOption("Auto 1", new AutoProfile1(driveSubsystem, shooterSubsystem));

      // chooser.addOption("auto1",);
      Shuffleboard.getTab("Main").add("Auto Command", chooser).withPosition(1, 1);
  }
  public Command getAutoCommand() {
    return chooser.getSelected();
  }
}
