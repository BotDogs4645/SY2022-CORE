package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Drive;
import frc.robot.commands.LimelightAlignToLower;
import frc.robot.commands.ToClosestPlottedPosition;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LimelightMath;
import frc.robot.Constants.JoystickButtons;
import frc.robot.commands.AutoProfile1;
import frc.robot.commands.ChangeDriveMode;
import frc.robot.subsystems.ShooterIntegratedPID;

public class RobotContainer {
  public static LimelightMath LimeMath = new LimelightMath();
 // tank drive motors
 private static WPI_TalonFX upperLeftMotor = new WPI_TalonFX(Constants.DriveConstants.UPPER_LEFT_MOTOR);
 private final static WPI_TalonFX lowerLeftMotor = new WPI_TalonFX(Constants.DriveConstants.LOWER_LEFT_MOTOR);

  private static WPI_TalonFX upperRightMotor = new WPI_TalonFX(Constants.DriveConstants.UPPER_RIGHT_MOTOR);
  private final static WPI_TalonFX lowerRightMotor = new WPI_TalonFX(Constants.DriveConstants.LOWER_RIGHT_MOTOR);
 
  // climber motors
  private final static WPI_TalonSRX rightClimberMotor = new WPI_TalonSRX(Constants.ClimberConstants.LEFT_CLIMBER_ID);
  private final static WPI_TalonSRX leftClimberMotor = new WPI_TalonSRX(Constants.ClimberConstants.RIGHT_CLIMBER_ID);

  // indexer motors
  private final static WPI_TalonFX verticalIndexerMotor = new WPI_TalonFX(Constants.IndexerConstants.VERTICAL_INDEXER_MOTOR);
  private final static WPI_TalonSRX horizontalIndexerMotor = new WPI_TalonSRX(Constants.IndexerConstants.HORIZONTAL_INDEXER_MOTOR);

  // tank drive motor groups
  private final static MotorControllerGroup leftMotors = new MotorControllerGroup(upperLeftMotor, lowerLeftMotor);
  private final static MotorControllerGroup rightMotors = new MotorControllerGroup(upperRightMotor, lowerRightMotor);

  // Controllers
  public final static Joystick driveController = new Joystick(Constants.DriveConstants.DRIVE_CONTROLLER);
  public final static XboxController buttonController  = new XboxController(Constants.DriveConstants.BUTTON_CONTROLLER); 

  // joy buttons
  public final JoystickButton enableLimey = new JoystickButton(driveController, Constants.JoystickButtons.LIMEY_TOGGLE);

  // buttons
  public final JoystickButton alignButton = new JoystickButton(buttonController, Constants.DriveModes.LIMELIGHT_DRIVE);
  //public final JoystickButton climbButton = new JoystickButton(buttonController, Constants.GamepadButtons.CLIMBER_BUTTON);
  public final JoystickButton shooterButton = new JoystickButton(driveController, Constants.JoystickButtons.SHOOTER);
  public final static JoystickButton absorbButton = new JoystickButton(buttonController, Constants.GamepadButtons.ABSORB);
  public final static JoystickButton unabsorbButton = new JoystickButton(buttonController, Constants.GamepadButtons.UNABSORB);
  public final JoystickButton verticalIndexerButton = new JoystickButton(buttonController, Constants.GamepadButtons.VERTICAL_INDEXER);
  public final JoystickButton raiseIntakeButton = new JoystickButton(buttonController, Constants.GamepadButtons.RAISE_INTAKE);
  public final JoystickButton lowerIntakeButton = new JoystickButton(buttonController, Constants.GamepadButtons.LOWER_INTAKE);
  public final JoystickButton shooterOn = new JoystickButton(driveController, Constants.JoystickButtons.SHOOTER_ENABLE);

  // shooter motors
  public final static WPI_TalonFX shooterMotor = new WPI_TalonFX(Constants.IntegratedShooterPID.SHOOTIE_ID);
  public final static WPI_TalonFX shooterMotor2 = new WPI_TalonFX(Constants.IntegratedShooterPID.LOADIE_ID);
 
  // subsystems
  public static final DriveTrain driveSubsystem = new DriveTrain(leftMotors, rightMotors, driveController, upperLeftMotor, upperRightMotor);
  public static final ShooterIntegratedPID shooterSubsystem = new ShooterIntegratedPID(shooterMotor, shooterMotor2, verticalIndexerMotor, horizontalIndexerMotor);
  public final static Climber climberSubsystem = new Climber(rightClimberMotor, leftClimberMotor, buttonController);
  public static Indexer indexerSubsystem = new Indexer(verticalIndexerMotor, horizontalIndexerMotor);
  
  // commands
  public final Drive driveCommand = new Drive(driveSubsystem);
  public final ChangeDriveMode changeDriveMode = new ChangeDriveMode(driveSubsystem, Constants.DriveModes.JOYSTICK_DRIVE); // default drive mode is manual joystick

  public SendableChooser<Command> chooser = new SendableChooser<>();

  public RobotContainer() {
    leftMotors.setInverted(false);
    rightMotors.setInverted(true);
    driveSubsystem.setDefaultCommand(driveCommand);
    configureButtonBindings();
  }
  
  private void configureButtonBindings() {
    // enableLimey.whenPressed(
    //   new SequentialCommandGroup(
    //     new InstantCommand(shooterSubsystem::limeOn, shooterSubsystem),
    //     new LimelightAlignToLower(driveSubsystem),
    //     new ToClosestPlottedPosition(driveSubsystem))
    //   ); 
      chooser.addOption("Main", new InstantCommand());
      chooser.addOption("Auto 1", new AutoProfile1(driveSubsystem, shooterSubsystem));

      // chooser.addOption("auto1",);
      Shuffleboard.getTab("Main").add("Auto Command", chooser).withPosition(1, 1);
      alignButton.whenPressed(new ChangeDriveMode(driveSubsystem, Constants.DriveModes.LIMELIGHT_DRIVE));
      shooterOn.whileHeld(new InstantCommand(driveSubsystem::trackObject, driveSubsystem));

      shooterButton.whenPressed(new InstantCommand(shooterSubsystem::setRPMFromDistanceAuto, shooterSubsystem));
      
      // intake + indexer
      absorbButton.whileHeld(new InstantCommand(indexerSubsystem::absorb, indexerSubsystem));
      unabsorbButton.whileHeld(new InstantCommand(indexerSubsystem::unabsorb, indexerSubsystem));

      verticalIndexerButton.whenPressed(new InstantCommand(indexerSubsystem::enableVerticalIndexer, indexerSubsystem));
      verticalIndexerButton.whenReleased(new InstantCommand(indexerSubsystem::stopIndexer, indexerSubsystem));

      raiseIntakeButton.whileHeld(new InstantCommand(indexerSubsystem::raiseIntake, indexerSubsystem));
      lowerIntakeButton.whenPressed(new InstantCommand(indexerSubsystem::lowerIntake, indexerSubsystem));
  }

  public Command getAutoCommand() {
    return chooser.getSelected();

  }
}
