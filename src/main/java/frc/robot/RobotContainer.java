package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
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

  // left motors
  private final MotorController upperLeftMotor = new WPI_TalonFX(Constants.DriveConstants.upperLeftMotor);
  private final MotorController lowerLeftMotor = new WPI_TalonFX(Constants.DriveConstants.lowerLeftMotor);

  private final WPI_TalonFX left = new WPI_TalonFX(Constants.DriveConstants.lowerLeftMotor);

  // right motors
  private final MotorController upperRightMotor = new WPI_TalonFX(Constants.DriveConstants.upperRightMotor);
  private final MotorController lowerRightMotor = new WPI_TalonFX(Constants.DriveConstants.lowerRightMotor);

  // The motors on the left side of the drive.
  private final MotorControllerGroup leftMotors = new MotorControllerGroup(upperLeftMotor, lowerLeftMotor);

  // The motors on the right side of the drive.
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(upperRightMotor, lowerRightMotor);

  // Initing the Joysticks so that we can pass them to the Drive command
  public final XboxController driveController = new XboxController(Constants.DriveConstants.driveController);
  public final JoystickButton driveModeChanger = new JoystickButton(driveController, 1);
  
  public final JoystickButton runShooterRPM = new JoystickButton(driveController, 2);
  public final JoystickButton stopShooterRPM = new JoystickButton(driveController, 3);

  // Initing the Shooter motors and their CANCoders
  //private final WPI_TalonFX shooterMotor = new WPI_TalonFX(Constants.ShooterConstants.shootMotor);
  //private final WPI_TalonFX loaderMotor = new WPI_TalonFX(Constants.ShooterConstants.loaderMotor);

  // Initing the Shooter subsystem
  //private final ShooterPID shooterSub = new ShooterPID(shooterMotor);

  // Initing the PID Command to demand the shooter's RPM based on it's current RPM

  // Drive subsystem
  public final DriveTrain driveSubsystem = new DriveTrain(leftMotors, rightMotors, driveController);
  
  // Drive command
  public final Drive driveCommand = new Drive(driveSubsystem);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //invert right side of the drive train
    //leftMotors.setInverted(true);
    //driveSubsystem.setDefaultCommand(driveCommand);
    // Configure the button bindings
    configureButtonBindings();
  }
 
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driveModeChanger.whenPressed(new ChangeDriveMode(driveSubsystem));
    //runShooterRPM.whenPressed(new InstantCommand(shooterSub::enable, shooterSub));
    //stopShooterRPM.whenPressed(new InstantCommand(shooterSub::disable, shooterSub));
  }
}
