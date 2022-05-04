package frc.robot;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command autoCommand;
  private RobotContainer m_robotContainer;

  //private DriveTrain driveSub = new DriveTrain(leftMotors, rightMotors, driveController, encLeftMotor, encRightMotor)
  public static double distanceToHub;
  public static double currentTime;
  private double lastCall = 0;
  UsbCamera driverVision;



  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
   // UsbCamera camcam = new CameraServer();
   // driverVision = CameraServer.startAutomaticCapture(0);
    //m_robotContainer.driveSubsystem.declareGrip();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //lastCall = Timer.getFPGATimestamp();
  }

  /** Autonomous Sequence assumes that the robot starts out with the robot aligned with a cargo ball
   *  1. Taxis bot out of tarmac and intakes a cargo
   *  2. Rotate 180 deg
   *  3. Call limelight align method and shoot both cargo
   */
  @Override
  public void autonomousPeriodic() {
    /*
    SmartDashboard.putNumber("current time:"  , Timer.getFPGATimestamp());
    SmartDashboard.putNumber("last call:", lastCall);


    if(Timer.getFPGATimestamp() - lastCall < 3) {
      RobotContainer.driveSubsystem.differentialDriveSub.arcadeDrive(0.3, 0);
    }
    else { // stop
      RobotContainer.driveSubsystem.differentialDriveSub.arcadeDrive(0, 0);
    }
    */
     //RobotContainer.driveSubsystem.encoderDrive();
    // if (RobotContainer.driveSubsystem.averageDisplacement >= Constants.EncoderConstants.TARGET_DISTANCE_FT) { // stop running encoderDrive
    //   RobotContainer.indexerSubsystem.absorb();
    //   RobotContainer.driveSubsystem.halfTurn();
    //   if(RobotContainer.driveSubsystem.avgRevolutionsTracked >= Constants.EncoderConstants.HALF_TURN) {
    //     RobotContainer.driveSubsystem.trackObject();
    //     RobotContainer.driveSubsystem.stop();
    //     // ↓ takes RPM instead of percentage as param ↓
    //     RobotContainer.shooterSubsystem.setVelocity(3395);

    //   }
    // }
    //RobotContainer.driveSubsystem.autoDrive(); 
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // if (autoCommand != null) {
    //   autoCommand.cancel();
    // }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    currentTime = Timer.getFPGATimestamp();
    // ↓ returns the hypotenuse to the hub, not the horizontal distance ↓ 
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
