package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GripPipeline;
import frc.robot.RobotContainer;

public class DriveTrain extends SubsystemBase {
  public static int driveMode;
  private Joystick driveController;

  private double driveSpeed;
  private double turnSpeed;

  SlewRateLimiter filterLeft = new SlewRateLimiter(2.5);
  SlewRateLimiter filterRight = new SlewRateLimiter(2.5);

  private double leftDistanceTraveled;
  private double rightDistanceTraveled;

  private MotorControllerGroup leftMotors;
  private MotorControllerGroup rightMotors;

  private WPI_TalonFX encLeftMotor;
  private WPI_TalonFX encRightMotor;

  private double rawEncoderOutLeft;
  private double rawEncoderOutRight; 
  public double averageDisplacement;

  public boolean resetHalfTurn = true;
  public double avgRevolutionsTracked;

  private GripPipeline pipe;
  private VisionThread VisionThread;
  public static boolean alignedToHub = false;
  public static boolean inPreferredPosition = false;
  private Object foundTarget = new Object();

  public LimelightMath LimeMath;
  public boolean testingMode = true;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-console");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry ty = table.getEntry("ty");

  private final AHRS ahrs = new AHRS();
  public final DifferentialDrive differentialDriveSub;
  private final PIDController drivePID = new PIDController(Constants.EncoderConstants.kP, Constants.EncoderConstants.kI, Constants.EncoderConstants.kD);
  
  public DriveTrain(MotorControllerGroup leftMotors, MotorControllerGroup rightMotors, Joystick driveController, MotorController encLeftMotor, MotorController encRightMotor) {
    this.driveController = driveController;

    driveMode = Constants.DriveModes.JOYSTICK_DRIVE;
    LimeMath = RobotContainer.LimeMath;

    this.leftMotors = leftMotors;
    this.rightMotors = rightMotors;

    this.encLeftMotor = (WPI_TalonFX) encLeftMotor;
    this.encRightMotor = (WPI_TalonFX) encRightMotor;
    
    resetEncoders();
    this.encLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    this.encRightMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    differentialDriveSub = new DifferentialDrive(leftMotors, rightMotors);
    differentialDriveSub.setMaxOutput(Constants.DriveConstants.MAX_OUTPUT);

    ahrs.reset();

    if (testingMode) {
      Shuffleboard.getTab("DriveTrain")
          .add("Rotation FF", 0.0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
          .getEntry()
          .addListener(event -> {
            Constants.LimelightConstants.LIMELIGHT_ROTATION_F = event.getEntry().getValue().getDouble();
          }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
      Shuffleboard.getTab("DriveTrain")
          .add("Forward FF", 0.0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
          .getEntry()
          .addListener(event -> {
            Constants.LimelightConstants.LIMELIGHT_FOW_F = event.getEntry().getValue().getDouble();
          }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
      Shuffleboard.getTab("DriveTrain")
          .add("Forward P", 0.0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
          .getEntry()
          .addListener(event -> {
            Constants.LimelightConstants.LIMELIGHT_FOW_P = event.getEntry().getValue().getDouble();
          }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }
  }
   
  public void updateAverageDisplacement() {
    rawEncoderOutLeft = encLeftMotor.getSelectedSensorPosition();
    rawEncoderOutRight = encRightMotor.getSelectedSensorPosition() * -1;
    
    leftDistanceTraveled = rawEncoderOutLeft / (Constants.EncoderConstants.k_UNITS_P_REVOLUTION * Constants.EncoderConstants.REVOLUTION_P_FT);
    rightDistanceTraveled = rawEncoderOutRight / (Constants.EncoderConstants.k_UNITS_P_REVOLUTION * Constants.EncoderConstants.REVOLUTION_P_FT);

    averageDisplacement = (leftDistanceTraveled + rightDistanceTraveled) / 2; // updates average displacement
  }

  public void updateRevolutionsTracked() {
    if (resetHalfTurn == true) {
      resetEncoders();
      resetHalfTurn = false;
    }
    rawEncoderOutLeft = encLeftMotor.getSelectedSensorPosition() / Constants.EncoderConstants.k_UNITS_P_REVOLUTION;
    rawEncoderOutRight = encRightMotor.getSelectedSensorPosition() * -1 / Constants.EncoderConstants.k_UNITS_P_REVOLUTION;
    avgRevolutionsTracked = (rawEncoderOutLeft + rawEncoderOutRight) / 2;
  }

  public boolean halfTurn() {
    turnSpeed = 0.5 * -1;
    if(averageDisplacement < Constants.EncoderConstants.HALF_TURN) {
      SmartDashboard.putNumber("Revolutions Tracked", avgRevolutionsTracked);
      differentialDriveSub.arcadeDrive(0, turnSpeed); 
      updateRevolutionsTracked();
      return true;
    }
    stop();
    return false;
  }

  public void driveWithJoystick() {
    driveSpeed = driveController.getY();
    turnSpeed = driveController.getZ() * 0.8;

    driveSpeed = filterLeft.calculate(driveController.getY() * -1);
    turnSpeed = filterRight.calculate(driveController.getZ()); //* -1);

    differentialDriveSub.arcadeDrive(driveSpeed, turnSpeed);

    // LEFT CLIMBER TRIGGER
    if(RobotContainer.buttonController.getLeftTriggerAxis() >= 0.5) {
      SmartDashboard.putNumber("left trigger", RobotContainer.buttonController.getLeftTriggerAxis());
      RobotContainer.climberSubsystem.climberDown();
    }
    // RIGHT CLIMBER TRIGGER
    if(RobotContainer.buttonController.getRightTriggerAxis() >= 0.5) {
      SmartDashboard.putNumber("right trigger", RobotContainer.buttonController.getRightTriggerAxis());
      RobotContainer.climberSubsystem.climberUp(); //sets speed to 0
    }
  }

  public void resetEncoders() {
    this.encLeftMotor.setSelectedSensorPosition(0);
    this.encRightMotor.setSelectedSensorPosition(0);
    averageDisplacement = 0;
  }

  public boolean encoderDrive() {
    driveSpeed = Constants.EncoderConstants.SPEED * -1;  

    SmartDashboard.putNumber("drive speed", driveSpeed);
    SmartDashboard.putNumber("turn rate", turnSpeed);
  
    if(averageDisplacement < Constants.EncoderConstants.TARGET_DISTANCE_FT) {
      SmartDashboard.putNumber("Average Displacement", averageDisplacement);
      // getCorrection(); // updates turn
      differentialDriveSub.tankDrive(Constants.EncoderConstants.SPEED, Constants.EncoderConstants.SPEED); 
      updateAverageDisplacement();
      return true;
    }
    stop();
    return false;
  }

  public void stop() {
    driveSpeed = 0;
    turnSpeed = 0;
    differentialDriveSub.arcadeDrive(driveSpeed, turnSpeed);
  }

  public void trackObject(double f, double rot) { // IN TANK DRIVE!
    if (rot > 0) {
      rot += Constants.LimelightConstants.LIMELIGHT_ROTATION_F;
    }
    else if (rot < 0) {
      rot -= Constants.LimelightConstants.LIMELIGHT_ROTATION_F;
    }
    
    if (Math.abs(LimeMath.tx) < .8) {
      alignedToHub = true;
    } else {
      alignedToHub = false;
    }
    
    differentialDriveSub.arcadeDrive(0, rot);
  }
  
  public boolean isAligned() {
    return alignedToHub;
  }

  public boolean isRepositioned() {
    return inPreferredPosition;
  }

  public double getLimeX() {
    return LimeMath.tx;
  }

  public void repositionBot(double f, double rot) {
    // Assumes bot is aligned, meaning that we only need to manipulate the bot's direction in the Y direction.
    double closestDistance = LimeMath.getClosestRelatedDistance(true);
    // if closestdisance is less than the distance, it means we go forward, opposite -> backward
    if (closestDistance < LimeMath.getDistanceFromHub()) {
      differentialDriveSub.arcadeDrive(f + Constants.LimelightConstants.LIMELIGHT_FOW_F, 0);
    } else {
      differentialDriveSub.arcadeDrive(f - Constants.LimelightConstants.LIMELIGHT_FOW_F, 0);
    }

    if (Math.abs(closestDistance - LimeMath.getDistanceFromHub()) < 5) {
      inPreferredPosition = true;
    }  else {
      inPreferredPosition = false;
    }
  }

  public double getLimeDistanceToHub() {
    return LimeMath.adjacent;
  }

  public double getClosestRelatedDistance() {
    return LimeMath.getClosestRelatedDistance(true);
  }

  public void trackObject() {
    double xOffset = -tx.getDouble(0.0);
    SmartDashboard.putNumber("xOffset", xOffset);
    double finalRot = 0.0;
    if (xOffset < .25) { //0.25 represents 1/4 of a degree as measured by the limelight, this prevents the robot from overshooting its turn
      finalRot = Constants.DriveConstants.ROT_MULTIPLIER * xOffset + Constants.DriveConstants.MIN_ROT_SPEED;
    }
    else if (xOffset > .25) {   // dampens the rotation at the end while turning
      finalRot = Constants.DriveConstants.ROT_MULTIPLIER * xOffset - Constants.DriveConstants.MIN_ROT_SPEED;
    }
    differentialDriveSub.tankDrive(finalRot, -finalRot);
  }

  public void autoDrive() {
    double sped = 0.4;
    differentialDriveSub.arcadeDrive(sped, 0);
    SmartDashboard.putNumber("tank drive", sped);
    SmartDashboard.putNumber("last call", Timer.getFPGATimestamp());
    /*
    RobotContainer.lowerLeftMotor.set(0.3);
    RobotContainer.upperLeftMotor.set(0.3);
    RobotContainer.lowerRightMotor.set(-0.3);
    RobotContainer.upperRightMotor.set(-0.3);
    */
    //Timer.delay(3);
    //RobotContainer.lowerLeftMotor.set(0);
    //RobotContainer.upperLeftMotor.set(0);
    //RobotContainer.lowerRightMotor.set(0);
    //RobotContainer.upperRightMotor.set(0);
  }
}
