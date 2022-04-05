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

  SlewRateLimiter filterLeft = new SlewRateLimiter(2);
  SlewRateLimiter filterRight = new SlewRateLimiter(2);

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
  public boolean resetHalfTurn = true;
  public double avgRevolutionsTracked;

  public boolean testingMode = true;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-console");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry ty = table.getEntry("ty");

  private final AHRS ahrs = new AHRS();
  private final DifferentialDrive differentialDriveSub;
  private final PIDController drivePID = new PIDController(Constants.EncoderConstants.kP, Constants.EncoderConstants.kI, Constants.EncoderConstants.kD);
  
  public DriveTrain(MotorControllerGroup leftMotors, MotorControllerGroup rightMotors, Joystick driveController, MotorController encLeftMotor, MotorController encRightMotor) {
    this.driveController = driveController;

    driveMode = Constants.DriveConstants.JOYSTICK_DRIVE;
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
    turnSpeed = driveController.getZ();

    // set limit on speed, increase slew rate limiter, fix vertical intake button, make intake whileHeld() and reset them 

    driveSpeed = filterLeft.calculate(driveController.getY() * -1);
    turnSpeed = filterRight.calculate(driveController.getZ() * -1);

    differentialDriveSub.arcadeDrive(driveSpeed, turnSpeed);
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
      differentialDriveSub.arcadeDrive(driveSpeed, 0); 
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
    return LimeMath.hypotenuse;
  }

  public double getClosestRelatedDistance() {
    return LimeMath.getClosestRelatedDistance(true);
  }

  // grip declarations
  public void declareGrip() {
    UsbCamera cam = new UsbCamera("GRIP Cam","/dev/video0");
    cam.setResolution(480, 720);
    VisionThread = new VisionThread(cam, new GripPipeline(), pipeline -> {
    synchronized(foundTarget) {
      pipe = pipeline;
    }
    });
  }

  public void orientWithGrip() {
    synchronized (foundTarget) {
      Rect r = Imgproc.boundingRect(pipe.findBlobsOutput());
      double finalRot = 0.0;
      double centerX = r.x;

      if (centerX < .25) { //0.25 represents 1/4 of a degree as measured by the limelight, this prevents the robot from overshooting its turn
        finalRot = Constants.DriveConstants.ROT_MULTIPLIER * centerX + Constants.DriveConstants.MIN_ROT_SPEED;
      }
      else if (centerX > .25) {   // dampens the rotation at the end while turning
        finalRot = Constants.DriveConstants.ROT_MULTIPLIER * centerX - Constants.DriveConstants.MIN_ROT_SPEED;
      }
      differentialDriveSub.tankDrive(finalRot, -finalRot);
    }
  }
  public void updateRevolutionsTracked() {
    if (resetHalfTurn == true) {
      resetEncoders();
      resetHalfTurn = false;
    }
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
}
