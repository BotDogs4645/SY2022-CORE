package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GripPipeline;

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
  public static double averageDisplacement;

  private GripPipeline pipe;
  private VisionThread VisionThread;
  public static boolean alignedToHub;
  private Object foundTarget = new Object();

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
    driveMode = Constants.GamepadButtons.JOYSTICK_DRIVE;

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
  }
   
  public void updateAverageDisplacement() {
    rawEncoderOutLeft = encLeftMotor.getSelectedSensorPosition();
    rawEncoderOutRight = encRightMotor.getSelectedSensorPosition() * -1;
    
    leftDistanceTraveled = rawEncoderOutLeft / (Constants.EncoderConstants.k_UNITS_P_REVOLUTION * Constants.EncoderConstants.REVOLUTION_P_FT);
    rightDistanceTraveled = rawEncoderOutRight / (Constants.EncoderConstants.k_UNITS_P_REVOLUTION * Constants.EncoderConstants.REVOLUTION_P_FT);

    averageDisplacement = (leftDistanceTraveled + rightDistanceTraveled) / 2; // updates average displacement
  }

  public void driveWithJoystick() {
    driveSpeed = driveController.getY();
    turnSpeed = driveController.getZ();

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

  public void trackObject() { // IN TANK DRIVE!
    double xOffset = -tx.getDouble(0.0);
    SmartDashboard.putNumber("xOffset", xOffset);
    double finalRot = 0.0;

    if (xOffset < 1.5) { // 1.5 represents 3/2 of a degree as measured by the limelight, this prevents the robot from overshooting its turn
      finalRot = Constants.DriveConstants.ROT_MULTIPLIER * xOffset + Constants.DriveConstants.MIN_ROT_SPEED;
    }
    else if (xOffset > 1.5) {   // dampens the rotation at the end while turning
      finalRot = Constants.DriveConstants.ROT_MULTIPLIER * xOffset - Constants.DriveConstants.MIN_ROT_SPEED;
    }

    if (Math.abs(xOffset) < 3) {
      alignedToHub = true;
    }

    differentialDriveSub.tankDrive(finalRot, -finalRot);
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
}
