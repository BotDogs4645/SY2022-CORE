package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GripPipeline;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import java.nio.channels.Pipe;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.vision.VisionPipeline;

public class DriveTrain extends SubsystemBase {

  public static int driveMode;
  public double averageDisplacement;

  private XboxController driveController;
  private final DifferentialDrive differentialDriveSub;

  private double leftSpeed;
  private double rightSpeed;

  private double leftDistanceTraveled;
  private double rightDistanceTraveled;

  private MotorControllerGroup leftMotors;
  private MotorControllerGroup rightMotors;

  private WPI_TalonFX encLeftMotor;
  private WPI_TalonFX encRightMotor;

  private double rawEncoderOutLeft;
  private double rawEncoderOutRight;

  private double error = 0;
  private double heading = 0; 
  private double turnPower = 0;

  private VisionThread VisionThread;

  private GripPipeline pipe;
  private Object foundTarget = new Object();

  private final AHRS ahrs = new AHRS();

  private final PIDController drivePID = new PIDController(Constants.encoderConstants.kP, Constants.encoderConstants.kI, Constants.encoderConstants.kD);
  
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-console");

  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry ty = table.getEntry("ty");

  public DriveTrain(MotorControllerGroup leftMotors, MotorControllerGroup rightMotors, XboxController driveController, MotorController encLeftMotor, MotorController encRightMotor) {
    this.driveController = driveController;
    driveMode = Constants.gamepadButtons.JOYSTICK_DRIVE;

    this.leftMotors = leftMotors;
    this.rightMotors = rightMotors;

    this.encLeftMotor = (WPI_TalonFX) encLeftMotor;
    this.encRightMotor = (WPI_TalonFX) encRightMotor;

    resetEncoders();
    this.encLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    this.encRightMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    differentialDriveSub = new DifferentialDrive(leftMotors, rightMotors);
    differentialDriveSub.setMaxOutput(Constants.driveConstants.MAX_OUTPUT);

    ahrs.reset(); 
    heading = Constants.encoderConstants.FLYWHEEL_RPM * ahrs.getAngle(); // incorporates flywheel feedforward
    drivePID.setTolerance(Constants.encoderConstants.ENCODER_TOLERANCE);
    drivePID.setSetpoint(heading); // sets setpoint to initial heading
  }

  public void updateAverageDisplacement() { // still needs to account for margin of error
    rawEncoderOutLeft = encLeftMotor.getSelectedSensorPosition();
    rawEncoderOutRight = encRightMotor.getSelectedSensorPosition() * -1;

    leftDistanceTraveled = rawEncoderOutLeft / (Constants.encoderConstants.k_UNITS_P_REVOLUTION * Constants.encoderConstants.REVOLUTION_P_FT);
    rightDistanceTraveled = rawEncoderOutRight / (Constants.encoderConstants.k_UNITS_P_REVOLUTION * Constants.encoderConstants.REVOLUTION_P_FT);

    averageDisplacement = (leftDistanceTraveled + rightDistanceTraveled) / 2; // updates average displacement
  }

  public void driveWithJoystick() {
    leftSpeed = driveController.getLeftY();
    rightSpeed = driveController.getRightY();

    SmartDashboard.putNumber("Left Speed", leftMotors.get());
    SmartDashboard.putNumber("Right Speed", rightMotors.get());

    differentialDriveSub.tankDrive(leftSpeed, rightSpeed);
  }

  public void resetEncoders() {
    this.encLeftMotor.setSelectedSensorPosition(0);
    this.encRightMotor.setSelectedSensorPosition(0);
    averageDisplacement = 0;
  }

  public void getCorrection() {
    error = heading - ahrs.getAngle();
    turnPower = error * Constants.encoderConstants.kP;
  }

  public boolean encoderDrive() {
    leftSpeed = Constants.encoderConstants.LEFT_SPEED * -1;
    rightSpeed = Constants.encoderConstants.RIGHT_SPEED * -1;

    SmartDashboard.putNumber("leftiespeed", leftSpeed);
    SmartDashboard.putNumber("rightiespeed", rightSpeed);

    if(averageDisplacement < Constants.encoderConstants.TARGET_DISTANCEFT) {
      SmartDashboard.putNumber("Average Displacement", averageDisplacement);
      getCorrection(); // updates turnPower
      differentialDriveSub.tankDrive(leftSpeed * turnPower, rightSpeed * turnPower);
      updateAverageDisplacement();
      return true;
    }
    return false;
  }

  public void stop() {
    leftSpeed = 0;
    rightSpeed = 0;
    differentialDriveSub.tankDrive(leftSpeed, rightSpeed);
  }

  public void trackObject() {
    double xOffset = -tx.getDouble(0.0);
    SmartDashboard.putNumber("xOffset", xOffset);
    double finalRot = 0.0;

    if (xOffset < .25) { //0.25 represents 1/4 of a degree as measured by the limelight, this prevents the robot from overshooting its turn
      finalRot = Constants.driveConstants.ROT_MULTIPLIER * xOffset + Constants.driveConstants.MIN_ROT_SPEED;
    }
    else if (xOffset > .25) {   // dampens the rotation at the end while turning
      finalRot = Constants.driveConstants.ROT_MULTIPLIER * xOffset - Constants.driveConstants.MIN_ROT_SPEED;
    }
    differentialDriveSub.tankDrive(finalRot, -finalRot);
  }

  public double getDistance() {
    double yOffset = ty.getDouble(0.0);
    double radians = Math.toRadians(yOffset);
    double distance;
    
    distance = ((Constants.limelightConstants.LIMELIGHT_HEIGHT - Constants.gameConstants.GOAL_HEIGHT)/Math.tan(radians))/12;
    SmartDashboard.putNumber("Distance to Target", distance);
    return distance;
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
        finalRot = Constants.driveConstants.ROT_MULTIPLIER * centerX + Constants.driveConstants.MIN_ROT_SPEED;
      }
      else if (centerX > .25) {   // dampens the rotation at the end while turning
        finalRot = Constants.driveConstants.ROT_MULTIPLIER * centerX - Constants.driveConstants.MIN_ROT_SPEED;
      }
      differentialDriveSub.tankDrive(finalRot, -finalRot);
    }
  }
}
