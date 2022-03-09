package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

  public static int driveMode;
  public double averageDisplacement;

  private XboxController driveController;
  private final DifferentialDrive differentialDriveSub;

  private double leftSpeed;
  private double rightSpeed;

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

  private double error = 0;
  private double prev_error = 0;
  private double integral = 1;
  private double derivative = 0;
  private double turn = 0;
  private double idealHeading;

  private final AHRS ahrs = new AHRS();
  
  private final PIDController drivePID = new PIDController(Constants.EncoderConstants.kP, Constants.EncoderConstants.kI, Constants.EncoderConstants.kD);
  
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-console");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry ty = table.getEntry("ty");

  public DriveTrain(MotorControllerGroup leftMotors, MotorControllerGroup rightMotors, XboxController driveController, MotorController encLeftMotor, MotorController encRightMotor) {
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
    idealHeading = ahrs.getYaw(); // sets starting position as "0"
    drivePID.setTolerance(1);
    drivePID.setSetpoint(idealHeading); // sets setpoint to initial heading
  }

  public void updateAverageDisplacement() {
    rawEncoderOutLeft = encLeftMotor.getSelectedSensorPosition();
    rawEncoderOutRight = encRightMotor.getSelectedSensorPosition() * -1;

    leftDistanceTraveled = rawEncoderOutLeft / (Constants.EncoderConstants.k_UNITS_P_REVOLUTION * Constants.EncoderConstants.REVOLUTION_P_FT);
    SmartDashboard.putNumber("Distance Traveled Left", leftDistanceTraveled);

    rightDistanceTraveled = rawEncoderOutRight / (Constants.EncoderConstants.k_UNITS_P_REVOLUTION * Constants.EncoderConstants.REVOLUTION_P_FT);

    averageDisplacement = (leftDistanceTraveled + rightDistanceTraveled) / 2; // updates average displacement
  }

  public void driveWithJoystick() {
    leftSpeed = filterLeft.calculate(driveController.getLeftY() * -1);
    rightSpeed = filterRight.calculate(driveController.getRightY() * -1);

    SmartDashboard.putNumber("Teleop Speed", leftSpeed);

    differentialDriveSub.tankDrive(leftSpeed, rightSpeed);
  }

  public void resetEncoders() {
    this.encLeftMotor.setSelectedSensorPosition(0);
    this.encRightMotor.setSelectedSensorPosition(0);
    averageDisplacement = 0;
  }

  /* NOT USED */
  public void getCorrection() {
    SmartDashboard.putNumber("Start Heading", idealHeading);
    SmartDashboard.putNumber("Actual Heading", ahrs.getYaw());
    error = (ahrs.getYaw() - idealHeading);
    prev_error = error;
    integral += error * .02 * Constants.EncoderConstants.kI; // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
    derivative = ((error - prev_error) / .02) * Constants.EncoderConstants.kD;
    turn = error * Constants.EncoderConstants.kP * -1;

    SmartDashboard.putNumber("Error", error);
    SmartDashboard.putNumber("Integral", integral);
    SmartDashboard.putNumber("Derivative", derivative);
    SmartDashboard.putNumber("Adjusted Right Side Speed with kP", rightSpeed + turn); // right side is underturning, so only adjust R motors
    SmartDashboard.putNumber("Adjusted Right Side Speed with kP, kI", rightSpeed + (turn + integral));
    SmartDashboard.putNumber("Adjusted Right Side Speed with kP, kI, kD", rightSpeed + ( turn + integral + derivative) * -1);
  }

  public boolean encoderDrive() {
    leftSpeed = Constants.EncoderConstants.LEFT_SPEED;
    rightSpeed = Constants.EncoderConstants.RIGHT_SPEED;

    if(averageDisplacement < Constants.EncoderConstants.TARGET_DISTANCE_FT) {
      SmartDashboard.putNumber("Average Displacement", averageDisplacement);
      getCorrection(); // updates turn
      SmartDashboard.putNumber("left speed", leftSpeed); 
      SmartDashboard.putNumber("right speed", rightSpeed);
      differentialDriveSub.tankDrive(leftSpeed, rightSpeed);
      updateAverageDisplacement();
      return true;
    }
    stop();
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
      finalRot = Constants.DriveConstants.ROT_MULTIPLIER * xOffset + Constants.DriveConstants.MIN_ROT_SPEED;
    }
    else if (xOffset > .25) {   // dampens the rotation at the end while turning
      finalRot = Constants.DriveConstants.ROT_MULTIPLIER * xOffset - Constants.DriveConstants.MIN_ROT_SPEED;
    }
    differentialDriveSub.tankDrive(finalRot, -finalRot);
  }

  public double getDistance() {
    double yOffset = ty.getDouble(0.0);
    double radians = Math.toRadians(yOffset);
    double distance;
    
    distance = ((Constants.LimelightConstants.LIMELIGHT_HEIGHT - Constants.GameConstants.GOAL_HEIGHT)/Math.tan(radians))/12;
    SmartDashboard.putNumber("Distance to Target", distance);
    return distance;
  }
}
