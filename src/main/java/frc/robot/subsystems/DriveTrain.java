package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTable;

public class DriveTrain extends SubsystemBase {

  public static int driveMode;
  private double averageDisplacement;

  private XboxController driveController;
  private final DifferentialDrive differentialDriveSub;

  public final double rotMultiplier = -0.05;
  public final double minRotSpeed = .15;

  public double leftSpeed;
  public double rightSpeed;

  private MotorControllerGroup leftMotors;
  private MotorControllerGroup rightMotors;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-console");

  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry ty = table.getEntry("ty");

  // initialize Drive subsystem
  public DriveTrain(MotorControllerGroup leftMotors, MotorControllerGroup rightMotors, XboxController driveController) {
    this.driveController = driveController;
    this.leftMotors = leftMotors;
    this.rightMotors= rightMotors;

    this.encLeftMotor = (WPI_TalonFX) encLeftMotor;
    this.encRightMotor = (WPI_TalonFX) encRightMotor;

    this.encLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    this.encRightMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    //reset encoders
    this.encLeftMotor.setSelectedSensorPosition(0);
    this.encRightMotor.setSelectedSensorPosition(0);

    // reset displacement
    averageDisplacement = 0;

    driveMode = Constants.driveModeConstants.JOYSTICK_DRIVE;

    differentialDriveSub = new DifferentialDrive(leftMotors, rightMotors);

    differentialDriveSub.setMaxOutput(Constants.driveConstants.MAX_OUTPUT);
  }

  public void updateAverageDisplacement() { // still needs to account for margin of error
    rawEncoderOutLeft = encLeftMotor.getSelectedSensorPosition();
    rawEncoderOutRight = encRightMotor.getSelectedSensorPosition() * -1;

    double leftDistanceTraveled = rawEncoderOutLeft / (Constants.encoderConstants.k_UNITS_PREVOLUTION * Constants.encoderConstants.REVOLUTION_PFT);
    double rightDistanceTraveled = rawEncoderOutRight / (Constants.encoderConstants.k_UNITS_PREVOLUTION * Constants.encoderConstants.REVOLUTION_PFT);

    averageDisplacement = (leftDistanceTraveled + rightDistanceTraveled) / 2; // returns average displacement
  }

  public void driveWithJoystick() {
    leftSpeed = driveController.getLeftY();
    rightSpeed = driveController.getRightY();

    SmartDashboard.putNumber("Left Speed", leftMotors.get());
    SmartDashboard.putNumber("Right Speed", rightMotors.get());

    differentialDriveSub.tankDrive(leftSpeed, rightSpeed);
  }

  public void resetEncoders() {
    //reset encoders
    this.encLeftMotor.setSelectedSensorPosition(0);
    this.encRightMotor.setSelectedSensorPosition(0);

    // reset displacement
    averageDisplacement = 0;
  }

  public boolean encoderDrive() {
    leftSpeed = Constants.encoderConstants.LEFT_SPEED * -1;
    rightSpeed = Constants.encoderConstants.RIGHT_SPEED * -1;

    if(averageDisplacement < Constants.encoderConstants.TARGET_DISTANCEFT) {
      SmartDashboard.putNumber("Average Displacement", averageDisplacement);
      differentialDriveSub.tankDrive(leftSpeed, rightSpeed);
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
        finalRot = rotMultiplier * xOffset + minRotSpeed;
      }
      else if (xOffset > .25) {   // dampens the rotation at the end while turning
        finalRot = rotMultiplier * xOffset - minRotSpeed;
      }
      differentialDriveSub.tankDrive(finalRot, -finalRot);
    }

    public double getDistance() {
      double h1 = 3.25;
      double h2 = 57.5;
      double yOffset = ty.getDouble(0.0);
      double radians = Math.toRadians(yOffset);
      double distance = ((h2-h1)/Math.tan(radians))/12;
      SmartDashboard.putNumber("Distance to Target", distance);

      return distance;
    }
}
