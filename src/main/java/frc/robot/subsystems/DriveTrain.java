package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

  private final DifferentialDrive differentialDriveSub;

  private XboxController driveController;

  private double leftSpeed;
  private double rightSpeed;

  private MotorControllerGroup leftMotors;
  private MotorControllerGroup rightMotors;

  // encoder variables:
  private WPI_TalonFX encLeftMotor;
  private WPI_TalonFX encRightMotor;

  private double rawEncoderOutLeft;
  private double rawEncoderOutRight;

  // initialize Drive subsystem
  public DriveTrain(MotorControllerGroup leftMotors, MotorControllerGroup rightMotors, XboxController driveController, MotorController encLeftMotor, MotorController encRightMotor) {
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

    differentialDriveSub = new DifferentialDrive(leftMotors, rightMotors);

    differentialDriveSub.setMaxOutput(Constants.driveConstants.MAX_OUTPUT);
  }

  public double getAverageDisplacement() { // still needs to account for margin of error
    rawEncoderOutLeft = encLeftMotor.getSelectedSensorPosition();
    rawEncoderOutRight = encRightMotor.getSelectedSensorPosition() * -1;

    double leftDistanceTraveled = rawEncoderOutLeft / (Constants.encoderConstants.k_UNITS_PREVOLUTION * Constants.encoderConstants.REVOLUTION_PFT);
    double rightDistanceTraveled = rawEncoderOutRight / (Constants.encoderConstants.k_UNITS_PREVOLUTION * Constants.encoderConstants.REVOLUTION_PFT);
    
    return (leftDistanceTraveled + rightDistanceTraveled) / 2; // returns average displacement
  }

  public void driveWithJoystick() {
    leftSpeed = driveController.getLeftY();
    rightSpeed = driveController.getRightY();

    SmartDashboard.putNumber("Left Speed", leftMotors.get());
    SmartDashboard.putNumber("Right Speed", rightMotors.get());

    differentialDriveSub.tankDrive(leftSpeed, rightSpeed);
  }

  public boolean encoderDrive() {
    leftSpeed = Constants.encoderConstants.LEFT_SPEED * -1;
    rightSpeed = Constants.encoderConstants.RIGHT_SPEED * -1;
   
    if(getAverageDisplacement() < Constants.encoderConstants.TARGET_DISTANCEFT) {
      differentialDriveSub.tankDrive(leftSpeed, rightSpeed);
      SmartDashboard.putNumber("Average Displacement", getAverageDisplacement());
    }

    return getAverageDisplacement() < Constants.encoderConstants.TARGET_DISTANCEFT; 
  }

  public void stop() {
    leftSpeed = 0;
    rightSpeed = 0;
    differentialDriveSub.tankDrive(leftSpeed, rightSpeed);
  }
}
