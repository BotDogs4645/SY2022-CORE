package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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

  private WPI_TalonFX encLeftMotor;
  private WPI_TalonFX encRightMotor;

  private double rawEncoderOutLeft;
  private double rawEncoderOutRight;

  private double leftDistanceTraveled;
  private double rightDistanceTraveled;

  private double averageDisplacement;

  private static double circumferenceEquation = 2 * Math.PI;

  // initialize Drive subsystem
  public DriveTrain(MotorControllerGroup leftMotors, MotorControllerGroup rightMotors, XboxController driveController) {
    this.driveController = driveController;
    this.leftMotors = leftMotors;
    this.rightMotors= rightMotors;
    averageDisplacement = 0;
    differentialDriveSub = new DifferentialDrive(leftMotors, rightMotors);
    differentialDriveSub.setMaxOutput(Constants.driveConstants.maxOutput);
  }

  public void driveWithJoystick() {
    leftSpeed = driveController.getLeftY();
    rightSpeed = driveController.getRightY();

    SmartDashboard.putNumber("Left Speed", leftMotors.get());
    SmartDashboard.putNumber("Right Speed", rightMotors.get());
    
    differentialDriveSub.tankDrive(leftSpeed, rightSpeed);
  }

  public static double getDistance(WPI_TalonFX encMotor, double rawEncoderOut) {
    double mRevolutionsConversion = rawEncoderOut / Constants.encoderConstants.kUnitsPerRevolution;
    double mRevolutionsPerFoot = 12 / (Constants.encoderConstants.wheelRadiusInches * circumferenceEquation); // inches per foot / circumference
    return mRevolutionsConversion / mRevolutionsPerFoot;
  }

  public void getAverageDisplacement(WPI_TalonFX encLeftMotor, WPI_TalonFX encRightMotor) {
    this.encLeftMotor = encLeftMotor;
    this.encRightMotor = encRightMotor;

    rawEncoderOutLeft = encLeftMotor.getSelectedSensorPosition();
    rawEncoderOutRight = encRightMotor.getSelectedSensorPosition();

    leftDistanceTraveled = getDistance(encLeftMotor, rawEncoderOutLeft);
    rightDistanceTraveled = getDistance(encRightMotor, rawEncoderOutRight);

    averageDisplacement = (leftDistanceTraveled + rightDistanceTraveled) / 2;
  }

  public void encoderDrive() { // drives 18 ft
    leftSpeed = 0.2;
    rightSpeed = 0.2;
    while(averageDisplacement < 18) { // 18 FT; change to constant; should be continously updating
      differentialDriveSub.tankDrive(leftSpeed, rightSpeed);
      getAverageDisplacement(encLeftMotor, encRightMotor); // update avgDisplacement
    }
    stop();
    SmartDashboard.putNumber("Average Distance Traveled", averageDisplacement);
  }

  public void stop() {
    leftSpeed = 0;
    rightSpeed = 0;
    differentialDriveSub.tankDrive(leftSpeed, rightSpeed);
  }
}
