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
import edu.wpi.first.networktables.NetworkTable;

public class DriveTrain extends SubsystemBase {

  private final DifferentialDrive differentialDriveSub;
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-console");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry ty = table.getEntry("ty");

  private XboxController driveController;

  public static int driveMode = 1;

  public final double rotMultiplier = -0.03;
  public final double minRotSpeed = .1;

  private double leftSpeed;
  private double rightSpeed;

  private MotorControllerGroup leftMotors;
  private MotorControllerGroup rightMotors;

  // initialize Drive subsystem
  public DriveTrain(MotorControllerGroup leftMotors, MotorControllerGroup rightMotors, XboxController driveController) {
    this.driveController = driveController;
    this.leftMotors = leftMotors;
    this.rightMotors = rightMotors;
    differentialDriveSub = new DifferentialDrive(leftMotors, rightMotors);
    differentialDriveSub.setMaxOutput(Constants.DriveConstants.maxOutput);
  }

  public void driveWithJoystick() {
    leftSpeed = driveController.getLeftY();
    rightSpeed = driveController.getRightY();

    SmartDashboard.putNumber("Left Speed", leftMotors.get());
    SmartDashboard.putNumber("Right Speed", rightMotors.get());
    
    differentialDriveSub.tankDrive(leftSpeed, rightSpeed); // should never exceed 0.5 bc of max output
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
      if (xOffset < 1.0) {
        //leftMotors.set(-.1);
        //rightMotors.set(.1);
        finalRot = rotMultiplier * xOffset + minRotSpeed;
      } else if (xOffset > 1.0) {
        //leftMotors.set(.1);
        //rightMotors.set(-.1);
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
