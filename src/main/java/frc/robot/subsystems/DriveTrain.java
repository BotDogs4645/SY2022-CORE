package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class DriveTrain extends SubsystemBase {

  private final DifferentialDrive differentialDrive;

  private Joystick leftJoystick;
  private Joystick rightJoystick;

  private double leftSpeed;
  private double rightSpeed;

  // initialize Drive subsystem
  public DriveTrain(MotorControllerGroup leftMotors, MotorControllerGroup rightMotors, Joystick leftJoystick, Joystick rightJoystick) {
    this.leftJoystick = leftJoystick;
    this.rightJoystick = rightJoystick;
    differentialDrive = new DifferentialDrive(leftMotors, rightMotors);
    differentialDrive.setMaxOutput(Constants.DriveConstants.maxOutput);
  }

  public void driveWithJoystick() {
    leftSpeed = leftJoystick.getY();
    rightSpeed = rightJoystick.getY();
    differentialDrive.tankDrive(leftSpeed, rightSpeed); // should never exceed 0.5 bc of max output
  }

  public void stop() {
    leftSpeed = 0;
    rightSpeed = 0;
    differentialDrive.tankDrive(leftSpeed, rightSpeed);
  }
}
