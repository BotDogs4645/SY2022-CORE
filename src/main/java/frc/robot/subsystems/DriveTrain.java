package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class DriveTrain extends SubsystemBase {

  private final DifferentialDrive differentialDriveSub;

  private Joystick driveJoystick;

  private double forward;
  private double rotation;

  // initialize Drive subsystem
  public DriveTrain(MotorControllerGroup leftMotors, MotorControllerGroup rightMotors, Joystick driveJoystick) {
    this.driveJoystick = driveJoystick;
    differentialDriveSub = new DifferentialDrive(leftMotors, rightMotors);
    differentialDriveSub.setMaxOutput(Constants.DriveConstants.maxOutput);
  }

  public void driveWithJoystick() {
    forward = driveJoystick.getY();
    rotation = driveJoystick.getX();
    
    differentialDriveSub.arcadeDrive(forward, -rotation); // should never exceed 0.5 bc of max output
  }

  public void stop() {
    forward = 0;
    rotation = 0;
    differentialDriveSub.arcadeDrive(forward, -rotation);
  }
}
