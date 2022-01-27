// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DriveTrain extends SubsystemBase {

  private final DifferentialDrive differentialDrive;

  private Joystick leftJoystick;
  private Joystick rightJoystick;

  private double leftSpeed;
  private double rightSpeed;

  /** Creates a new DriveTrain. */
  public DriveTrain(MotorControllerGroup leftMotors, MotorControllerGroup rightMotors, Joystick leftJoystick, Joystick rightJoystick) {
    this.leftJoystick = leftJoystick;
    this.rightJoystick = rightJoystick;
    differentialDrive = new DifferentialDrive(leftMotors, rightMotors);
  }

  public void driveWithJoystick() {
    leftSpeed = leftJoystick.getY() * 0.3;
    rightSpeed = rightJoystick.getY() * 0.3;

    differentialDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void stop() {
    leftSpeed = 0;
    rightSpeed = 0;
    differentialDrive.tankDrive(leftSpeed, rightSpeed);
  }
}
