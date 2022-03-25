// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private CANSparkMax rightClimberMotor;
  private CANSparkMax leftClimberMotor;
  
  private Joystick driveController;

  public static boolean upFlag;

  /** Creates a new Climber. */
  public Climber(CANSparkMax rightClimberMotor, CANSparkMax leftClimberMotor, Joystick driveController) {
    this.rightClimberMotor = rightClimberMotor;
    this.rightClimberMotor = rightClimberMotor;
    this.driveController = driveController;
    }

  public void climberDown() {
    rightClimberMotor.set(-0.5);
    leftClimberMotor.set(-0.5);
    if (rightClimberMotor.getBusVoltage() >= 30 && leftClimberMotor.getBusVoltage() >= 30) {
      rightClimberMotor.set(0);
      leftClimberMotor.set(0);
      upFlag = false;
    }
  }

  public void climberUp() {
    rightClimberMotor.set(0.5);
    leftClimberMotor.set(0.5);
    if (rightClimberMotor.getBusVoltage() >= 30 && leftClimberMotor.getBusVoltage() >= 30) {
      rightClimberMotor.set(0);
      leftClimberMotor.set(0);
      upFlag = true;
    }
  }

  public boolean getUpFlag() {
    return upFlag;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
