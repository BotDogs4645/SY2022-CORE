package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  public static boolean upFlag;

  private XboxController buttonController;

  private CANSparkMax leftClimberMotor;
  private CANSparkMax rightClimberMotor;

  public Climber(CANSparkMax rightClimberMotor, CANSparkMax leftClimberMotor, XboxController buttonController) {
    this.leftClimberMotor = leftClimberMotor;
    this.rightClimberMotor = rightClimberMotor;
    this.buttonController = buttonController;
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
  public void periodic() {}
}
