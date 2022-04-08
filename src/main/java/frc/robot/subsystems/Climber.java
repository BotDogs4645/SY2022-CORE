package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  public static boolean upFlag;
  
  private XboxController buttonController;

  private WPI_TalonSRX leftClimberMotor;
  private WPI_TalonSRX rightClimberMotor;

  public Climber(WPI_TalonSRX rightClimberMotor, WPI_TalonSRX leftClimberMotor, XboxController buttonController) {
    this.leftClimberMotor = leftClimberMotor;
    this.rightClimberMotor = rightClimberMotor;
    this.buttonController = buttonController;
  }

  public void climberDown() {
    SmartDashboard.putString("climber:", "down");
    leftClimberMotor.set(-0.25);
    rightClimberMotor.set(-0.25);
  }

  public void climberUp() {
    SmartDashboard.putString("climber:", "up");
    leftClimberMotor.set(0.12);
    rightClimberMotor.set(0.12);
  }

  @Override
  public void periodic() {}
}
