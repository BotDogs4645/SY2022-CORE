package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  public static boolean upFlag;
  private double climbStartTime;
  private double distance;

  private XboxController buttonController;
  private AHRS ahrs;

  private WPI_TalonSRX leftClimberMotor;
  private WPI_TalonSRX rightClimberMotor;


  public Climber(WPI_TalonSRX rightClimberMotor, WPI_TalonSRX leftClimberMotor, XboxController buttonController, AHRS ahrs) {
    this.leftClimberMotor = leftClimberMotor;
    this.rightClimberMotor = rightClimberMotor;
    this.buttonController = buttonController;
    this.ahrs = ahrs;
  }

  public void climberDown() {
    SmartDashboard.putString("climber:", "down");
    rightClimberMotor.set(-0.5);
    leftClimberMotor.set(-0.5);
    if (rightClimberMotor.getBusVoltage() >= 30 && leftClimberMotor.getBusVoltage() >= 30) {
      rightClimberMotor.set(0);
      leftClimberMotor.set(0);
      upFlag = false;
    }
  }

  public void climberUp() {
    SmartDashboard.putString("climber:", "up");
    rightClimberMotor.set(0.5);
    leftClimberMotor.set(0.5);
    if (rightClimberMotor.getBusVoltage() >= 30 && leftClimberMotor.getBusVoltage() >= 30) {
      rightClimberMotor.set(0);
      leftClimberMotor.set(0);
      upFlag = true;
    }
  }


  @Override
  public void periodic() {}
}
