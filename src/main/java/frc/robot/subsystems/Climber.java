package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  public static boolean upFlag;
  private double climbStartTime;
  private double distance;

  private XboxController buttonController;
  private AHRS ahrs;

  private CANSparkMax leftClimberMotor;
  private CANSparkMax rightClimberMotor;

  public Climber(CANSparkMax rightClimberMotor, CANSparkMax leftClimberMotor, XboxController buttonController, AHRS ahrs) {
    this.leftClimberMotor = leftClimberMotor;
    this.rightClimberMotor = rightClimberMotor;
    this.buttonController = buttonController;
    this.ahrs = ahrs;
  }

  // method below is probably not needed, b/c the climbers are only long enough to extend to mid.
  public double calculateDistance() {
    distance = ahrs.getVelocityY() * ((Timer.getFPGATimestamp() - climbStartTime) / 1000);
    return distance;
  }

  public void climberToggle() {
    if(upFlag) {
      climbStartTime = Timer.getFPGATimestamp();
      latch();
    }
    else {
      climbStartTime = Timer.getFPGATimestamp();
      distance = 0;
      climberUp();
    }
  }

  public void climberUp() {
    rightClimberMotor.set(0);
    leftClimberMotor.set(0);
    upFlag = true;
  }
  // applies more torque & motor power than climberDown to lift the robot
  public void latch() {
    if(upFlag) {
      rightClimberMotor.set(0.7);
      leftClimberMotor.set(0.7);
    }
  }
  //brings the arm down slowly and keeps it set
  public void climberDown() {
    rightClimberMotor.set(0.25);
    leftClimberMotor.set(0.25);
  }

  @Override
  public void periodic() {}
}
