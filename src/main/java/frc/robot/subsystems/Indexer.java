package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
public class Indexer extends SubsystemBase {
  public WPI_TalonFX verticalIndexerMotor;
  public WPI_TalonSRX horizontalIndexerMotor;
  private WPI_TalonSRX raiseIntakeMotor = new WPI_TalonSRX(Constants.IndexerConstants.RAISE_INTAKE_MOTOR);


  public WPI_TalonSRX intakeMotor = new WPI_TalonSRX(Constants.IndexerConstants.INTAKE_MOTOR);

  private double indexerSpeed = 0.3; 
  private double intakeSpeed = 0.5;

  private double armSpeed = 0.5;

  public Indexer(WPI_TalonFX verticalIndexerMotor, WPI_TalonSRX horizontalIndexerMotor) {
    //Indexer splits path from intake to shooter into vertical and horizontal belt systems. 
    this.verticalIndexerMotor = verticalIndexerMotor;
    this.horizontalIndexerMotor = horizontalIndexerMotor;
    stopIndexer();
  }

  /*enables intake and bottom indexer*/
  public void absorb() {
    intakeMotor.set(intakeSpeed);
    horizontalIndexerMotor.set(indexerSpeed);
    verticalIndexerMotor.set(0); // don't want it to be running
  }

  /* reverses bottom indexer in case a ball gets stuck */
  public void unabsorb() {
    intakeMotor.set(-intakeSpeed);
    verticalIndexerMotor.set(indexerSpeed);
    horizontalIndexerMotor.set(-indexerSpeed);
  }

  public void enableVerticalIndexer() {
    SmartDashboard.putNumber("vertical indexer", -indexerSpeed);
    verticalIndexerMotor.set(-indexerSpeed);
  }

  public void lowerIntake() {
    SmartDashboard.putNumber("lower intake sped", armSpeed);
    
    if(RobotContainer.limitSwitchState) { // lower
      armSpeed = -0.25;
      raiseIntakeMotor.set(armSpeed);

    }
    raiseIntakeMotor.set(-0.12);
  }

  public void raiseIntake() {
    SmartDashboard.putNumber("raise intake sped", armSpeed);
    armSpeed = 0.4;
    raiseIntakeMotor.set(armSpeed);
    Timer.delay(0.5);
    armSpeed = 0.1;
    raiseIntakeMotor.set(armSpeed);
  }

  public void stopIndexer() {
    SmartDashboard.putNumber("vertical indexer", 0);
    verticalIndexerMotor.set(0);
    horizontalIndexerMotor.set(0);
    intakeMotor.set(0);
  }

  @Override
  public void periodic() {}
}
