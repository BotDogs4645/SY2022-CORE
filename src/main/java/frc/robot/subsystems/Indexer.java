package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  private WPI_TalonFX verticalIndexerMotor;
  private WPI_TalonSRX horizontalIndexerMotor;

  private WPI_TalonSRX intakeMotor = new WPI_TalonSRX(Constants.IndexerConstants.INTAKE_MOTOR);

  private double indexerSpeed = 0.3; 
  private double intakeSpeed = 0.5;

  public Indexer(WPI_TalonFX verticalIndexerMotor, WPI_TalonSRX horizontalIndexerMotor) {
    //Indexer splits path from intake to shooter into vertical and horizontal belt systems. 
    this.verticalIndexerMotor = verticalIndexerMotor;
    this.horizontalIndexerMotor = horizontalIndexerMotor;
    stop();
  }

  /*enables intake and bottom indexer*/
  public void absorb() {
    SmartDashboard.putString("absorb?", "jas");
    intakeMotor.set(intakeSpeed);
    horizontalIndexerMotor.set(indexerSpeed);
    verticalIndexerMotor.set(0); // don't want it to be running
  }

  /* reverses bottom indexer in case a ball gets stuck */
  public void unabsorb() {
    SmartDashboard.putString("absorb?", " not jas");
    intakeMotor.set(-intakeSpeed);
    verticalIndexerMotor.set(indexerSpeed);
    horizontalIndexerMotor.set(-indexerSpeed);
  }

  public void enableVerticalIndexer() {
    verticalIndexerMotor.set(-indexerSpeed);
  }

  public void stop() {
    verticalIndexerMotor.set(0);
    horizontalIndexerMotor.set(0);
  }

  @Override
  public void periodic() {}
}
