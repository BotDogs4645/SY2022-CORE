package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  private WPI_TalonFX verticalIndexerMotor;
  private WPI_TalonFX horizontalIndexerMotor;

  private CANSparkMax intakeMotor = new CANSparkMax(Constants.IndexerConstants.INTAKE_MOTOR, MotorType.kBrushed);

  private double indexerSpeed = 0.3; 
  private double intakeSpeed = 0.6;

  public Indexer(WPI_TalonFX verticalIndexerMotor, WPI_TalonFX horizontalIndexerMotor) {
    //Indexer splits path from intake to shooter into vertical and horizontal belt systems. 
    this.verticalIndexerMotor = verticalIndexerMotor;
    this.horizontalIndexerMotor = horizontalIndexerMotor;
  }

  /*enables intake and bottom indexer*/
  public void absorb() {
    SmartDashboard.putString("absorb?", "jas");
    intakeMotor.set(-intakeSpeed);
    horizontalIndexerMotor.set(indexerSpeed);
    verticalIndexerMotor.set(0); // don't want it to be running
  }

  /* reverses bottom indexer in case a ball gets stuck */
  public void unabsorb() {
    SmartDashboard.putString("absorb?", " not jas");
    intakeMotor.set(intakeSpeed);
    verticalIndexerMotor.set(-indexerSpeed);
    horizontalIndexerMotor.set(-indexerSpeed);
  }

  public void enableVerticalIndexer() {
    verticalIndexerMotor.set(indexerSpeed);
  }

  @Override
  public void periodic() {}
}
