package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private WPI_TalonFX verticalIndexerMotor;
  private WPI_TalonFX horizontalIndexerMotor;

  private double speed = 0.3; 

  public Indexer(WPI_TalonFX verticalIndexerMotor, WPI_TalonFX horizontalIndexerMotor) {
    //Indexer splits path from intake to shooter into vertical and horizontal belt systems. 
    this.verticalIndexerMotor = verticalIndexerMotor;
    this.horizontalIndexerMotor = horizontalIndexerMotor;

  }
  public void indexCargo() {
    verticalIndexerMotor.set(speed);
    horizontalIndexerMotor.set(speed);
  }

  @Override
  public void periodic() {}
}
