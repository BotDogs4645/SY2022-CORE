// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;

public class Auto extends CommandBase {
  private DriveTrain driveSub;
  private Indexer indexerSub;

  private int iterations = 0;

  private boolean armDown;
  
  public Auto(DriveTrain driveSub, Indexer indexerSub) {
    this.driveSub = driveSub;
    addRequirements(this.driveSub);
    addRequirements(this.indexerSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armDown = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("upper left", RobotContainer.upperLeftMotor.get());
    SmartDashboard.putNumber("avg displacement", RobotContainer.driveSubsystem.averageDisplacement);
    SmartDashboard.putBoolean("bool", driveSub.averageDisplacement < Constants.EncoderConstants.TARGET_DISTANCE_FT);
    SmartDashboard.putNumber("intake", RobotContainer.indexerSubsystem.intakeMotor.get());
    /*
    if(!armDown) { // if arm down is not true:
      indexerSub.lowerIntake();
      armDown = true;
    }
    indexerSub.absorb();
    */
    if(driveSub.averageDisplacement < Constants.EncoderConstants.TARGET_DISTANCE_FT) {
      this.driveSub.encoderDrive();
    }
    else {
      driveSub.stop();
      //driveSub.halfTurn();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSub.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
