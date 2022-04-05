// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ShooterIntegratedPID;
import frc.robot.Constants;
// command below is the main auto profile, it moves back, intakes a bal;
public class AutoProfile1 extends CommandBase {
  /** Creates a new Auto1. */
  public AutoProfile1(DriveTrain driveSubsystem, ShooterIntegratedPID shooterSubsystem) {
    addRequirements(driveSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.driveSubsystem.encoderDrive();
    if (RobotContainer.driveSubsystem.averageDisplacement >= Constants.EncoderConstants.TARGET_DISTANCE_FT) { // stop running encoderDrive
      // INTAKE CODE! --> while its moving
      RobotContainer.driveSubsystem.halfTurn();
      if(RobotContainer.driveSubsystem.avgRevolutionsTracked >= Constants.EncoderConstants.HALF_TURN) {
        new SequentialCommandGroup(
          new InstantCommand(RobotContainer.shooterSubsystem::limeOn, RobotContainer.shooterSubsystem),
          new LimelightAlignToLower(RobotContainer.driveSubsystem),
          new ToClosestPlottedPosition(RobotContainer.driveSubsystem)
        );
        RobotContainer.driveSubsystem.stop();
        
        RobotContainer.shooterSubsystem.setVelocity(3500);
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
