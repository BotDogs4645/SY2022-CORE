package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveTrain;

public class ChangeDriveMode extends CommandBase {

  private int driveMode;

  private DriveTrain driveTrainSubsystem

  public ChangeDriveMode(DriveTrain subsystem, int driveMode) {
    driveTrainSubsystem = subsystem;
    this.driveMode = driveMode;
    addRequirements(driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override

  public void execute() {
    DriveTrain.driveMode = driveMode; // change driveMode varibale in subsystem
    CommandScheduler.getInstance().cancel(this);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
