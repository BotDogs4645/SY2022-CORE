package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import frc.robot.Constants;

public class ChangeDriveMode extends CommandBase {

  // DriveTrain subsystem to manipulate the DifferentialDrive
  private DriveTrain driveTrainSubsystem;
  private NetworkTableEntry ledMode = NetworkTableInstance.getDefault().getTable("limelight-console").getEntry("ledMode");

  // initialize Drive command
  private int driveMode;

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
    driveTrainSubsystem.driveMode = driveMode; // change driveMode varibale in subsystem
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
