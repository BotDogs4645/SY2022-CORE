package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class Drive extends CommandBase {

  // DriveTrain subsystem to manipulate the DifferentialDrive
  private DriveTrain driveTrainSubsystem;

  // initialize Drive command
  public Drive(DriveTrain subsystem) {
    driveTrainSubsystem = subsystem;
    addRequirements(driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (DriveTrain.driveMode == 0) {
      //driveTrainSubsystem.driveWithJoystick();
    } else if (DriveTrain.driveMode == 1) {
      //driveTrainSubsystem.trackObject();
      //driveTrainSubsystem.getDistance();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
