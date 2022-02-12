package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class Drive extends CommandBase {

  private DriveTrain driveTrainSubsystem;

  private int driveMode;

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
    if(driveTrainSubsystem.driveMode == Constants.driveModeConstants.LIMELIGHT_DRIVE) {
      // limelight code
    }
    else if(driveTrainSubsystem.driveMode == Constants.driveModeConstants.ENCODER_DRIVE){
      if(driveTrainSubsystem.encoderDrive() == true) { // while encoders have not yet reached target distance and need to continue measuring...
        driveTrainSubsystem.encoderDrive();
      }
      else {
        driveMode = Constants.driveModeConstants.JOYSTICK_DRIVE; // once target has been reached, toggle to manual
      }
    }
    else {
      driveMode = Constants.driveModeConstants.JOYSTICK_DRIVE; // default drive mode is manual joystick (driveMode == 0)
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
    

