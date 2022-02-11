package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class Drive extends CommandBase {

  private DriveTrain driveTrainSubsystem;

  private int driveMode;

  public Drive(DriveTrain subsystem, int driveMode) {
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
    if(driveMode == Constants.driveModeConstants.JOYSTICK_DRIVE) {
      driveTrainSubsystem.driveWithJoystick();
    }
    else if(driveMode == Constants.driveModeConstants.LIMELIGHT_DRIVE) {
      // limelight
    }
    else if(driveMode == Constants.driveModeConstants.ENCODER_DRIVE){
      if(driveTrainSubsystem.encoderDrive() == false) { // if the encoders have not yet reached target distance
        driveTrainSubsystem.encoderDrive();
      }
    }
    else {
        driveMode = Constants.driveModeConstants.JOYSTICK_DRIVE; // toggle back to manual
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
    

