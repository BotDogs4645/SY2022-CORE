package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class Drive extends CommandBase {

  private DriveTrain driveTrainSubsystem;

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
    SmartDashboard.putNumber("DriveTrain driveMode is toggled to:", DriveTrain.driveMode);

    if(DriveTrain.driveMode == Constants.driveModeConstants.LIMELIGHT_DRIVE) { // 1
      // limelight code
    }
    else if(DriveTrain.driveMode == Constants.driveModeConstants.ENCODER_DRIVE) { // 2
      if(driveTrainSubsystem.encoderDrive() == true) { // while encoders have not yet reached target distance and need to continue measuring...
        driveTrainSubsystem.encoderDrive();
      }
      else {
        DriveTrain.driveMode = Constants.driveModeConstants.JOYSTICK_DRIVE; // once target has been reached, toggle to manual
        driveTrainSubsystem.resetEncoders();
      }
    }
    else { // 0
      SmartDashboard.putNumber("DriveTrain driveMode has been changed to: ", DriveTrain.driveMode);
      driveTrainSubsystem.driveWithJoystick();
      driveTrainSubsystem.resetEncoders();
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
    

