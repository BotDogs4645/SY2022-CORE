package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
    SmartDashboard.putBoolean("DriveTrain toggled to: ", driveTrainSubsystem.driveWithEncoders);

    if (driveTrainSubsystem.driveWithEncoders) { // if true
      driveTrainSubsystem.encoderDrive();
    }
    
    else { // if encoders are not enabled, drive manually
      driveTrainSubsystem.driveWithJoystick();
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
    