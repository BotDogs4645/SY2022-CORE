package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;


public class Drive extends CommandBase {

  private DriveTrain driveTrainSubsystem;

  private NetworkTableEntry ledMode = NetworkTableInstance.getDefault().getTable("limelight-console").getEntry("ledMode");

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

    if(DriveTrain.driveMode == Constants.gamepadButtons.LIMELIGHT_DRIVE) { // 1
      ledMode.setNumber(1);
      driveTrainSubsystem.trackObject();
    }
    else if(DriveTrain.driveMode == Constants.gamepadButtons.ENCODER_DRIVE) { // 2 | while encoders have not yet reached target distance and need to continue measuring...
      driveTrainSubsystem.encoderDrive();
      ledMode.setNumber(0);
      if (driveTrainSubsystem.averageDisplacement >= Constants.encoderConstants.TARGET_DISTANCEFT)
      {
        ledMode.setNumber(1);
        driveTrainSubsystem.trackObject();
      }
  }
    else { 
      DriveTrain.driveMode = Constants.gamepadButtons.JOYSTICK_DRIVE; // once target has been reached, toggle to manual
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
