package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;


public class Drive extends CommandBase {

  private DriveTrain driveTrainSubsystem;

  private NetworkTableEntry ledMode = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode");

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
      ledMode.setNumber(0);
      DriveTrain.driveMode = Constants.gamepadButtons.JOYSTICK_DRIVE;
    }

    else if(DriveTrain.driveMode == Constants.gamepadButtons.ENCODER_DRIVE) { // 2
      if(driveTrainSubsystem.encoderDrive() == true) { // while encoders have not yet reached target distance and need to continue measuring...
        driveTrainSubsystem.encoderDrive();
        ledMode.setNumber(0);
      }
      else {
        DriveTrain.driveMode = Constants.gamepadButtons.JOYSTICK_DRIVE; // once target has been reached, toggle to manual
        driveTrainSubsystem.resetEncoders();
        ledMode.setNumber(0);
      }
    }

    else { // 0
      driveTrainSubsystem.driveWithJoystick();
      driveTrainSubsystem.resetEncoders();
      ledMode.setNumber(0); // turn off limelight
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
