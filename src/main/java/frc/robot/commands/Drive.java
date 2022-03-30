package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;

public class Drive extends CommandBase {

  private DriveTrain driveTrainSubsystem;
  private Indexer indexerSubsystem;

  private NetworkTableEntry ledMode = NetworkTableInstance.getDefault().getTable("limelight-console").getEntry("ledMode");

  public Drive(DriveTrain subsystem, Indexer indexerSubsystem) {
    driveTrainSubsystem = subsystem;
    this.indexerSubsystem = indexerSubsystem;
    addRequirements(driveTrainSubsystem);
    addRequirements(indexerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("DriveTrain driveMode is toggled to:", DriveTrain.driveMode);
    
    //indexer constantly running
    indexerSubsystem.indexCargo();

    if(DriveTrain.driveMode == Constants.DriveModes.LIMELIGHT_DRIVE) { // HMM
      ledMode.setNumber(1);
      driveTrainSubsystem.trackObject();
    }
    else if(DriveTrain.driveMode == Constants.DriveModes.ENCODER_DRIVE) { // 2 | while encoders have not yet reached target distance and need to continue measuring...
      driveTrainSubsystem.encoderDrive();
      ledMode.setNumber(0);
      if (driveTrainSubsystem.averageDisplacement >= Constants.EncoderConstants.TARGET_DISTANCE_FT)
      {
        ledMode.setNumber(1);
        driveTrainSubsystem.trackObject();
        DriveTrain.driveMode = Constants.DriveModes.JOYSTICK_DRIVE;
      }
    }
    else { 
      DriveTrain.driveMode = Constants.DriveModes.JOYSTICK_DRIVE; // once target has been reached, toggle to manual
      driveTrainSubsystem.resetEncoders();
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
