package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;


public class Drive extends CommandBase {

  private DriveTrain driveTrainSubsystem;

  private WPI_TalonFX encLeftMotor; 
  private WPI_TalonFX encRightMotor;

  private boolean driveEncoders;

  public Drive(DriveTrain subsystem, MotorController encLeftMotor, MotorController encRightMotor, boolean driveEncoders) {
    driveTrainSubsystem = subsystem;
    addRequirements(driveTrainSubsystem);
    this.encLeftMotor = (WPI_TalonFX) encLeftMotor;
    this.encRightMotor = (WPI_TalonFX) encRightMotor;
    this.driveEncoders = driveEncoders;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override

  public void execute() {
    if(driveEncoders) {
      driveTrainSubsystem.encoderDrive(encLeftMotor, encRightMotor);
    }
    else {
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
    