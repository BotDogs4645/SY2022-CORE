package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;


public class Drive extends CommandBase {

  // DriveTrain subsystem to manipulate the DifferentialDrive
  private DriveTrain driveTrainSubsystem;

  private WPI_TalonFX encLeftMotor; 
  private WPI_TalonFX encRightMotor;

  // initialize Drive command
  public Drive(DriveTrain subsystem, MotorController encLeftMotor, MotorController encRightMotor) {
    driveTrainSubsystem = subsystem;
    addRequirements(driveTrainSubsystem);
    this.encLeftMotor = (WPI_TalonFX) encLeftMotor;
    this.encRightMotor = (WPI_TalonFX) encRightMotor;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override

  public void execute() {
    driveTrainSubsystem.driveWithJoystick();
    driveTrainSubsystem.encoderDrive(encLeftMotor, encRightMotor);
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
    