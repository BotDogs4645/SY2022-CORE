// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class SetShooterRPM extends CommandBase {

  private double RPM = 2000;
  private Shooter sub;

  /** Creates a new SetShooterRPM. */
  public SetShooterRPM(Shooter sub, double RPM) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.sub = sub;
    addRequirements(sub);
    this.RPM = RPM;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  public void overrideRPM(double override) {
    RPM = override;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sub.setShooterDemandRPM(RPM);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sub.setShooterDemandRPM(0);
    sub.pidReset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
