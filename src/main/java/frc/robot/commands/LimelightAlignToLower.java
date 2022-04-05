// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LimelightAlignToLower extends PIDCommand {
  /** Creates a new LimelightAlignToLower. */
  private DriveTrain drive;

  public LimelightAlignToLower(DriveTrain sub) {
    super(
        // The controller that the command will use
        new PIDController(LimelightConstants.LIMELIGHT_ROTATION_P, LimelightConstants.LIMELIGHT_ROTATION_I, 0),
        // This should return the measurement
        sub::getLimeX,
        // This should return the setpoint (can also be a constant)
        0,
        // This uses the output
        output -> sub.trackObject(0, output),
          // Use the output here
        sub
        );
      drive = sub;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  public void initialize() {  // FIXME: remove after tuning
    super.initialize();
    getController().setP(Constants.LimelightConstants.LIMELIGHT_ROTATION_P);
  }

  public void end(boolean interrupted) {
    super.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drive.isAligned();
  }
}
