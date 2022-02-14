// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.invoke.ConstantBootstraps;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ShooterIntegratedPID;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final WPI_TalonFX shooterMotor = new WPI_TalonFX(Constants.IntegratedShooterPID.ShooterID);
  public final WPI_TalonFX shooterMotor2 = new WPI_TalonFX(Constants.IntegratedShooterPID.ShooterID2);
  public final ShooterIntegratedPID shooter = new ShooterIntegratedPID(shooterMotor, shooterMotor2);

  public final XboxController xbox = new XboxController(Constants.DriveConstants.driveController);
  public final JoystickButton joyEnable = new JoystickButton(xbox, 2);
  public final JoystickButton joyDisable = new JoystickButton(xbox, 3);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    joyEnable.whenPressed(new InstantCommand(shooter::enable, shooter));
    joyDisable.whenPressed(new InstantCommand(shooter::disable, shooter));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
