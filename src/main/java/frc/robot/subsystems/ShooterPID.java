// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class ShooterPID extends PIDSubsystem {
  private WPI_TalonFX shootie;
  private WPI_TalonFX loadie;

  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(-0.00028163, 0.00032042, 1.0505E-05);

  /** Creates a new Shooter2. */
  public ShooterPID(WPI_TalonFX shooter) {
    super(new PIDController(Constants.ShooterConstants.kP, Constants.ShooterConstants.kI, Constants.ShooterConstants.kD));
    shootie = shooter;
    //loadie = loader;
    getController().setTolerance(Constants.ShooterConstants.shooterTolerance);
    // RPM to voltage conversion rate (Max voltage over the max rotation ath that voltage)
    setSetpoint(Constants.ShooterConstants.RPM_SETPOINT * ((double)12 / 6380));
    SmartDashboard.putNumber("setpoint", Constants.ShooterConstants.RPM_SETPOINT * ((double)12 / 6380));
  }

  @Override
  public void useOutput(double output, double setpoint) {
    shootie.setVoltage(output + feedforward.calculate(setpoint));
    SmartDashboard.putNumber("output", output);
    SmartDashboard.putNumber("feedforward", feedforward.calculate(setpoint));
    SmartDashboard.putNumber("shootie@rpm:", shootie.getSelectedSensorVelocity() / 2048 * 60);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return shootie.getMotorOutputVoltage();
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  public void runFeeder() {
    loadie.set(ControlMode.PercentOutput, Constants.ShooterConstants.loaderPercentOut);
  }

  public void stopFeeder() {
    loadie.set(ControlMode.PercentOutput, 0);
  }
}
