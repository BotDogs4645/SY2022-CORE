// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private WPI_TalonFX loader;
  private WPI_TalonFX shooter;

  private double loaderRPM = 0;
  private double shooterRPM = 0;

  private final PIDController pid = new PIDController(0.1, 0.00014, 0.12);

  public Shooter(WPI_TalonFX loader, WPI_TalonFX shooter) {
    this.loader = loader;
    this.shooter = shooter;
    pid.setTolerance(5, 10);
  }

  @Override
  public void periodic() {
    // Calculate the tangent line velocity of the two shooters and update the RPMs for those values.
    loaderRPM = loader.getSelectedSensorVelocity() * Constants.ShooterConstants.CONVERT_RPM;
    shooterRPM = shooter.getSelectedSensorVelocity() * Constants.ShooterConstants.CONVERT_RPM;

    SmartDashboard.putNumber("Shooter RPM", shooterRPM);
    SmartDashboard.putNumber("Loader RPM", loaderRPM);
    SmartDashboard.putBoolean("Shooter@Setpoint", pid.atSetpoint());
  }

  public void setShooterDemandRPM(double setpoint) {
    SmartDashboard.putNumber("setpoint", setpoint);
    shooter.set(ControlMode.Velocity, pid.calculate(loaderRPM, setpoint));
  }

  public void setLoaderDemandRPM(double setpoint) {
    loader.set(ControlMode.Velocity, pid.calculate(loaderRPM, setpoint));
  }

  public void testTrial() {
    setLoaderDemandRPM(12);
  }

  public double getShooterCurrentRPM() {
    return shooterRPM;
  }

  public double getLoaderCurrentRPM() {
    return loaderRPM;
  }

  public void pidReset() {
    pid.reset();
  }
}
