// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterIntegratedPID extends SubsystemBase {
  private WPI_TalonFX _talon;
  private WPI_TalonFX _talon2;

  private boolean enabled = false;
  private double avg_error = 0;
  private int countee = 0;

  public ShooterIntegratedPID(WPI_TalonFX shootie, WPI_TalonFX loadie) {
    this._talon = shootie;
    _talon.configFactoryDefault();
    _talon.set(TalonFXControlMode.Velocity, Constants.IntegratedShooterPID.RPMSetpoint);
    _talon.configNeutralDeadband(.05); // 25
    _talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.IntegratedShooterPID.PIDLoopId, Constants.IntegratedShooterPID.timeoutMS);

    _talon.configNominalOutputForward(0, Constants.IntegratedShooterPID.timeoutMS);
		_talon.configNominalOutputReverse(0, Constants.IntegratedShooterPID.timeoutMS);
    _talon.configPeakOutputForward(1, Constants.IntegratedShooterPID.timeoutMS);
		_talon.configPeakOutputReverse(-1,  Constants.IntegratedShooterPID.timeoutMS);

    _talon.config_kF(Constants.IntegratedShooterPID.PIDLoopId, Constants.IntegratedShooterPID.kF, Constants.IntegratedShooterPID.timeoutMS);
		_talon.config_kP(Constants.IntegratedShooterPID.PIDLoopId, Constants.IntegratedShooterPID.kP, Constants.IntegratedShooterPID.timeoutMS);
		_talon.config_kI(Constants.IntegratedShooterPID.PIDLoopId, Constants.IntegratedShooterPID.kI, Constants.IntegratedShooterPID.timeoutMS);
		_talon.config_kD(Constants.IntegratedShooterPID.PIDLoopId, Constants.IntegratedShooterPID.kD, Constants.IntegratedShooterPID.timeoutMS);
    // Aidan was here
    //Loadie PID
    _talon2 = loadie;
    _talon2.configFactoryDefault();
    _talon2.set(TalonFXControlMode.Velocity, Constants.IntegratedShooterPID.RPMSetpoint2);
    _talon2.configNeutralDeadband(.05); // 25
    _talon2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.IntegratedShooterPID.PIDLoopId, Constants.IntegratedShooterPID.timeoutMS);

    _talon2.configNominalOutputForward(0, Constants.IntegratedShooterPID.timeoutMS);
		_talon2.configNominalOutputReverse(0, Constants.IntegratedShooterPID.timeoutMS);
    _talon2.configPeakOutputForward(1, Constants.IntegratedShooterPID.timeoutMS);
		_talon2.configPeakOutputReverse(-1,  Constants.IntegratedShooterPID.timeoutMS);

    _talon2.config_kF(Constants.IntegratedShooterPID.PIDLoopId, Constants.IntegratedShooterPID.kF, Constants.IntegratedShooterPID.timeoutMS);
		_talon2.config_kP(Constants.IntegratedShooterPID.PIDLoopId, Constants.IntegratedShooterPID.kP, Constants.IntegratedShooterPID.timeoutMS);
		_talon2.config_kI(Constants.IntegratedShooterPID.PIDLoopId, Constants.IntegratedShooterPID.kI, Constants.IntegratedShooterPID.timeoutMS);
		_talon2.config_kD(Constants.IntegratedShooterPID.PIDLoopId, Constants.IntegratedShooterPID.kD, Constants.IntegratedShooterPID.timeoutMS);

  }

  @Override
  public void periodic() {
    if (enabled) {
      SmartDashboard.putNumber("shootie@errorRpm:", _talon.getClosedLoopError());
      SmartDashboard.putNumber("shootie@target:", _talon.getClosedLoopTarget(Constants.IntegratedShooterPID.PIDLoopId) / Constants.IntegratedShooterPID.ConversionRate);
      SmartDashboard.putNumber("shootie@currentrpm:", _talon.getSelectedSensorVelocity() * 2048 / 6000);
      avg_error += _talon.getClosedLoopError();
      countee++;
      SmartDashboard.putNumber("shootie@avgErr:", avg_error / countee);
    }
  }
//impasta
  public void enable() {
    enabled = true;
    countee = 0;
    avg_error = 0;
    _talon.set(TalonFXControlMode.Velocity, Constants.IntegratedShooterPID.RPMSetpoint * Constants.IntegratedShooterPID.ConversionRate);
   // _talon2.set(TalonFXControlMode.Velocity, Constants.IntegratedShooterPID.RPMSetpoint2 * Constants.IntegratedShooterPID.ConversionRate);
  }

  public void disable() {
    enabled = false;
    _talon.set(TalonFXControlMode.Disabled, 0);
    //_talon2.set(TalonFXControlMode.Disabled, 0);
  }


}