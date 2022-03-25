package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterIntegratedPID extends SubsystemBase {
  private WPI_TalonFX _talon;
  private WPI_TalonFX _talon2;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-console");

  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry ty = table.getEntry("ty");

  private boolean enabled = false;
  private double avg_error = 0;
  private int countee = 0;

  private static double lastShotTime = Integer.MAX_VALUE;

  public ShooterIntegratedPID(WPI_TalonFX shootie, WPI_TalonFX loadie) {
    this._talon = shootie;
    _talon.configFactoryDefault();
    _talon.setNeutralMode(NeutralMode.Brake);
    _talon.set(TalonFXControlMode.Velocity, 0);
    _talon.setInverted(true);
    _talon.configNeutralDeadband(0); // 25
    _talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.IntegratedShooterPID.PID_LOOP_ID, Constants.IntegratedShooterPID.timeoutMS);

    _talon.configNominalOutputForward(0, Constants.IntegratedShooterPID.timeoutMS);
		_talon.configNominalOutputReverse(0, Constants.IntegratedShooterPID.timeoutMS);
    _talon.configPeakOutputForward(1, Constants.IntegratedShooterPID.timeoutMS);
		_talon.configPeakOutputReverse(-1,  Constants.IntegratedShooterPID.timeoutMS);

    _talon.config_kF(Constants.IntegratedShooterPID.PID_LOOP_ID, Constants.IntegratedShooterPID.kF, Constants.IntegratedShooterPID.timeoutMS);
		_talon.config_kP(Constants.IntegratedShooterPID.PID_LOOP_ID, Constants.IntegratedShooterPID.kP, Constants.IntegratedShooterPID.timeoutMS);
		_talon.config_kI(Constants.IntegratedShooterPID.PID_LOOP_ID, Constants.IntegratedShooterPID.kI, Constants.IntegratedShooterPID.timeoutMS);
		_talon.config_kD(Constants.IntegratedShooterPID.PID_LOOP_ID, Constants.IntegratedShooterPID.kD, Constants.IntegratedShooterPID.timeoutMS);
    _talon.getSensorCollection().setIntegratedSensorPosition(0, 30);
    // Aidan was here
    //Loadie PID
    _talon2 = loadie;
    _talon2.configFactoryDefault();
    _talon2.setNeutralMode(NeutralMode.Brake);
    _talon2.set(TalonFXControlMode.Velocity, 0);
    _talon2.configNeutralDeadband(0); // 25
    _talon2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.IntegratedShooterPID.PID_LOOP_ID, Constants.IntegratedShooterPID.timeoutMS);

    _talon2.configNominalOutputForward(0, Constants.IntegratedShooterPID.timeoutMS);
	  _talon2.configNominalOutputReverse(0, Constants.IntegratedShooterPID.timeoutMS);
    _talon2.configPeakOutputForward(1, Constants.IntegratedShooterPID.timeoutMS);
	  _talon2.configPeakOutputReverse(-1,  Constants.IntegratedShooterPID.timeoutMS);

    _talon2.config_kF(Constants.IntegratedShooterPID.PID_LOOP_ID, Constants.IntegratedShooterPID.kF, Constants.IntegratedShooterPID.timeoutMS);
	  _talon2.config_kP(Constants.IntegratedShooterPID.PID_LOOP_ID, Constants.IntegratedShooterPID.kP, Constants.IntegratedShooterPID.timeoutMS);
	  _talon2.config_kI(Constants.IntegratedShooterPID.PID_LOOP_ID, Constants.IntegratedShooterPID.kI, Constants.IntegratedShooterPID.timeoutMS);
	  _talon2.config_kD(Constants.IntegratedShooterPID.PID_LOOP_ID, Constants.IntegratedShooterPID.kD, Constants.IntegratedShooterPID.timeoutMS);
    _talon2.getSensorCollection().setIntegratedSensorPosition(0, 30);
    SmartDashboard.putNumber("setpoint@shooter", Constants.IntegratedShooterPID.SHOOTIE_RPM_SETPOINT);
}

  @Override
  public void periodic() {
    if (enabled) {
      SmartDashboard.putNumber("shootie@errorRpm:", _talon.getClosedLoopError(0));
      SmartDashboard.putNumber("shootie@target:", _talon.getClosedLoopTarget(Constants.IntegratedShooterPID.PID_LOOP_ID) / Constants.IntegratedShooterPID.CONVERSION_RATE);
      SmartDashboard.putNumber("shootie@currentrpm:", _talon.getSelectedSensorVelocity(1) * (2048.0 / 6000.0));
      SmartDashboard.putNumber("loadie@currentrpm:", _talon2.getSelectedSensorVelocity(1) * (2048.0 / 6000.0));
      avg_error += _talon.getClosedLoopError();
      countee++;
      SmartDashboard.putNumber("shootie@avgErr:", avg_error / countee);

      if (DriveTrain.alignedToHub) {
        double distance = getDistanceFromHub();
        double exitVelocity;
        double RPMConversion;
        if (distance < 11.811) {
          exitVelocity = ((-0.00354 * Math.pow(distance, 2)) + (.348 * distance) + (2.38));
        } else {
          exitVelocity = ((.00107 * Math.pow(distance, 2)) + (.111 * distance) + (6.28));
        }

        RPMConversion = Math.pow(exitVelocity / .703595, 3.57002606);

        Constants.IntegratedShooterPID.LOADIE_RPM_SETPOINT = RPMConversion + 200;
        Constants.IntegratedShooterPID.SHOOTIE_RPM_SETPOINT = RPMConversion;
        
        SmartDashboard.putNumber("exitVeloReq@", exitVelocity);
        SmartDashboard.putNumber("distanceFromHub@", distance);
      }
    }
  }

  public double getDistanceFromHub() {
    double yOffset = ty.getDouble(0.0);
    double radians = Math.toRadians(yOffset);
    double limeDistance = ((Constants.limelightConstants.LIMELIGHT_HEIGHT - Constants.gameConstants.HIGH_GOAL_HEIGHT) / Math.tan(radians)) / 12;
  
    double exitDistance = Math.pow((Math.pow((Math.pow(limeDistance, 2) - 0.23512801), .5) + 0.0198161929) + 0.060516, .5);

    return exitDistance;
  }
  
  public void requestToggle() {
    enabled = !enabled;
    if (enabled) {
      countee = 0;
      avg_error = 0;
      _talon.set(TalonFXControlMode.Velocity, (Constants.IntegratedShooterPID.SHOOTIE_RPM_SETPOINT * Constants.IntegratedShooterPID.CONVERSION_RATE));
      _talon2.set(TalonFXControlMode.Velocity, (Constants.IntegratedShooterPID.LOADIE_RPM_SETPOINT * Constants.IntegratedShooterPID.CONVERSION_RATE));
    } else {
      _talon.set(TalonFXControlMode.Disabled, 0);
      _talon2.set(TalonFXControlMode.Disabled, 0);
    }
  }

  public static boolean shooterCooldown() {
    if (Timer.getFPGATimestamp() - lastShotTime > 2/3 && Timer.getFPGATimestamp() > 2/3) {
      return true;
    }
    return false;
  }
}
