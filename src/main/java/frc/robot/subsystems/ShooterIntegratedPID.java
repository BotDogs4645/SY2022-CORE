package frc.robot.subsystems;

import javax.swing.text.html.HTMLDocument.BlockElement;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterIntegratedPID extends SubsystemBase {
  private WPI_TalonFX shootie;
  private WPI_TalonFX loadie;

  private WPI_TalonFX vertical;
  private WPI_TalonFX horizontal;
  private double speed = .3;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-console");

  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry limeMode = table.getEntry("ledMode");

  private boolean limeOn = false;
  public boolean isAtSetpoint = false;
  private boolean limelightModeEnabled = false;
  private double avg_error = 0;
  private int setPointCount = 0;
  private int countee = 0;

  private static double lastShotTime = Integer.MAX_VALUE;

  public ShooterIntegratedPID(WPI_TalonFX shootie, WPI_TalonFX loadie, WPI_TalonFX vertical, WPI_TalonFX horizontal) {
    this.vertical = vertical;
    this.horizontal = horizontal;

    this.shootie = shootie;
    shootie.configFactoryDefault();
    this.loadie = loadie;
    loadie.configFactoryDefault();
    
    TalonFXConfiguration _rightConfig = new TalonFXConfiguration();
    
    shootie.set(TalonFXControlMode.PercentOutput, 0);
    loadie.set(TalonFXControlMode.PercentOutput, 0);

    shootie.setNeutralMode(NeutralMode.Brake);
    loadie.setNeutralMode(NeutralMode.Brake);

    shootie.follow(loadie);

    shootie.setInverted(TalonFXInvertType.CounterClockwise);
    loadie.setInverted(TalonFXInvertType.Clockwise);

    _rightConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

    _rightConfig.slot0.kF = Constants.IntegratedShooterPID.kF;
    _rightConfig.slot0.kP = Constants.IntegratedShooterPID.kP;
    _rightConfig.slot0.kI = Constants.IntegratedShooterPID.kI;
    _rightConfig.slot0.kD = Constants.IntegratedShooterPID.kD;
    _rightConfig.slot0.integralZone = Constants.IntegratedShooterPID.kIZone;
    _rightConfig.slot0.closedLoopPeakOutput = Constants.IntegratedShooterPID.peakOut;

    _rightConfig.neutralDeadband = 0.001;

    int closedLoopTimeMs = 1;

    _rightConfig.slot0.closedLoopPeriod = closedLoopTimeMs;
    _rightConfig.slot1.closedLoopPeriod = closedLoopTimeMs;
    _rightConfig.slot2.closedLoopPeriod = closedLoopTimeMs;
    _rightConfig.slot3.closedLoopPeriod = closedLoopTimeMs;


    loadie.configAllSettings(_rightConfig);

    shootie.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, Constants.IntegratedShooterPID.timeoutMS);
    shootie.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255, Constants.IntegratedShooterPID.timeoutMS);

    Constants.IntegratedShooterPID.RPM_SETPOINT = 0.0;

  }

  /*
    A basic run down of the Shooter subsystem:
    1. Limelight
      a. Limelight is to be enabled with a button press (Currently button 7 on the side of the joystick)
      b. Shooting w/o limelight is not recommended but can be done if its manually disabled.
      c. The limelight has to be initally aimed somewhere near the tape of the upper hub to begin to orient to it.
      d. When the chassis is aimed and aligned to the hub: the down, hold and release methods will be enabled on the trigger
    2. The down, hold and release methods
      a. whenPressed - activates and runs the flywheels to a specific setpoint after the limelight mode is enabled, constantly calculating
        distance to translate to RPM for shooting. 
      b. whenReleased - deactivates the the flywheels and stops indexing the cargo.
      c. whenHeld - Constantly checks the boolean for the cooldown. If the cooldown is over, then the indexer belts move the ball to the
        shooter.
  */

  @Override
  public void periodic() {
    if (limelightModeEnabled) {
      SmartDashboard.putNumber("shootie@errorRPM:", shootie.getClosedLoopError(0));
      SmartDashboard.putNumber("shootie@target:", loadie.getClosedLoopTarget(Constants.IntegratedShooterPID.PID_LOOP_ID) / Constants.IntegratedShooterPID.CONVERSION_RATE);
      SmartDashboard.putNumber("shootie@currentRPM:", shootie.getSelectedSensorVelocity(1) * (2048.0 / 6000.0));
      SmartDashboard.putNumber("loadie@currentRPM:", loadie.getSelectedSensorVelocity(1) * (2048.0 / 6000.0));
      avg_error += loadie.getClosedLoopError();
      countee++;
      SmartDashboard.putNumber("shootie@avgErr:", avg_error / countee);

      if (!limeOn) {
        limeOn = true;
        limeMode.setDouble(0.0);
        DriveTrain.driveMode = Constants.GamepadButtons.LIMELIGHT_DRIVE;
      }
      // tracks object when limey is on

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

        Constants.IntegratedShooterPID.RPM_SETPOINT = RPMConversion + 200;


        SmartDashboard.putNumber("exitVeloReq@", exitVelocity);
        SmartDashboard.putNumber("distanceFromHub@", distance);
      } 
    } else {
      limeOn = false;
      limeMode.setDouble(1.0);
      DriveTrain.driveMode = Constants.GamepadButtons.JOYSTICK_DRIVE;
    }
  }

  public boolean isAtSetpoint() {
    if(Math.abs(this.getVelocity() - Constants.IntegratedShooterPID.RPM_SETPOINT) < 300) {
      setPointCount++;
      if(setPointCount >= 20) {
          return true;
      }
    } else {
        setPointCount = 0;
    }
    return false;
  }

  public double getVelocity() {
    return loadie.getSelectedSensorVelocity(0);   
  }

  public double getDistanceFromHub() {
    double yOffset = ty.getDouble(0.0);
    double radians = Math.toRadians(yOffset);
    //In inchies vv
    double limeDistance = ((Constants.limelightConstants.LIMELIGHT_HEIGHT - Constants.gameConstants.HIGH_GOAL_HEIGHT) / Math.tan(radians)) / 12;
    double hypotenuse = (Math.sqrt((Math.pow(limeDistance, 2) + Math.pow((Constants.gameConstants.LOW_GOAL_HEIGHT - Constants.limelightConstants.LIMELIGHT_HEIGHT), 2))));
    //Ball exit distance as a funtion of limelight distance; converts from feet to m and returns m
    double exitDistanceMeters = Math.pow((Math.pow((Math.pow((hypotenuse * .3048), 2) - 0.23512801), .5) + 0.0198161929) + 0.060516, .5);
    double exitDistanceFeet = exitDistanceMeters * 3.28084; //back to footsies

    return exitDistanceFeet;
  }

  public void continuousIndexCheck() {
    if(isAtSetpoint()) {
      upperBeltIndex();
    } else {
      stopCargo();
    }
  }
  
  public void toggleOn() {
    countee = 0;
    avg_error = 0;
    
    if (isAtSetpoint()) {
      upperBeltIndex();
      lastShotTime = Timer.getFPGATimestamp();
    }

    loadie.set(TalonFXControlMode.Velocity, (Constants.IntegratedShooterPID.RPM_SETPOINT * Constants.IntegratedShooterPID.CONVERSION_RATE));
  }

  public void toggleOff() {
    stopCargo();
    shootie.set(TalonFXControlMode.Disabled, 0);
    loadie.set(TalonFXControlMode.Disabled, 0);
  }

  public void limeyToggle() {
    limelightModeEnabled = !limelightModeEnabled;
  }

  public void lowerBeltIndex() {
    vertical.set(speed);
  }

  public void upperBeltIndex() {
    horizontal.set(speed);
  }

  public void stopCargo() {
    vertical.set(0);
    horizontal.set(0);
  }

  public void rejectCargo() {
    vertical.set(-speed);
    horizontal.set(-speed);
  }

}
