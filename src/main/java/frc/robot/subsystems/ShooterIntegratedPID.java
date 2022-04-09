package frc.robot.subsystems;

import java.util.Map;
import java.util.logging.LogManager;

import javax.swing.text.html.HTMLDocument.BlockElement;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ShooterIntegratedPID extends SubsystemBase {
  private WPI_TalonFX shootie;
  private WPI_TalonFX loadie;

  private WPI_TalonFX vertical;
  private WPI_TalonSRX horizontal;
  private double speed = .3;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-console");

  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry limeMode = table.getEntry("ledMode");
  LimelightMath LimeMath;

  private boolean limeOn = false;
  public boolean isAtSetpoint = false;

  // MANUAL CONTROL
  public boolean testingMode = true;

  private boolean limelightModeEnabled = false;
  public boolean canShoot = false;
  private double avg_error = 0;
  private int setPointCount = 0;
  private int countee = 0;

  private boolean enabledManual = false;

  private static double lastShotTime = Integer.MAX_VALUE;

  public ShooterIntegratedPID(WPI_TalonFX shootie, WPI_TalonFX loadie, WPI_TalonFX vertical, WPI_TalonSRX horizontalindexermotor) {
    this.vertical = vertical;
    this.horizontal = horizontalindexermotor;
    this.LimeMath = RobotContainer.LimeMath;

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

    shootie.setInverted(TalonFXInvertType.Clockwise);
    loadie.setInverted(TalonFXInvertType.CounterClockwise);

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
    if (testingMode) {
      Shuffleboard.getTab("Shooter")
          .add("VelocitySetpoint", 0.0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 6380)) // specify widget properties here
          .getEntry()
          .addListener(event -> {
            this.setVelocity(event.getEntry().getValue().getDouble());
            setVelocity(event.getEntry().getValue().getDouble());
          }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

      Shuffleboard.getTab("Shooter")
          .add("Flywheel Power", 0.0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
          .getEntry()
          .addListener(event -> {
            loadie.set(TalonFXControlMode.PercentOutput,
                event.getEntry().getValue().getDouble());
            shootie.set(TalonFXControlMode.PercentOutput,
                event.getEntry().getValue().getDouble());
          }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

      Shuffleboard.getTab("Shooter")
          .add("Flywheel F", Constants.IntegratedShooterPID.kF)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
          .getEntry()
          .addListener(event -> {
            this.loadie.config_kF(0, event.getEntry().getValue().getDouble(),
                30);
          }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

      Shuffleboard.getTab("Shooter")
          .add("Flywheel P", Constants.IntegratedShooterPID.kP)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 2.0)) // specify widget properties here
          .getEntry()
          .addListener(event -> {
            this.loadie.config_kP(0, event.getEntry().getValue().getDouble(),
                30);
          }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

      Shuffleboard.getTab("Shooter")
          .add("Flywheel I", Constants.IntegratedShooterPID.kI)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 1.0)) // specify widget properties here
          .getEntry()
          .addListener(event -> {
            this.loadie.config_kI(0, event.getEntry().getValue().getDouble(),
                30);
          }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

      Shuffleboard.getTab("Shooter")
          .add("Flywheel D", Constants.IntegratedShooterPID.kD)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 10.0)) // specify widget properties here
          .getEntry()
          .addListener(event -> {
            this.loadie.config_kD(0, event.getEntry().getValue().getDouble(),
                30);
          }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
          shootie.follow(loadie);
    }
  }

  // being up at 1 am really sucks why did i wait to do this until now

  @Override
  public void periodic() {
    if (loadie.getMotorOutputPercent() > 0) {
      SmartDashboard.putNumber("shootie@errorRPM:", shootie.getClosedLoopError(0));
      SmartDashboard.putNumber("shootie@target:", loadie.getClosedLoopTarget(Constants.IntegratedShooterPID.PID_LOOP_ID) / Constants.IntegratedShooterPID.CONVERSION_RATE);
      SmartDashboard.putNumber("shootie@currentRPM:", shootie.getSelectedSensorVelocity() * (2048.0 / 6000.0));
      SmartDashboard.putNumber("loadie@currentRPM:", loadie.getSelectedSensorVelocity() * (2048.0 / 6000.0));
      avg_error += loadie.getClosedLoopError();
      countee++;
      SmartDashboard.putNumber("shootie@avgErr:", avg_error / countee);

      // enables bot RPM when everything's good to go
      if (DriveTrain.alignedToHub && DriveTrain.inPreferredPosition) {
        double RPM = LimeMath.relateDistanceToRPM();
        setVelocity(RPM);
        if (isAtSetpoint()) {
          canShoot = true;
        }
      }
    }
  }

  public void setRPMFromDistanceAuto() {
    SmartDashboard.putNumber("target", LimeMath.target);
    if ((LimeMath.target == 1.0) && !enabledManual) {
      enabledManual = true;
      double closestRPMToDistance = LimeMath.getClosestRelatedDistance(false);
      double closestDistance = LimeMath.getClosestRelatedDistance(true);
      double curDistance = LimeMath.getDistanceFromHub();
      double slope = 545.0 / 3.3785;
      double inc = (curDistance - closestDistance) * slope;
      setVelocity(closestRPMToDistance + inc);
    }
    else if ((LimeMath.target == 0.0)) {
      enabledManual = false;
      toggleOff();
    }
  }

  public void setVelocity(double rpm) {
    Constants.IntegratedShooterPID.RPM_SETPOINT = rpm;
    loadie.set(TalonFXControlMode.Velocity, (Constants.IntegratedShooterPID.RPM_SETPOINT * Constants.IntegratedShooterPID.CONVERSION_RATE));
  }
  
  public boolean isAtSetpoint() {
    if (Math.abs(this.getVelocity() - Constants.IntegratedShooterPID.RPM_SETPOINT) < 300) {
      setPointCount++;
      if (setPointCount >= 20) {
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

  public void toggleOff() {
    stopCargo();
    shootie.set(TalonFXControlMode.Disabled, 0);
    loadie.set(TalonFXControlMode.Disabled, 0);
  }

  public void limeOn() {
    limelightModeEnabled = true;
    LimeMath.ledSetDefaultState();
  }

  public void limeOff() {
    limelightModeEnabled = false;
    LimeMath.ledOff();
  }

  public void lowerBeltIndex() {
    vertical.set(speed);
  }

  public void upperBeltIndex() {
    if (canShoot || !limelightModeEnabled) {
      horizontal.set(speed);
    }
  }

  public void stopCargo() {
    vertical.set(0);
    horizontal.set(0);
  }

  public void rejectCargo() {
    vertical.set(-speed);
    horizontal.set(-speed);
  }
  public void shoot(int speed, int balls) {
    double lastShotTime = Timer.getFPGATimestamp();
    if(balls == 1) {
      upperBeltIndex();
      setVelocity(speed);
      if(Timer.getFPGATimestamp() - lastShotTime > 1) {
        setVelocity(0);
        horizontal.set(0);
      }
    }
    if(balls == 2) {
      upperBeltIndex();
      setVelocity(speed);
      if (Timer.getFPGATimestamp() - lastShotTime > 4) {
        horizontal.set(0);
        setVelocity(0);
      }
    }
  }
}