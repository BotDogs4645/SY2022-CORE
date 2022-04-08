package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;

public class LimelightMath extends SubsystemBase {
    private NetworkTableEntry limelightDistanceInNT;
    private NetworkTableEntry velocityNT;
    private NetworkTableEntry nTy;
    private NetworkTableEntry nTx;
    public double ty;
    public double tx;
    public double v;
    public double target = 0.0;

    private HashMap<Double, Double> control;

    public double adjacent;

    /* 
        Unused:

        double exitVelocity;
        double RPMConversion;

        if (distance < 11.811) {
          exitVelocity = ((-0.00354 * Math.pow(distance, 2)) + (.348 * distance) + (2.38));
        } else {
          exitVelocity = ((.00107 * Math.pow(distance, 2)) + (.111 * distance) + (6.28));
        }

        RPMConversion = Math.pow(exitVelocity / .703595, 3.57002606);
    */

    public LimelightMath() {
        // Correlating distances -> RPMs
        control = new HashMap<Double, Double>();
        control.put(9.676, 3365.0);
        control.put(13.0545, 3910.0);
        control.put(23.141, 6325.0);

        this.limelightDistanceInNT = Shuffleboard.getTab("LimeTuning")
                .add("DistanceOutput", 0.0)
                .getEntry();
        this.velocityNT = Shuffleboard.getTab("LimeTuning")
                .add("IdealVelocity", 0.0)
                .getEntry();
        this.nTy = Shuffleboard.getTab("LimeTuning")
                .add("ty", 0.0)
                .getEntry();
        this.nTx = Shuffleboard.getTab("LimeTuning")
                .add("tx", 0.0)
                .getEntry();
    }

    public void periodic() {
        target = NetworkTableInstance.getDefault().getTable("limelight-console").getEntry("tv").getDouble(0.0);
        SmartDashboard.putNumber("tar", target);
        if (target == 0.0) {
            this.adjacent = getDistanceFromHub();
            this.limelightDistanceInNT.setDouble(adjacent);
            this.velocityNT.setDouble(v);
            this.nTx.setDouble(tx);
            this.nTy.setDouble(ty);
        }
    }

    public double getDistanceFromHub() {
        this.ty = NetworkTableInstance.getDefault().getTable("limelight-console").getEntry("ty").getDouble(0.0);
        this.tx = NetworkTableInstance.getDefault().getTable("limelight-console").getEntry("tx").getDouble(0.0);
        double radians = Math.toRadians(ty) + Math.toRadians(LimelightConstants.LIMELIGHT_ANGLE);
        //In feet vv
        double adjacent = ((Constants.GameConstants.HIGH_GOAL_HEIGHT - Constants.LimelightConstants.LIMELIGHT_HEIGHT) / Math.tan(radians)) / 12;
        return adjacent;
    }

    public double relateDistanceToRPM() {
        return getClosestRelatedDistance(false);
    }

    public double getClosestRelatedDistance(boolean key) {
        double curClosest = Integer.MIN_VALUE;
        double curClosestDistance = Integer.MAX_VALUE;
        double curClosestRPM = 0;
        // Searches for the closest distance to a currently plotted distance -> rpm
        for (Map.Entry<Double, Double> entry : control.entrySet()) {
            if (Math.abs(entry.getKey() - adjacent) < curClosestDistance) {
                curClosest = entry.getKey();
                curClosestDistance = Math.abs(entry.getKey() - adjacent);
                curClosestRPM = entry.getValue();
            }
        } 
        if (key) {
            return curClosest;
        } else {
            return curClosestRPM;
        }
    }



    public void ledSetDefaultState() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    }

    public void ledOff() {
        NetworkTableInstance.getDefault().getTable("limelight-console").getEntry("ledMode").setNumber(1);
    }

    
    public void ledBlink() {
        NetworkTableInstance.getDefault().getTable("limelight-console").getEntry("ledMode").setNumber(2);
    }

    public void ledOn() {
        NetworkTableInstance.getDefault().getTable("limelight-console").getEntry("ledMode").setNumber(3);
    }

}
