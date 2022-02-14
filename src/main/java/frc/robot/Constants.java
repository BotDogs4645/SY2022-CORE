// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveConstants {
        // left motor ports
        public static final int upperLeftMotor = 10;
        public static final int lowerLeftMotor = 7;

        // right motor ports
        public static final int upperRightMotor = 9;
        public static final int lowerRightMotor = 8;

        // joystick ports
        public static final int driveController = 0;

        // max output
        public static final double maxOutput = 0.5;
    }

    public static final class IntegratedShooterPID {
        public final static int ShooterID = 12;
        public final static int ShooterID2 = 13;

        public final static double kU = 0.35;
        public final static double oscP = 4/5;

        public static final double kP = 0.21;
        public static final double kI = 0.325; 
        public static final double kD = 60;
        public static final double kF = (1023 * .75) / 2260; // 22600

        public static final int slotId = 0;
        public static final int PIDLoopId = 0;

        public static final int timeoutMS = 30;

        public static final double RPMSetpoint = 100.0;
        public static final double RPMSetpoint2 = 100.0;

        public static final double ConversionRate = 2048.0 / 600.0;
    }
}
