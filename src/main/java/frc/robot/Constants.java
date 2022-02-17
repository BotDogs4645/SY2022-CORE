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
    public static final class gamepadButtons {
        public static int JOYSTICK_DRIVE = 0;
        public static int LIMELIGHT_DRIVE = 1;
        public static int ENCODER_DRIVE = 2;
    }

    public static final class driveConstants {
        // left motor ports
        public static final int UPPER_LEFT_MOTOR = 10;
        public static final int LOWER_LEFT_MOTOR = 7;

        // right motor ports
        public static final int UPPER_RIGHT_MOTOR = 9;
        public static final int LOWER_RIGHT_MOTOR = 8;

        // joystick ports
        public static final int DRIVE_CONTROLLER = 0;

        // max output
        public static final double MAX_OUTPUT = 0.75;
        
        // limelight tracking constants
        public static final double MIN_ROT_SPEED = .15;
        public static final double ROT_MULTIPLIER = -0.05;
    }

    public static final class limelightConstants {
        public static final double LIMELIGHT_HEIGHT = 3.25; // distance from ground in inches
    }

    public static final class gameConstants {
        public static final double GOAL_HEIGHT = 57.5; //in inches
    } 

    public static final class IntegratedShooterPID {
        public final static int SHOOTIE_ID = 13;
        public final static int LOADIE_ID = 12;

        public final static double kU = 0.4;
        public final static double OSCILLATION_P = 4/5;

        public static final double kP = .15; // .15 - .1 too low <3 //kU * 0.6;
        public static final double kI = .0055; //(1.2* kU) / OSCILLATION_P; 
        public static final double kD = 50; //(3 * kU * OSCILLATION_P) / 40;
        public static final double kF = 0; //(1023 * .75) / 2260; // 22600

        public static final int SLOT_ID = 0;
        public static final int PID_LOOP_ID = 0;

        public static final int timeoutMS = 30;

        public static final double SHOOTIE_RPM_SETPOINT = 2000.0;
        public static final double LOADIE_RPM_SETPOINT = 0.0;

        public static final double CONVERSION_RATE = 2048.0 / 600.0; // conversion to RPM
    }
    
    public static final class encoderConstants {
        public static int k_UNITS_PREVOLUTION = 2048;
        public static double WHEEL_DIAMETERFT = 0.5; // 6 inches, unit of variable is feet
        public static double REVOLUTION_PFT = 1 / WHEEL_DIAMETERFT * Math.PI;

        public static double TARGET_DISTANCEFT = 8.46; // the distance from the field to the bottom of the vision tape
        
        public static double LEFT_SPEED = 0.6;
        public static double RIGHT_SPEED = 0.6;
    }
}
