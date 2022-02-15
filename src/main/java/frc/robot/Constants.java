// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
    public static final class GamepadButtons {
        public static int ENCODER_BUTTON = 2;
    }

    public static final class DriveModeConstants {
        public static int JOYSTICK_DRIVE = 0;
        public static int LIMELIGHT_DRIVE = 1;
        public static int ENCODER_DRIVE = 2;
    }

    public static final class DriveConstants {
        // left motor ports
        public static final int UPPER_LEFT_MOTOR = 10;
        public static final int LOWER_LEFT_MOTOR  = 7;

        // right motor ports
        public static final int UPPER_RIGHT_MOTOR = 9;
        public static final int LOWER_RIGHT_MOTOR = 8;

        // joystick ports
        public static final int DRIVE_CONTROLLER = 0;

        // max output
        public static final double MAX_OUTPUT = 0.75;
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

    public static final class EncoderConstants {
      // wheel constants
      public static int k_UNITS_PREVOLUTION = 2048;
      public static double WHEEL_DIAMETER_FT = 0.5; // 6 inches

      public static double REVOLUTION_P_FT = 1 / WHEEL_DIAMETER_FT * Math.PI;

      // target distance for autonomous
      public static double TARGET_DISTANCE_FT = 8.46; // the distance from the field to the bottom of the vision tape

      public static double LEFT_SPEED = 0.4;
      public static double RIGHT_SPEED = 0.4;
    }
}
