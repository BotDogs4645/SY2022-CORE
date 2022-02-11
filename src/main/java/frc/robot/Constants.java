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
    public static final class driveModeConstants {
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
    }  

    public static final class encoderConstants {
        // wheel constants
        public static int k_UNITS_PREVOLUTION = 2048;
        public static double WHEEL_DIAMETERFT = 0.5; // 6 inches

        public static double REVOLUTION_PFT = 1 / WHEEL_DIAMETERFT * Math.PI;

        // target distance for autonomous
        public static double TARGET_DISTANCEFT = 8.46; // the distance from the field to the bottom of the vision tape

        public static double LEFT_SPEED = 0.4;
        public static double RIGHT_SPEED = 0.4;

        public static int ENCODER_BUTTON = 2;
    }
}
