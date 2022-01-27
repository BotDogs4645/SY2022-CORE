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
        public static final int upperLeftMotor = 0;
        public static final int lowerLeftMotor = 1;

        // right motor ports
        public static final int upperRightMotor = 2;
        public static final int lowerRightMotor = 3;

        // joystick ports
        public static final int leftJoystick = 4;
        public static final int rightJoystick = 5;

        // max output
        public static final double maxOutput = 0.5;
    }
}
