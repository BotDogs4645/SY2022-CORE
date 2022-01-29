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
    public static final class driveConstants {
        // left motor ports
        public static final int upperLeftMotor = 10;
        public static final int lowerLeftMotor = 7;

        // right motor ports
        public static final int upperRightMotor = 9;
        public static final int lowerRightMotor = 8;

        // joystick ports
        public static final int driveController = 0;

        // max output
        public static final double maxOutput = 0.75;
    }   
    public static final class encoderConstants{
        public static int kUnitsPerRevolution = 2048;
        public static double wheelRadiusInches = 3;

        public static String messagetoMrBerg = "HI MR BERG :)";
    }
}
