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

    public static final class encoderConstants {
        // wheel constants
        public static int kUnitsPerRevolution = 2048;
        public static double wheelDiameterFeet = 0.5; // 6 inches

        public static double revolutionsPerFoot = 1 / wheelDiameterFeet * Math.PI;

        // target distance for autonomous
        public static int targetDistanceFt = 15; // temporary for testing purposes

        public static double leftSpeed = 0.4;
        public static double rightSpeed = 0.4;

        public static int encoderButton = 2;

    }
}
