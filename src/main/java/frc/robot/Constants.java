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
        public static final double maxOutput = 0.75;
    }

    public static final class ShooterConstants {
        public static final int loaderMotor = 11;
        public static final int shootMotor = 12;
        //0.1, 0.00014, 0.12
        public static final double kP = 0.1;
        public static final double kI = 0.00014;
        public static final double kD = 0.12;

        public static final double shooterTolerance = 5/60;
        public static final double shooterRPMSetpoint = 6;

        public static final double MAX_VOLT = 12;
        public static final double MAX_RPM = 6380;

        public static final double loaderPercentOut = .3;

        // the max speed in RPM that the Falcon 500 can spin at
        public static final int FALCON_MAX_RPM = 6380;
        public static final int CONVERT_RPM = Constants.ShooterConstants.FALCON_MAX_RPM * 2048 / 600;
    }
}
