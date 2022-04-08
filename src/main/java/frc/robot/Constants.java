package frc.robot;

public final class Constants {
    public static final class DriveModes {
        public static int LIMELIGHT_DRIVE = 2;
        public static int ENCODER_DRIVE = 3;
        public static int JOYSTICK_DRIVE = 6;
    }

    public static final class GamepadButtons {
        //public static int CLIMBER_BUTTON = 0; // CHANGE

        public static int LOWER_INTAKE = 2;
        public static int RAISE_INTAKE = 4;

        public static int ABSORB = 6;
        public static int UNABSORB = 1;

        public static int VERTICAL_INDEXER = 5;

        /*
        public static int SHOOTER = 2;

        public static int LOWER_INTAKE = 1;
        public static int RAISE_INTAKE = 4;

        public static int ABSORB = 6;
        public static int UNABSORB = 3;

        public static int VERTICAL_INDEXER = 5;
        */
    }

    public static final class JoystickButtons {
        public static int LIMEY_TOGGLE = 7;
        public static int SHOOTER = 5;
        public static int SHOOTER_ENABLE = 6;
    }

    public static final class DriveConstants {
        // left motor ports
        public static final int UPPER_LEFT_MOTOR = 10;
        public static final int LOWER_LEFT_MOTOR = 7;

        // right motor port
        public static final int UPPER_RIGHT_MOTOR = 9;
        public static final int LOWER_RIGHT_MOTOR = 8;

        // joystick ports
        public static final int DRIVE_CONTROLLER = 0;
        public static final int BUTTON_CONTROLLER = 1;

        // max output
        public static final double MAX_OUTPUT = 0.60;
        
        // limelight tracking constants
        public static final double MIN_ROT_SPEED = .15;
        public static final double ROT_MULTIPLIER = -0.05;
    }

    public static final class LimelightConstants {
        public static double LIMELIGHT_HEIGHT = 22.955; // distance from ground in inches
        public static double LIMELIGHT_ANGLE = 30;
        public static double LIMELIGHT_ROTATION_F = .2;
        public static double LIMELIGHT_ROTATION_P = .12;
        public static double LIMELIGHT_ROTATION_I = 0;
        public static double LIMELIGHT_ROTATION_D = 0;
        public static double LIMELIGHT_ROTATION_TOLERANCE = .8;
        public static double LIMELIGHT_FOW_P = .1;
        public static double LIMELIGHT_FOW_F = .2;
    }

    public static final class GameConstants {
        public static final double LOW_GOAL_HEIGHT = 57.5; //in inches
        public static final double HIGH_GOAL_HEIGHT = 104;
    } 

    public static final class IntegratedShooterPID {
        public final static int SHOOTIE_ID = 13;
        public final static int LOADIE_ID = 12;

        public static double RPM_SETPOINT = 6350.0; // 1560
        public static final double MAX_RPM = 6380.0;

        public static final double kP = 0.000525; // .000525 .005 - .1 too low <3 //kU * 0.6;
        public static final double kI = 0;
        public static final double kD = 5.75; // 70
        public static final double kF = 0.0442818804; // SHOOTIE_RPM_SETPOINT / MAX_RPM; //(1023 * .75) / 2260; // 22600
        public static final double kIZone = 0;
        public static final double peakOut = 1;

        public static final int SLOT_ID = 0;
        public static final int PID_LOOP_ID = 0;

        public static final int timeoutMS = 30;
        public static final double CONVERSION_RATE = 2048.0 / 600.0; // conversion to RPM
    }
    
    public static final class EncoderConstants {
        public static int k_UNITS_P_REVOLUTION = 2048;
        public static double WHEEL_DIAMETER_FT = 0.5; // 6 inches, unit of variable is feet
        public static double REVOLUTION_P_FT = 1 / WHEEL_DIAMETER_FT * Math.PI;
      
        public static double TARGET_DISTANCE_FT = 8.46; // the distance from the field to the bottom of the vision tape 8.46
      
        public static double SPEED = 0.6;

        public static final double kP = 0.0080; // INCREASE FOR TOMMOROW, GYRO IS BACKWARDS TOO, FIX formerly 0.0097
        public static final double kI = 0.00092; // NOT USED
        public static final double kD = 1; // NOT USE

        public static final int HALF_TURN = 3748;
    }
  
    public static final class ClimberConstants {
        public static int RIGHT_CLIMBER_ID = 5; 
        public static int LEFT_CLIMBER_ID = 2;

        public static final double kP = 0.0080; // INCREASE FOR TOMMOROW, GYRO IS BACKWARDS TOO, FIX formerly 0.0097
        public static final double kI = 0.00092; // NOT USED
        public static final double kD = 1; // NOT USED
    }
  
    public static final class IndexerConstants {
        public static int VERTICAL_INDEXER_MOTOR  = 14;  //formally 0, changed bc of pdp conflict
        public static int HORIZONTAL_INDEXER_MOTOR = 1;
        public static final int INTAKE_MOTOR = 3; 
        public static int RAISE_INTAKE_MOTOR = 4;
    }
}