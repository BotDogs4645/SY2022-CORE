package frc.robot;
public final class Constants {
    public static final class GamepadButtons {
        // drive modes
        public static int LIMELIGHT_DRIVE = 2;
        public static int ENCODER_DRIVE = 3;
		public static int JOYSTICK_DRIVE = 6;
        
        // misc. buttons
        public static int SHOOTER = 1;
        public static int CLIMBER_BUTTON = 4;
        public static int GRIP_BUTTON = 7;
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
        public static final double MAX_OUTPUT = 0.75;
        
        // limelight tracking constants
        public static final double MIN_ROT_SPEED = .15;
        public static final double ROT_MULTIPLIER = -0.05;
    }

    public static final class LimelightConstants {
        public static final double LIMELIGHT_HEIGHT = 3.25; // distance from ground in inches
    }

    public static final class GameConstants {
        public static final double GOAL_HEIGHT = 57.5; //in inches
        public static final double ITERATIVE_TIME = 0.02;
    } 

    public static final class IntegratedShooterPID {
        public final static int SHOOTIE_ID = 13;
        public final static int LOADIE_ID = 12;

        public static double SHOOTIE_RPM_SETPOINT = 6350.0; // 1560
        public static double LOADIE_RPM_SETPOINT = 6350.0;

        public static final double MAX_RPM = 6380.0;

        public static final double kP = 0.000525; // .000525 .005 - .1 too low <3 //kU * 0.6;
        public static final double kI = 0;
        public static final double kD = 5.75; // 70
        public static final double kF = 0.0442818804; // SHOOTIE_RPM_SETPOINT / MAX_RPM; //(1023 * .75) / 2260; // 22600

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
    }
  
    public static final class ClimberConstants {
        public static int RIGHT_CLIMBER_ID = 4; //PLACEHOLDERS
        public static int LEFT_CLIMBER_ID = 5;

        public static final double kP = 0.0080; // INCREASE FOR TOMMOROW, GYRO IS BACKWARDS TOO, FIX formerly 0.0097
        public static final double kI = 0.00092; // NOT USED
        public static final double kD = 1; // NOT USED
    }
  
    public static final class IndexerConstants {
        public static int VERTICAL_INDEXER_MOTOR  = 6; //PLACEHOLDERS
        public static int HORIZONTAL_INDEXER_MOTOR = 7;
    }
}
