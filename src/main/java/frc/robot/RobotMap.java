package frc.robot;

public final class RobotMap {

    public static final class DriverPort {
        public static final int FRONT_LEFT_DRIVE_MOTOR_PORT = 2;
        public static final int BACK_LEFT_DRIVE_MOTOR_PORT = 5;
        public static final int FRONT_RIGHT_DRIVE_MOTOR_PORT = 4;
        public static final int BACK_RIGHT_DRIVE_MOTOR_PORT = 7;

        public static final int FRONT_LEFT_TURNING_MOTOR_PORT = 1;
        public static final int BACK_LEFT_TURNING_MOTOR_PORT = 6;
        public static final int FRONT_RIGHT_TURNING_MOTOR_PORT = 3;
        public static final int BACK_RIGHT_TURNING_MOTOR_PORT = 8;

        public static final int FRONT_LEFT_DRIVE_ABS_ENCODER_PORT = 9;
        public static final int BACK_LEFT_DRIVE_ABS_ENCODER_PORT = 11;
        public static final int FRONT_RIGHT_DRIVE_ABS_ENCODER_PORT = 10;
        public static final int BACK_RIGHT_DRIVE_ABS_ENCODER_PORT = 12;

    }

    public static class ArmPort {
        public static final int RIGHT_WINCH_MOTOR = 13;
        public static final int LEFT_WINCH_MOTOR = 14;
        public static final int ELBOW_MOTOR = 15;
        public static final int WRIST_MOTOR = 16;
//        public static final int ROLL_MOTOR = 17;
//        public static final int INTAKE_LEADER_MOTOR = 18;
//        public static final int INTAKE_FOLLOWER_MOTOR = 19;

        // DIO
        public static final int WINCH_ABSOLUTE_ENCODER = 0;
        public static final int ELBOW_ABSOLUTE_ENCODER = 1;
        public static final int WRIST_ABSOLUTE_ENCODER = 2;
        public static final int[] GRIPPER_LIMIT_SWITCH = {4, 5};

        // Pneumatics
        public static final int[] GRIPPER_RELEASE_DOUBLE_SOLENOID = {8, 9};
        public static final int[] GRIPPER_PRESSURE_SWITCHER_DOUBLE_SOLENOID = {2, 3};
    }

    public static final class PneumaticsPort {
        public static final int REV_PH_PORT = 21;
    }

    public static final class LEDPort {
        // PWM
        public static final int LED_PORT = 9;
    }
}