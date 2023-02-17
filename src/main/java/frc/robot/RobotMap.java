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

    public static final class ElevatorPort {
        public static final int ELEVATOR_MOTOR = 13;
        public static final int ARM_MOTOR = 14;
        public static final int WRIST_MOTOR = 15;
        public static final int LEFT_INTAKE_MOTOR = 16;
        public static final int RIGHT_INTAKE_MOTOR = 17;
        public static final int WINCH_MOTOR = 19;

        // DIO
        public static final int ELEVATOR_LIMIT_SWITCH = 0;
        public static final int ARM_LIMIT_SWITCH = 1;
        public static final int WRIST_ABS_ENCODER = 2;
        public static final int WINCH_ABS_ENCODER = 3;
        public static final int[] GRIPPER_LIMIT_SWITCH = {4, 5};

        // Pneumatics
        public static final int[] WINCH_RELEASE_DOUBLE_SOLENOID = {14, 15};
        public static final int[] GRIPPER_RELEASE_DOUBLE_SOLENOID = {2, 3};
    }

    public static final class PneumaticsPort {
        public static final int REV_PH_PORT = 21;
    }
}