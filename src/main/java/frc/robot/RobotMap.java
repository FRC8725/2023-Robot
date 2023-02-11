package frc.robot;

public class RobotMap {

    public static class DriverPort {
        public static final int kFrontLeftDriveMotorPort = 2;
        public static final int kBackLeftDriveMotorPort = 5;
        public static final int kFrontRightDriveMotorPort = 4;
        public static final int kBackRightDriveMotorPort = 7;

        public static final int kFrontLeftTurningMotorPort = 1;
        public static final int kBackLeftTurningMotorPort = 6;
        public static final int kFrontRightTurningMotorPort = 3;
        public static final int kBackRightTurningMotorPort = 8;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 9;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 11;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 10;
        public static final int kBackRightDriveAbsoluteEncoderPort = 12;

    }

    public static class ElevatorPort {
        public static final int kRightWinchMotor = 13;
        public static final int kLeftWinchMotor = 17;
        public static final int kElbowMotor = 14;
        public static final int kWristMotor = 15;
        public static final int kRollMotor = 16;
        public static final int kIntakeLeaderMotor = 18;
        public static final int kIntakeFollowerMotor = 19;

        // DIO
        public static final int kElevatorLimitSwitch = 0;
        public static final int kElbowAbsoluteEncoder = 1;
        public static final int kWristAbsoluteEncoder = 2;
        public static final int kWinchAbsoluteEncoder = 3;
        public static final int[] kGripperLimitSwitch = {4, 5};

        // Pneumatics
        public static final int[] kGripperPressureSwitcherDoubleSolenoid = {0, 1};
        public static final int[] kGripperReleaseDoubleSolenoid = {2, 3};
    }

    public static class PneumaticsPort {
        public static final int kREVPHPort = 21;
    }
}