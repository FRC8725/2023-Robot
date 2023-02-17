package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class Joystick {

        public static final double DEADBAND = 0.2;
    }

    public static final class SwerveModuleConstants {
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
        public static final double DRIVE_MOTOR_GEAR_RATIO = 1 / 8.14;
        public static final double TURNING_MOTOR_GEAR_RATIO = 7. / 150.;
        public static final double DRIVE_ENCODER_ROT_2_METER = DRIVE_MOTOR_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS;
        public static final double DRIVE_ENCODER_RPM_2_METER_PER_SEC = DRIVE_ENCODER_ROT_2_METER / 60;
        public static final double TURNING_ENCODER_ROT_2_RAD = TURNING_MOTOR_GEAR_RATIO * 2 * Math.PI;
        public static final double TURNING_ENCODER_RPM_2_RAD_PER_SEC = TURNING_ENCODER_ROT_2_RAD / 60;
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_METERS * 2 * Math.PI;
        public static final double P_TURNING = .4;
        public static final double I_TURNING = 0;
        public static final double D_TURNING = 0;
    }

    public static final class DriveConstants {

        public static final double TRACK_WIDTH = Units.inchesToMeters(24);
        // Distance between right and left wheels
        public static final double WHEEL_BASE = Units.inchesToMeters(24);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
                new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));


        public static final boolean FRONT_LEFT_TURNING_REVERSED = true;
        public static final boolean BACK_LEFT_TURNING_REVERSED = true;
        public static final boolean FRONT_RIGHT_TURNING_REVERSED = true;
        public static final boolean BACK_RIGHT_TURNING_REVERSED = true;

        public static final boolean FRONT_LEFT_DRIVE_REVERSED = true;
        public static final boolean BACK_LEFT_DRIVE_REVERSED = true;
        // CAUTION : next line is only for neo, cause by the different diraction. (default should be false)
        public static final boolean FRONT_RIGHT_DRIVE_REVERSED = false;
        public static final boolean BACK_RIGHT_DRIVE_REVERSED = false;

        public static final boolean FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;
        public static final boolean FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;
        public static final boolean BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;
        public static final boolean BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;

        public static final double FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_OFFSET_ANGLE = 180. + 48.5;
        public static final double FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_OFFSET_ANGLE = 180. + 119.5;
        public static final double BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_OFFSET_ANGLE = 180. + 269.2;
        public static final double BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_OFFSET_ANGLE = 180. + 36.1;

        public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = Units.feetToMeters(13.5);
        // Neo: 12; Falcon: 13.5
        public static final double PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * 2 * Math.PI;

        public static final double TELEOP_DRIVE_MAX_SPEED_METERS_PER_SECOND = PHYSICAL_MAX_SPEED_METERS_PER_SECOND / 2;
        public static final double TELEOP_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = //
                PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND / 4;
        public static final double TELEOP_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND = 3;
        public static final double TELEOP_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = 3;
        public static final double DECREASE_DRIVING_SPEED_FACTOR = .25;
        public static final double DECREASE_TURNING_SPEED_FACTOR = .6;
    }

    public static final class AutoConstants {
        public static final double MAX_SPEED_METERS_PER_SECOND = DriveConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND / 2;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = //
                DriveConstants.PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND / 2;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = Math.PI / 4;
        public static final double PX_CONTROLLER = .6;
        public static final double PY_CONTROLLER = 1.;
        public static final double PTHETA_CONTROLLER = 1;

        public static final double CORRECT_POSITION_X_CONTROLLER = .5;
        public static final double CORRECT_POSITION_Y_CONTROLLER = 2;
        public static final double CORRECT_POSITION_THETA_CONTROLLER = 3;

        public static final TrapezoidProfile.Constraints DRIVE_CONTROLLER_CONSTRAINTS = //
                new TrapezoidProfile.Constraints(
                        MAX_SPEED_METERS_PER_SECOND,
                        MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = //
                new TrapezoidProfile.Constraints(
                        MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
                        MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
    }

    public static final class VisionConstants {
        public static final double Y_OFFSET = Units.inchesToMeters(21.319);

        // UsbCamera
        public static final int[] UsbCameraResolution = {320, 240};
        public static final int[] YELLOW_LOW_THRESHOLD = {0, 100, 50};
        public static final int[] YELLOW_HIGH_THRESHOLD = {40, 255, 255};
    }

    public static final class BalanceConstants {
        public static final double P_BALANCE = 0.05;
        public static final double I_BALANCE = .0;
        public static final double D_BALANCE = .0;
        public static final double xSpeedMax = 2;
        public static final double pitchThreshold = 3;
    }

    public static final class FieldConstants {
        // set the red alliance as reflection of the blue one.
        // Blue is on the left of the field
        public static final double length = Units.feetToMeters(54.27083);
        public static final double width = Units.feetToMeters(26.2916);
    }

    public static final class ElevatorConstants {
        public static final double ELEVATOR_REEL_DIAMETER_METERS = Units.inchesToMeters(0.75);
        public static final double kElevatorReelCircumferenceMeters = ELEVATOR_REEL_DIAMETER_METERS * Math.PI;
        public static final double ARM_REEL_DIAMETER_METERS = Units.inchesToMeters(0.75);
        public static final double ARM_REEL_CIRCUMFERENCE_METERS = ARM_REEL_DIAMETER_METERS * Math.PI;
        public static final double PID_ELEVATOR_POSITION_TOLERANCE = 0.001;
        public static final double PID_ARM_POSITION_TOLERANCE = .001;
        public static final double PID_GRIPPER_ANGULAR_TOLERANCE_RADS = 10./180* Math.PI;
        public static final double PID_WINCH_ANGULAR_TOLERANCE_RADS = 10./180* Math.PI;

        public static final double kElevatorGearRatio = 1./9;
        public static final double kIntakeGearRatio = 1;
        // TODO: Change kWristGearRatio to the data we tested
        public static final double kWristGearRatio = 1/25.;
        public static final double kArmGearRatio = 1./9;
        public static final double kWinchGearRatio = 1./25;

        public static final double kElevatorSpeed = .5;

        public static final double kPElevator = 8;
        public static final double kIElevator = 0;
        public static final double kDElevator = 0;

        public static final double kPWrist = .5;
        public static final double kIWrist = 0;
        public static final double kDWrist = 0;

        public static final double kArmSpeed = .5;
        public static final double kPArm = 5;
        public static final double kIArm = 0;
        public static final double kDArm = 0;

        public static final double kPWinch = .5;
        public static final double kIWinch = 0;
        public static final double kDWinch = 0;

        public static final double kIntakeSpeed = .2;

        public static final double kWristAbsoluteEncoderOffset = 0;
        public static final double kWinchAbsoluteEncoderOffset = 0;

        public static final double kMaxWristAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxWristAngularAccelerationRadiansPerSecond = Math.PI;
        public static final double kMaxWinchAngularSpeedRadiansPerSecond = Math.PI/2;
        public static final double kMaxWinchAngularAccelerationRadiansPerSecond = Math.PI/3;

//        public static final double kMinElevatorHeight = 0;
        public static final double kMinElevatorHeight = Units.inchesToMeters(1);
        public static final double kMaxElevatorHeight = Units.inchesToMeters(6);
        public static final double kMinArmHeight = Units.inchesToMeters(1);
        public static final double kMaxArmHeight = Units.inchesToMeters(20);
        public static final double kMinWinchAngle = Units.degreesToRadians(0);
        public static final double kMaxWinchAngle = Units.degreesToRadians(90);
        public static final double kMinWristAngle = Units.degreesToRadians(-90);
        public static final double kMaxWristAngle = Units.degreesToRadians(90);

        public static final TrapezoidProfile.Constraints kWristControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxWristAngularSpeedRadiansPerSecond,
                        kMaxWristAngularAccelerationRadiansPerSecond);

        public static final TrapezoidProfile.Constraints kWinchControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxWinchAngularSpeedRadiansPerSecond,
                        kMaxWinchAngularAccelerationRadiansPerSecond);
    }

    public static final class PoseConstants {
        // Position Order: Elevator Arm Winch Wrist
        public static final double[] kLowElevatorPose = {0, 0, Math.PI/2, -Math.PI/2};
        public static final double[] kMidElevatorPose = {0, 0, 0, 0};
        public static final double[] kHighElevatorPose = {0, 0, 0, 0};
        public static final double[] kLoadingZoneElevatorPose = {0, 0, 0, 0};
        public static final double[] kInitElevatorPose = {0, 0, 0, 0};
    }
}