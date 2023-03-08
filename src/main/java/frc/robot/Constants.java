package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import java.util.List;

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

        public static final double DEADBAND = 0.1;
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
        public static final double P_TURNING = .5;
        public static final double I_TURNING = 0;
        public static final double D_TURNING = 0;
    }

    public static final class DriveConstants {

        public static final double TRACK_WITH_BUMPER_WIDTH = 0.83;

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

        public static final double FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_OFFSET_ANGLE = 223.41;
        public static final double FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_OFFSET_ANGLE = 299.704;
        public static final double BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_OFFSET_ANGLE = 267.45;
        public static final double BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_OFFSET_ANGLE = 33.75;

        public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = Units.feetToMeters(13.5);
        // Neo: 12; Falcon: 13.5
        public static final double PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * 2 * Math.PI;

        public static final double TELEOP_DRIVE_MAX_SPEED_METERS_PER_SECOND = PHYSICAL_MAX_SPEED_METERS_PER_SECOND * 0.65;
        public static final double TELEOP_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = //
                PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND / 3;
        public static final double TELEOP_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND = 3;
        public static final double TELEOP_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = 3;
        public static final double DECREASE_DRIVING_SPEED_FACTOR = .4;
        public static final double DECREASE_TURNING_SPEED_FACTOR = .6;
        public static final double P_JOYSTICK_TURNING = Math.PI / 4;
    }

    public static final class AutoConstants {
        public static final double MAX_SPEED_METERS_PER_SECOND = DriveConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND / 3;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = //
                DriveConstants.PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND / 5;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 2;
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = Math.PI / 5;
        public static final double P_X_CONTROLLER = 1.;
        public static final double P_Y_CONTROLLER = 1.;
        public static final double P_THETA_CONTROLLER = .6;

        public static final double CORRECT_POSITION_X_CONTROLLER = 1;
        public static final double CORRECT_POSITION_Y_CONTROLLER = 1;
        public static final double CORRECT_POSITION_THETA_CONTROLLER = 1.7;

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
        public static final Transform3d Photon2Robot = new Transform3d(
                new Translation3d(-DriveConstants.WHEEL_BASE / 2, 0, 0.5),
                new Rotation3d());
        public static final Transform3d Robot2Photon = Photon2Robot.inverse();
        public static final Transform3d Tag2Goal =
                new Transform3d(
                        new Translation3d(Units.inchesToMeters(16.113) + DriveConstants.TRACK_WITH_BUMPER_WIDTH / 2, 0, 0),
                        new Rotation3d(0, 0, Math.PI));
        public static final Transform3d GoalMid2Left =
                new Transform3d(
                        new Translation3d(0, Units.inchesToMeters(21.319), 0), new Rotation3d());
        public static final Transform3d GoalMid2Right = GoalMid2Left.inverse();
        public static final Transform3d Tag2Single_Red =
                new Transform3d(
                        new Translation3d(-2.39, Units.feetToMeters(4) - DriveConstants.TRACK_WITH_BUMPER_WIDTH / 2, 0),
                        new Rotation3d(0, 0, -Math.PI / 2));
        public static final Transform3d Tag2Single_Blue =
                new Transform3d(
                        new Translation3d(-2.39, -Units.feetToMeters(4) + DriveConstants.TRACK_WITH_BUMPER_WIDTH / 2, 0),
                        new Rotation3d(0, 0, Math.PI / 2));
        // UsbCamera
        public static final int[] UsbCameraResolution = {320, 240};
    }

    public static final class BalanceConstants {
        public static final double P_BALANCE = 0.075;
        public static final double I_BALANCE = .0;
        public static final double D_BALANCE = .0;
        public static final double xSpeedMax = 1.3;
        public static final double pitchThreshold = 10.;
        public static final double pitchTolerance = 3.;
    }

    public static final class FieldConstants {
        // set the red alliance as reflection of the blue one.
        // Blue is on the left of the field
        public static final double length = Units.feetToMeters(54.27083);
        public static final double width = Units.feetToMeters(26.2916);
        public static final double gridHighDistance = 0.45;
        public static final double gridLowDistance = 0.25;
        public static final AprilTag aprilTag1 = new AprilTag(1, new Pose3d(new Pose2d(Units.inchesToMeters(40.45), Units.inchesToMeters(174.19), Rotation2d.fromDegrees(0))));
        public static final AprilTag aprilTag2 = new AprilTag(2, new Pose3d(new Pose2d(Units.inchesToMeters(40.45), Units.inchesToMeters(108.19), Rotation2d.fromDegrees(0))));
        public static final AprilTag aprilTag3 = new AprilTag(3, new Pose3d(new Pose2d(Units.inchesToMeters(40.45), Units.inchesToMeters(42.19), Rotation2d.fromDegrees(0))));
        //        public static final AprilTag aprilTag4 = new AprilTag(4, new Pose3d(new Pose2d(Units.inchesToMeters(636.96), Units.inchesToMeters(265.74), Rotation2d.fromDegrees(180))));
//        public static final AprilTag aprilTag5 = new AprilTag(5, new Pose3d(new Pose2d(Units.inchesToMeters(14.25), Units.inchesToMeters(265.74), Rotation2d.fromDegrees(0))));
        public static final AprilTag aprilTag6 = new AprilTag(6, new Pose3d(new Pose2d(Units.inchesToMeters(40.45), Units.inchesToMeters(174.19), Rotation2d.fromDegrees(0))));
        public static final AprilTag aprilTag7 = new AprilTag(7, new Pose3d(new Pose2d(Units.inchesToMeters(40.45), Units.inchesToMeters(108.19), Rotation2d.fromDegrees(0))));
        public static final AprilTag aprilTag8 = new AprilTag(8, new Pose3d(new Pose2d(Units.inchesToMeters(40.45), Units.inchesToMeters(42.19), Rotation2d.fromDegrees(0))));
        public static final AprilTagFieldLayout aprilTagField = new AprilTagFieldLayout(
                List.of(aprilTag1, aprilTag2, aprilTag3, aprilTag6, aprilTag7, aprilTag8),
                FieldConstants.length, FieldConstants.width);
    }

    public static final class ArmConstants {

        public static final double UPPER_ARM_LENGTH = Units.inchesToMeters(29.5);
        public static final double FOREARM_LENGTH = Units.inchesToMeters(32.5);

        public static final double xSpdConvertFactor = 0.2;
        public static final double ySpdConvertFactor = 0.2;

        public static final double P_WINCH_BRAKE = .3;
        public static final double P_WRIST_BRAKE = .1;
//        public static final double kElevatorReelDiameterMeters = Units.inchesToMeters(0.75);
//        public static final double kElevatorReelCircumferenceMeters = kElevatorReelDiameterMeters * Math.PI;
//        public static final double kArmReelDiameterMeters = Units.inchesToMeters(0.75);
//        public static final double kArmReelCircumferenceMeters = kArmReelDiameterMeters * Math.PI;
//        public static final double kPIDElevatorPositionTolerance = 0.001;
//        public static final double kPIDArmPositionTolerance = .001;
        public static final double PID_GRIPPER_ANGULAR_TOLERANCE_RADS = Units.degreesToRadians(12);
        public static final double PID_ELBOW_ANGULAR_TOLERANCE_RADS = Units.degreesToRadians(6);
        public static final double PID_WINCH_ANGULAR_TOLERANCE_RADS = Units.degreesToRadians(5);
        public static final double PID_ROLL_ANGULAR_TOLERANCE_RADS = Units.degreesToRadians(5);

//        public static final double kElevatorGearRatio = 1./9;
//        public static final double ROLL_MOTOR_GEAR_RATIO = 1./20;
        // TODO: Change WRIST_GEAR_RATIO to the data we tested
        public static final double WRIST_GEAR_RATIO = 1./80;
        public static final double ELBOW_GEAR_RATIO = 1./100 * 18 / 42;
        public static final double RIGHT_WINCH_GEAR_RATIO = 1./100 * 18 / 34;
//        public static final double RIGHT_WINCH_GEAR_RATIO = 1./125;
        public static final double LEFT_WINCH_GEAR_RATIO = 1./100 * 18 / 34;
//        public static final double LEFT_WINCH_GEAR_RATIO = 1./100;

        public static final boolean WRIST_MOTOR_INVERTED = false;
        public static final boolean ELBOW_MOTOR_INVERTED = true;
        public static final boolean WINCH_MOTOR_INVERTED = false;
        public static final double INTAKE_GEAR_RATIO = 1.;

        public static final double MAX_WINCH_SPEED = .23;
        public static final double MAX_ELBOW_SPEED = .42;
        public static final double MAX_WRIST_SPEED = .25;
        public static final double MAX_ROLL_SPEED = .4;
        public static final double INTAKE_SPEED = .5;

        public static final double P_WRIST = .65;
        public static final double I_WRIST = 0;
        public static final double D_WRIST = 0;

        public static final double P_ELBOW = 1.5;
        public static final double I_ELBOW = 0;
        public static final double D_ELBOW = 0;

        public static final double P_WINCH = 1.2;
        public static final double I_WINCH = 0;
        public static final double D_WINCH = 0;
//
//        public static final double P_ROLL = .7;
//        public static final double I_ROLL = 0;
//        public static final double D_ROLL = 0;

        public static final double WRIST_ABSOLUTE_ENCODER_OFFSET = 0.806;
        public static final double ELBOW_ABSOLUTE_ENCODER_OFFSET = 0.431;
        public static final double WINCH_ABSOLUTE_ENCODER_OFFSET = 0.236;

        public static final boolean WRIST_ABSOLUTE_ENCODER_INVERTED = true;
        public static final boolean ELBOW_ABSOLUTE_ENCODER_INVERTED = false;
        public static final boolean WINCH_ABSOLUTE_ENCODER_INVERTED = false;

        public static final double MAX_WRIST_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
        public static final double MAX_WRIST_ANGULAR_ACCELERATION_RADIANS_PER_SECOND = Math.PI / 2;
        public static final double MAX_ELBOW_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
        public static final double MAX_ELBOW_ANGULAR_ACCELERATION_RADIANS_PER_SECOND = Math.PI / 3;
        public static final double MAX_WINCH_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI * 0.2;
        public static final double MAX_WINCH_ANGULAR_ACCELERATION_RADIANS_PER_SECOND = Math.PI * 0.2;
//        public static final double MAX_ROLL_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI*3/2;
//        public static final double MAX_ROLL_ANGULAR_ACCELERATION_RADIANS_PER_SECOND = Math.PI;

        public static final double MIN_X_AXIS = 0;
        public static final double MAX_X_AXIS = 1.32;
        public static final double MIN_Y_AXIS = -0.4;
        public static final double MAX_Y_AXIS = 1.1;
//        public static final double MIN_ELBOW_ANGLE = Units.degreesToRadians(90);
        public static final double INITIAL_ELBOW_ANGLE = Units.degreesToRadians(155);
//        public static final double INITIAL_ELBOW_ANGLE = Units.degreesToRadians(90);
        public static final double MIN_ELBOW_ANGLE = Units.degreesToRadians(15);
//        public static final double MAX_ELBOW_ANGLE = Units.degreesToRadians(90);
        public static final double MAX_ELBOW_ANGLE = Units.degreesToRadians(155);
//        public static final double MIN_WINCH_ANGLE = Units.degreesToRadians(0);
        public static final double INITIAL_WINCH_ANGLE = Units.degreesToRadians(-15);
//        public static final double INITIAL_WINCH_ANGLE = Units.degreesToRadians(0);
        public static final double MIN_WINCH_ANGLE = Units.degreesToRadians(-25);
        public static final double MAX_WINCH_ANGLE = Units.degreesToRadians(60);
        public static final double MIN_WRIST_ANGLE = Units.degreesToRadians(-90);
        public static final double MAX_WRIST_ANGLE = Units.degreesToRadians(135);

        public static final TrapezoidProfile.Constraints WRIST_CONTROLLER_CONSTRAINTS = //
                new TrapezoidProfile.Constraints(
                        MAX_WRIST_ANGULAR_SPEED_RADIANS_PER_SECOND,
                        MAX_WRIST_ANGULAR_ACCELERATION_RADIANS_PER_SECOND);

        public static final TrapezoidProfile.Constraints ELBOW_CONTROLLER_CONSTRAINTS = //
                new TrapezoidProfile.Constraints(
                        MAX_ELBOW_ANGULAR_SPEED_RADIANS_PER_SECOND,
                        MAX_ELBOW_ANGULAR_ACCELERATION_RADIANS_PER_SECOND);

        public static final TrapezoidProfile.Constraints WINCH_CONTROLLER_CONSTRAINTS = //
                new TrapezoidProfile.Constraints(
                        MAX_WINCH_ANGULAR_SPEED_RADIANS_PER_SECOND,
                        MAX_WINCH_ANGULAR_ACCELERATION_RADIANS_PER_SECOND);

//        public static final TrapezoidProfile.Constraints ROLL_CONTROLLER_CONSTRAINTS = //
//                new TrapezoidProfile.Constraints(
//                        MAX_ROLL_ANGULAR_SPEED_RADIANS_PER_SECOND,
//                        MAX_ROLL_ANGULAR_ACCELERATION_RADIANS_PER_SECOND);
    }

    public static final class PoseConstants {
        // Position Order: XAxis YAxis
        public static final Pair<Double, Double> GROUND_ARM_POSE = new Pair<>(0.683, -0.304);
        public static final Pair<Double, Double> LOW_ARM_POSE = new Pair<>(0.5, -0.05);
        public static final Pair<Double, Double> MID_ARM_POSE = new Pair<>(0.694, 0.465);
        public static final Pair<Double, Double> HIGH_ARM_POSE = new Pair<>(1.069, 0.865);
        public static final Pair<Double, Double> VERTICAL_GRAB_ARM_POSE = new Pair<>(0.74, 0.05);
        public static final Pair<Double, Double> LOADING_ZONE_SINGLE_POSE = new Pair<>(0.455, 0.37);
        public static final Pair<Double, Double> LOADING_ZONE_DOUBLE_POSE = new Pair<>(0.565, 0.70);
    }

    public static final class LEDConstants {
        public static final int BUFFER_SIZE = 60;
        // The buffer size of the front and back LED is the same.
    }
}
