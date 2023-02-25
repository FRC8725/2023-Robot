package frc.robot;

import edu.wpi.first.math.Pair;
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

        public static final double TELEOP_DRIVE_MAX_SPEED_METERS_PER_SECOND = PHYSICAL_MAX_SPEED_METERS_PER_SECOND * 0.75;
        public static final double TELEOP_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = //
                PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND / 4;
        public static final double TELEOP_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND = 3;
        public static final double TELEOP_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = 3;
        public static final double DECREASE_DRIVING_SPEED_FACTOR = .4;
        public static final double DECREASE_TURNING_SPEED_FACTOR = .6;
    }

    public static final class AutoConstants {
        public static final double MAX_SPEED_METERS_PER_SECOND = DriveConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND / 2;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = //
                DriveConstants.PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND / 2;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = Math.PI / 4;
        public static final double P_X_CONTROLLER = 1.;
        public static final double P_Y_CONTROLLER = 1.;
        public static final double P_THETA_CONTROLLER = 1.5;

        public static final double CORRECT_POSITION_X_CONTROLLER = 1;
        public static final double CORRECT_POSITION_Y_CONTROLLER = 3.3;
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
    }

    public static final class BalanceConstants {
        public static final double P_BALANCE = 0.07;
        public static final double I_BALANCE = .0;
        public static final double D_BALANCE = .0;
        public static final double xSpeedMax = 2;
        public static final double pitchThreshold = 12.;
    }

    public static final class FieldConstants {
        // set the red alliance as reflection of the blue one.
        // Blue is on the left of the field
        public static final double length = Units.feetToMeters(54.27083);
        public static final double width = Units.feetToMeters(26.2916);
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
        public static final double PID_ELBOW_ANGULAR_TOLERANCE_RADS = Units.degreesToRadians(4);
        public static final double PID_WINCH_ANGULAR_TOLERANCE_RADS = Units.degreesToRadians(5);
        public static final double PID_ROLL_ANGULAR_TOLERANCE_RADS = Units.degreesToRadians(5);

//        public static final double kElevatorGearRatio = 1./9;
        public static final double ROLL_MOTOR_GEAR_RATIO = 1./20;
        // TODO: Change WRIST_GEAR_RATIO to the data we tested
        public static final double WRIST_GEAR_RATIO = 1./80;
        public static final double ELBOW_GEAR_RATIO = 1./100 * 18 / 42;
        public static final double RIGHT_WINCH_GEAR_RATIO = 1./100 * 18 / 34;
//        public static final double RIGHT_WINCH_GEAR_RATIO = 1./125;
        public static final double LEFT_WINCH_GEAR_RATIO = 1./100 * 18 / 34;
//        public static final double LEFT_WINCH_GEAR_RATIO = 1./100;

        public static final boolean WRIST_MOTOR_INVERTED = true;
        public static final boolean ELBOW_MOTOR_INVERTED = true;
        public static final boolean WINCH_MOTOR_INVERTED = false;
        public static final double INTAKE_GEAR_RATIO = 1.;

        public static final double MAX_WINCH_SPEED = .3;
        public static final double MAX_ELBOW_SPEED = .3;
        public static final double MAX_WRIST_SPEED = .4;
        public static final double MAX_ROLL_SPEED = .4;
        public static final double INTAKE_SPEED = .5;

        public static final double P_WRIST = .9;
        public static final double I_WRIST = 0;
        public static final double D_WRIST = 0;

        public static final double P_ELBOW = 1.4;
        public static final double I_ELBOW = 0;
        public static final double D_ELBOW = 0;

        public static final double P_WINCH = 1.2;
        public static final double I_WINCH = 0;
        public static final double D_WINCH = 0;

        public static final double P_ROLL = .7;
        public static final double I_ROLL = 0;
        public static final double D_ROLL = 0;

        public static final double WRIST_ABSOLUTE_ENCODER_OFFSET = 0.131;
        public static final double ELBOW_ABSOLUTE_ENCODER_OFFSET = 0.095;
        public static final double WINCH_ABSOLUTE_ENCODER_OFFSET = 0.929;

        public static final boolean WRIST_ABOSOLUTE_ENCODER_INVERTED = false;
        public static final boolean ELBOW_ABOSOLUTE_ENCODER_INVERTED = true;
        public static final boolean WINCH_ABOSOLUTE_ENCODER_INVERTED = true;

        public static final double MAX_WRIST_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
        public static final double MAX_WRIST_ANGULAR_ACCELERATION_RADIANS_PER_SECOND = Math.PI*2;
        public static final double MAX_ELBOW_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
        public static final double MAX_ELBOW_ANGULAR_ACCELERATION_RADIANS_PER_SECOND = Math.PI;
        public static final double MAX_WINCH_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI*3/2;
        public static final double MAX_WINCH_ANGULAR_ACCELERATION_RADIANS_PER_SECOND = Math.PI;
        public static final double MAX_ROLL_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI*3/2;
        public static final double MAX_ROLL_ANGULAR_ACCELERATION_RADIANS_PER_SECOND = Math.PI;

//        public static final double kMinElevatorHeight = 0;
//        public static final double kMinElevatorHeight = Units.inchesToMeters(1);
//        public static final double kMaxElevatorHeight = Units.inchesToMeters(6);

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
        public static final double MAX_WRIST_ANGLE = Units.degreesToRadians(145);

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

        public static final TrapezoidProfile.Constraints ROLL_CONTROLLER_CONSTRAINTS = //
                new TrapezoidProfile.Constraints(
                        MAX_ROLL_ANGULAR_SPEED_RADIANS_PER_SECOND,
                        MAX_ROLL_ANGULAR_ACCELERATION_RADIANS_PER_SECOND);
    }

    public static final class PoseConstants {
        // Position Order: XAxis YAxis
        public static final Pair<Double, Double> LOW_ARM_POSE = new Pair<>(0.683, -0.254);
        public static final Pair<Double, Double> MID_ARM_POSE = new Pair<>(0.744, 0.549);
        public static final Pair<Double, Double> HIGH_ARM_POSE = new Pair<>(1.279, 0.865);
        public static final Pair<Double, Double> INITIAL_POSE = new Pair<>(0.33, 0.5);
        public static final Pair<Double, Double> VISION_ARM_POSE = new Pair<>(0.74, 0.8);
        public static final Pair<Double, Double> VERTICAL_GRAB_ARM_POSE = new Pair<>(0.74, 0.05);
        public static final Pair<Double, Double> LOADING_ZONE_ARM_POSE = new Pair<>(0.765, 0.78);
    }
}