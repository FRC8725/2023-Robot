package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import org.opencv.core.Scalar;

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

        public static final double kDeadband = 0.2;
    }

    public static final class SwerveModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 8.14;
        public static final double kTurningMotorGearRatio = 7. / 150.;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kWheelCircumference = kWheelDiameterMeters * 2 * Math.PI;
        public static final double kPTurning = .4;
        public static final double kITurning = 0;
        public static final double kDTurning = 0;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(24);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(24);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));


        public static final boolean kFrontLeftTurningReversed = true;
        public static final boolean kBackLeftTurningReversed = true;
        public static final boolean kFrontRightTurningReversed = true;
        public static final boolean kBackRightTurningReversed = true;

        public static final boolean kFrontLeftDriveReversed = true;
        public static final boolean kBackLeftDriveReversed = true;
        // CAUTION : next line is only for neo, cause by the different diraction. (default should be false)
        public static final boolean kFrontRightDriveReversed = false;
        public static final boolean kBackRightDriveReversed = false;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetAngle = 178.9453;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetAngle = 133.0664;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetAngle = 121.99219;
        public static final double kBackRightDriveAbsoluteEncoderOffsetAngle = 34.54102;

        public static final double kPhysicalMaxSpeedMetersPerSecond = Units.feetToMeters(13.5);
        // Neo: 12; Falcon: 13.5
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 2;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
        public static final double kDecreaseDrivingSpeedFactor = .25;
        public static final double kDecreaseTurningSpeedFactor = .6;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 2;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.;
        public static final double kPYController = 1.;
        public static final double kPThetaController = 1.5;

        public static final TrapezoidProfile.Constraints kDriveControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxSpeedMetersPerSecond,
                        kMaxAccelerationMetersPerSecondSquared);
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class VisionConstants {
        public static final Transform3d Photon2Robot = new Transform3d(
                new Translation3d(-DriveConstants.kWheelBase / 2, 0, 0.5),
                new Rotation3d());
        public static final Transform3d Robot2Photon = Photon2Robot.inverse();
        public static final Transform3d Tag2Goal =
                new Transform3d(
                        new Translation3d(Units.inchesToMeters(16.113), 0, 0),
                        new Rotation3d(0, 0, Math.PI));
        public static final Transform3d GoalMid2Left = new Transform3d(new Translation3d(0, Units.inchesToMeters(21.319), 0), new Rotation3d());
        public static final Transform3d GoalMid2Right = GoalMid2Left.inverse();

        // UsbCamera
        public static final int[] UsbCameraResolution = {320, 240};
        public static final Scalar kYellowLowThreshold = new Scalar(0, 100, 50);
        public static final Scalar kYellowHighThreshold = new Scalar(40, 255, 255);
    }

    public static final class BalanceConstants {
        public static final double kPBalance = 0.05;
        public static final double kIBalance = .0;
        public static final double kDBalance = .0;
        public static final double xSpeedMax = .4;
        public static final double xSpeedThreshold = 2;
    }

    public static final class FieldConstants {
        // set the red alliance as reflection of the blue one.
        // Blue is on the left of the field
        public static final AprilTag aprilTag1 = new AprilTag(1, new Pose3d(new Pose2d(Units.inchesToMeters(40.45), Units.inchesToMeters(174.19), Rotation2d.fromDegrees(0))));
        public static final AprilTag aprilTag2 = new AprilTag(2, new Pose3d(new Pose2d(Units.inchesToMeters(40.45), Units.inchesToMeters(108.19), Rotation2d.fromDegrees(0))));
        public static final AprilTag aprilTag3 = new AprilTag(3, new Pose3d(new Pose2d(Units.inchesToMeters(40.45), Units.inchesToMeters(42.19), Rotation2d.fromDegrees(0))));
//        public static final AprilTag aprilTag4 = new AprilTag(4, new Pose3d(new Pose2d(Units.inchesToMeters(636.96), Units.inchesToMeters(265.74), Rotation2d.fromDegrees(180))));
//        public static final AprilTag aprilTag5 = new AprilTag(5, new Pose3d(new Pose2d(Units.inchesToMeters(14.25), Units.inchesToMeters(265.74), Rotation2d.fromDegrees(0))));
        public static final AprilTag aprilTag6 = new AprilTag(6, new Pose3d(new Pose2d(Units.inchesToMeters(40.45), Units.inchesToMeters(174.19), Rotation2d.fromDegrees(0))));
        public static final AprilTag aprilTag7 = new AprilTag(7, new Pose3d(new Pose2d(Units.inchesToMeters(40.45), Units.inchesToMeters(108.19), Rotation2d.fromDegrees(0))));
        public static final AprilTag aprilTag8 = new AprilTag(8, new Pose3d(new Pose2d(Units.inchesToMeters(40.45), Units.inchesToMeters(42.19), Rotation2d.fromDegrees(0))));
        public static final double length = Units.feetToMeters(54.27083);
        public static final double width = Units.feetToMeters(26.2916);
        public static final AprilTagFieldLayout aprilTagField = new AprilTagFieldLayout(
                List.of(aprilTag1, aprilTag2, aprilTag3, aprilTag6, aprilTag7, aprilTag8),
                FieldConstants.length, FieldConstants.width);

        public static final double kReflectiveTrapeTargetHeight = 0.8;
    }

    public static final class ArmConstants {

        public static final double kUpperArmLength = Units.inchesToMeters(29.5);
        public static final double kForearmLength = Units.inchesToMeters(32.5);

        public static final double xSpdConvertFactor = 0.15;
        public static final double ySpdConvertFactor = 0.15;

        public static final double kPBrake = .4;
//        public static final double kElevatorReelDiameterMeters = Units.inchesToMeters(0.75);
//        public static final double kElevatorReelCircumferenceMeters = kElevatorReelDiameterMeters * Math.PI;
//        public static final double kArmReelDiameterMeters = Units.inchesToMeters(0.75);
//        public static final double kArmReelCircumferenceMeters = kArmReelDiameterMeters * Math.PI;
//        public static final double kPIDElevatorPositionTolerance = 0.001;
//        public static final double kPIDArmPositionTolerance = .001;
        public static final double kPIDGripperAngularToleranceRads = Units.degreesToRadians(2.5);
        public static final double kPIDElbowAngularToleranceRads = Units.degreesToRadians(2.5);
        public static final double kPIDWinchAngularToleranceRads = Units.degreesToRadians(2.5);
        public static final double kPIDRollAngularToleranceRads = 3./180* Math.PI;

//        public static final double kElevatorGearRatio = 1./9;
        public static final double kRollMotorGearRatio = 1./9;
        // TODO: Change kWristGearRatio to the data we tested
        public static final double kWristGearRatio = 1./80;
        public static final double kElbowGearRatio = 1./100 * 18 / 42 * 36 / 54;
        public static final double kRightWinchGearRatio = 1./100 * 17 / 36;
//        public static final double kRightWinchGearRatio = 1./125;
        public static final double kLeftWinchGearRatio = 1./100 * 17 / 36;
//        public static final double kLeftWinchGearRatio = 1./100;

        public static final boolean kWristMotorInverted = true;
        public static final boolean kElbowMotorInverted = false;
        public static final boolean kWinchMotorInverted = false;
        public static final double kIntakeGearRatio = 1.;

        public static final double kMaxWinchSpeed = .3;
        public static final double kMaxElbowSpeed = .5;
        public static final double kMaxWristSpeed = .25;
        public static final double kMaxRollSpeed = .2;
        public static final double kIntakeSpeed = .5;

        public static final double kPWrist = .5;
        public static final double kIWrist = 0;
        public static final double kDWrist = 0;

        public static final double kPElbow = 1;
        public static final double kIElbow = 0;
        public static final double kDElbow = 0;

        public static final double kPWinch = 1.2;
        public static final double kIWinch = 0;
        public static final double kDWinch = 0;

        public static final double kPRoll = .7;
        public static final double kIRoll = 0;
        public static final double kDRoll = 0;

        public static final double kWristAbsoluteEncoderOffset = 0.7984;
        public static final double kElbowAbsoluteEncoderOffset = 0.206;
        public static final double kWinchAbsoluteEncoderOffset = 0.025;

        public static final boolean kWristAbosoluteEncoderInverted = false;
        public static final boolean kElbowAbosoluteEncoderInverted = true;
        public static final boolean kWinchAbosoluteEncoderInverted = true;

        public static final double kMaxWristAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxWristAngularAccelerationRadiansPerSecond = Math.PI*2;
        public static final double kMaxElbowAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxElbowAngularAccelerationRadiansPerSecond = Math.PI;
        public static final double kMaxWinchAngularSpeedRadiansPerSecond = Math.PI*3/2;
        public static final double kMaxWinchAngularAccelerationRadiansPerSecond = Math.PI;
        public static final double kMaxRollAngularSpeedRadiansPerSecond = Math.PI*3/2;
        public static final double kMaxRollAngularAccelerationRadiansPerSecond = Math.PI;

//        public static final double kMinElevatorHeight = 0;
//        public static final double kMinElevatorHeight = Units.inchesToMeters(1);
//        public static final double kMaxElevatorHeight = Units.inchesToMeters(6);

        public static final double kMinXAxis = 0;
        public static final double kMaxXAxis = 1.32;
        public static final double kMinYAxis = -0.4;
        public static final double kMaxYAxis = 1.1;
        public static final double kMinElbowAngle = Units.degreesToRadians(15);
        public static final double kMaxElbowAngle = Units.degreesToRadians(90);
//        public static final double kMaxElbowAngle = Units.degreesToRadians(150);
        public static final double kMinWinchAngle = Units.degreesToRadians(0);
//        public static final double kMinWinchAngle = Units.degreesToRadians(-10);
        public static final double kMaxWinchAngle = Units.degreesToRadians(60);
        public static final double kMinWristAngle = Units.degreesToRadians(-90);
        public static final double kMaxWristAngle = Units.degreesToRadians(150);

        public static final TrapezoidProfile.Constraints kWristControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxWristAngularSpeedRadiansPerSecond,
                        kMaxWristAngularAccelerationRadiansPerSecond);

        public static final TrapezoidProfile.Constraints kElbowControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxElbowAngularSpeedRadiansPerSecond,
                        kMaxElbowAngularAccelerationRadiansPerSecond);

        public static final TrapezoidProfile.Constraints kWinchControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxWinchAngularSpeedRadiansPerSecond,
                        kMaxWinchAngularAccelerationRadiansPerSecond);

        public static final TrapezoidProfile.Constraints kRollControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxRollAngularSpeedRadiansPerSecond,
                        kMaxRollAngularAccelerationRadiansPerSecond);
    }

    public static final class PoseConstants {
        // Position Order: XAxis YAxis
        public static final Pair<Double, Double> kLowArmPose = new Pair<>(0.543, -0.306);
        public static final Pair<Double, Double> kMidArmPose = new Pair<>(0.744, 0.549);
        public static final Pair<Double, Double> kHighArmPose = new Pair<>(1.279, 0.865);
        public static final Pair<Double, Double> kVisionArmPose = new Pair<>(0.5, 0.5);
        public static final Pair<Double, Double> kVerticalGrabArmPose = new Pair<>(0.5, 0.5);
        public static final Pair<Double, Double> kLoadingZoneArmPose = new Pair<>(0.765, 0.668);
    }
}