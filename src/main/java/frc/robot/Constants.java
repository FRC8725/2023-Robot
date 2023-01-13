package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
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

        public static final double kDeadband = 0.3;
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

        public static final double kTrackWidth = Units.inchesToMeters(26);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(26);
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
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 2;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 0.12;
        public static final double kPYController = 0.12;
        public static final double kPThetaController = 0.8;

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
                        new Translation3d(1.5, 0, 0),
                        new Rotation3d(0, 0, Math.PI));
    }

    public static final class Balance {
        public static final double kPBalance = 0.1;
        public static final double kIBalance = .0;
        public static final double kDBalance = .0;
    }

    public static final class FieldConstants {
        public static final AprilTag aprilTag1 = new AprilTag(1, new Pose3d(new Pose2d(Units.inchesToMeters(610.77), Units.inchesToMeters(42.19), Rotation2d.fromDegrees(180))));
        public static final AprilTag aprilTag2 = new AprilTag(2, new Pose3d(new Pose2d(Units.inchesToMeters(610.77), Units.inchesToMeters(108.19), Rotation2d.fromDegrees(180))));
        public static final AprilTag aprilTag3 = new AprilTag(3, new Pose3d(new Pose2d(Units.inchesToMeters(610.77), Units.inchesToMeters(174.19), Rotation2d.fromDegrees(180))));
        public static final AprilTag aprilTag4 = new AprilTag(4, new Pose3d(new Pose2d(Units.inchesToMeters(636.96), Units.inchesToMeters(265.74), Rotation2d.fromDegrees(180))));
        public static final AprilTag aprilTag5 = new AprilTag(5, new Pose3d(new Pose2d(Units.inchesToMeters(14.25), Units.inchesToMeters(265.74), Rotation2d.fromDegrees(0))));
        public static final AprilTag aprilTag6 = new AprilTag(6, new Pose3d(new Pose2d(Units.inchesToMeters(40.45), Units.inchesToMeters(174.19), Rotation2d.fromDegrees(0))));
        public static final AprilTag aprilTag7 = new AprilTag(7, new Pose3d(new Pose2d(Units.inchesToMeters(40.45), Units.inchesToMeters(108.19), Rotation2d.fromDegrees(0))));
        public static final AprilTag aprilTag8 = new AprilTag(8, new Pose3d(new Pose2d(Units.inchesToMeters(40.45), Units.inchesToMeters(42.19), Rotation2d.fromDegrees(0))));
        public static final double length = Units.feetToMeters(54.27083);
        public static final double width = Units.feetToMeters(26.2916);
        public static final AprilTagFieldLayout atfl = new AprilTagFieldLayout(
                List.of(aprilTag1, aprilTag2, aprilTag3, aprilTag4, aprilTag5, aprilTag6, aprilTag7, aprilTag8),
                FieldConstants.length, FieldConstants.width);
    }

    public static final class Elevator {
        //public static final double winchEncoderPositionFactor = 1./45;
        //public static final double elevatorEncoderPositionFactor = 1./30;

        //public static final double kpWinch = .1;
        //public static final double kiWinch = .0;
        //public static final double kdWinch = .0;

        //public static final double kpElevator = .5;
        //public static final double kiElevator = 0;
        //public static final double kdElevator = 0;

        //public static final double releaseWinchSetpoint = .4;
        //public static final double upper = .5;
    }
}