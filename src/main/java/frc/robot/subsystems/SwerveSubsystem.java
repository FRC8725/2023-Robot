package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotMap.DriverPort;

public class SwerveSubsystem extends SubsystemBase {
    private final static SwerveSubsystem instance = new SwerveSubsystem();
    private final SwerveModule frontLeft = new SwerveModule(
            DriverPort.FRONT_LEFT_DRIVE_MOTOR_PORT,
            DriverPort.FRONT_LEFT_TURNING_MOTOR_PORT,
            DriveConstants.FRONT_LEFT_DRIVE_REVERSED,
            DriveConstants.FRONT_LEFT_TURNING_REVERSED,
            DriverPort.FRONT_LEFT_DRIVE_ABS_ENCODER_PORT,
            DriveConstants.FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_OFFSET_ANGLE,
            DriveConstants.FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_REVERSED);
    private final SwerveModule frontRight = new SwerveModule(
            DriverPort.FRONT_RIGHT_DRIVE_MOTOR_PORT,
            DriverPort.FRONT_RIGHT_TURNING_MOTOR_PORT,
            DriveConstants.FRONT_RIGHT_DRIVE_REVERSED,
            DriveConstants.FRONT_RIGHT_TURNING_REVERSED,
            DriverPort.FRONT_RIGHT_DRIVE_ABS_ENCODER_PORT,
            DriveConstants.FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_OFFSET_ANGLE,
            DriveConstants.FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_REVERSED);
    private final SwerveModule backLeft = new SwerveModule(
            DriverPort.BACK_LEFT_DRIVE_MOTOR_PORT,
            DriverPort.BACK_LEFT_TURNING_MOTOR_PORT,
            DriveConstants.BACK_LEFT_DRIVE_REVERSED,
            DriveConstants.BACK_LEFT_TURNING_REVERSED,
            DriverPort.BACK_LEFT_DRIVE_ABS_ENCODER_PORT,
            DriveConstants.BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_OFFSET_ANGLE,
            DriveConstants.BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_REVERSED);
    private final SwerveModule backRight = new SwerveModule(
            DriverPort.BACK_RIGHT_DRIVE_MOTOR_PORT,
            DriverPort.BACK_RIGHT_TURNING_MOTOR_PORT,
            DriveConstants.BACK_RIGHT_DRIVE_REVERSED,
            DriveConstants.BACK_RIGHT_TURNING_REVERSED,
            DriverPort.BACK_RIGHT_DRIVE_ABS_ENCODER_PORT,
            DriveConstants.BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_OFFSET_ANGLE,
            DriveConstants.BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_REVERSED);
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDrivePoseEstimator SwerveEstimator = new SwerveDrivePoseEstimator(DriveConstants.DRIVE_KINEMATICS, getRotation2d(), new SwerveModulePosition[]{
            frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()
    }, new Pose2d());
//    private final Field2d field = new Field2d();

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                gyro.calibrate();
                Thread.sleep(2000);
                zeroHeading();
                resetEncoders();
                // setRobotPoseWithVision();
            } catch (Exception ignored) {
            }
        }).start();
    }

    public static SwerveSubsystem getInstance() {
        return instance;
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(-gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return SwerveEstimator.getEstimatedPosition();
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    public void resetOdometry(Pose2d pose) {
        SwerveEstimator.resetPosition(getRotation2d(), new SwerveModulePosition[]{frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()}, pose);
    }

    @Override
    public void periodic() {
        SwerveEstimator.update(getRotation2d(), new SwerveModulePosition[]{frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()});
        // var gloabalPose = vision.getEstimatedGlobalPose();
        // if (vision.hasTarget()) SwerveEstimator.addVisionMeasurement(gloabalPose.get().getFirst(), gloabalPose.get().getSecond());
//        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putNumber("Robot Pitch", getPitch());
//        SmartDashboard.putData(field);
//        field.setRobotPose(getPose());
        backLeft.putDashboard();
        backRight.putDashboard();
        frontLeft.putDashboard();
        frontRight.putDashboard();
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void lockModules() {
        frontLeft.lockModule();
        frontRight.lockModule();
        backLeft.lockModule();
        backRight.lockModule();
    }
    // public void updateRobotPoseWithVision() {
    //     if (vision.getEstimatedGlobalPose().isPresent())SwerveEstimator.addVisionMeasurement(vision.getEstimatedGlobalPose().get().getFirst(), Timer.getFPGATimestamp() + vision.getEstimatedGlobalPose().get().getSecond()/1000.);
    // }

    // public void setRobotPoseWithVision() {
    //     if (vision.getEstimatedGlobalPose().isPresent())resetOdometry(vision.getEstimatedGlobalPose().get().getFirst());
    // }
}