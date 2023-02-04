package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotMap.DriverPort;

public class SwerveSubsystem extends SubsystemBase {

    private final static SwerveSubsystem instance = new SwerveSubsystem();
    private final Limelight vision = Limelight.getInstance();
    private final SwerveModule frontLeft = new SwerveModule(
            DriverPort.kFrontLeftDriveMotorPort,
            DriverPort.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveReversed,
            DriveConstants.kFrontLeftTurningReversed,
            DriverPort.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetAngle,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);
    private final SwerveModule frontRight = new SwerveModule(
            DriverPort.kFrontRightDriveMotorPort,
            DriverPort.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveReversed,
            DriveConstants.kFrontRightTurningReversed,
            DriverPort.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetAngle,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);
    private final SwerveModule backLeft = new SwerveModule(
            DriverPort.kBackLeftDriveMotorPort,
            DriverPort.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveReversed,
            DriveConstants.kBackLeftTurningReversed,
            DriverPort.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetAngle,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);
    private final SwerveModule backRight = new SwerveModule(
            DriverPort.kBackRightDriveMotorPort,
            DriverPort.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveReversed,
            DriveConstants.kBackRightTurningReversed,
            DriverPort.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetAngle,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDrivePoseEstimator SwerveEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, getRotation2d(), new SwerveModulePosition[]{
            frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()
    }, vision.getEstimatedGlobalPose().orElse(new Pose2d()));
    private final Field2d m_field = new Field2d();

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();  
                resetEncoders();
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

    public double getRoll() {
        return gyro.getRoll();
    }

    public Field2d getfield2d() {
        return m_field;
    }

    public void resetOdometry(Pose2d pose) {
        SwerveEstimator.resetPosition(getRotation2d(), new SwerveModulePosition[]{frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()}, pose);
    }

    @Override
    public void periodic() {
        SwerveEstimator.update(getRotation2d(), new SwerveModulePosition[]{frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()});
        if (DriverStation.isTeleop())updateRobotPoseWithVision();
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Rotation2d", getRotation2d().toString());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        SmartDashboard.putData(m_field);
        m_field.setRobotPose(getPose());
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
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
    public void updateRobotPoseWithVision() {
        if (vision.getEstimatedGlobalPose().isPresent())SwerveEstimator.addVisionMeasurement(vision.getEstimatedGlobalPose().get(), Timer.getFPGATimestamp());
    }

    public void setRobotPoseWithVision() {
        if (vision.getEstimatedGlobalPose().isPresent())resetOdometry(vision.getEstimatedGlobalPose().get());
    }
}