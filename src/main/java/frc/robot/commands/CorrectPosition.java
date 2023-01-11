package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionManager;
import org.photonvision.targeting.PhotonTrackedTarget;


public class CorrectPosition extends CommandBase {

    private VisionManager visionManager;
    private SwerveSubsystem swerveSubsystem;
    private ProfiledPIDController xController, yController, thetaController;

    private PhotonTrackedTarget lastTarget;
    private int whereChase;
    // 0 stand for left side
    // 1 stand for middle
    // 2 stand for right side

    public CorrectPosition(SwerveSubsystem swerveSubsystem, VisionManager visionManager, int whereChase) {
        // 0 stand for left side
        // 1 stand for middle
        // 2 stand for right side
        this.swerveSubsystem = swerveSubsystem;
        this.visionManager = visionManager;
        this.whereChase = whereChase;
        addRequirements(swerveSubsystem, visionManager);

        // Controller Settings
        xController = new ProfiledPIDController(AutoConstants.kPXController, 0, 0, AutoConstants.kDriveControllerConstraints);
        yController = new ProfiledPIDController(AutoConstants.kPYController, 0, 0, AutoConstants.kDriveControllerConstraints);
        thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        xController.setTolerance(.2);
        yController.setTolerance(.2);
        thetaController.setTolerance(Units.degreesToRadians(3));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        xController.reset(swerveSubsystem.getPose().getX());
        yController.reset(swerveSubsystem.getPose().getY());
        thetaController.reset(swerveSubsystem.getPose().getRotation().getRadians());
    }

    @Override
    public void execute() {
        Transform3d relativePos = visionManager.getAprilTagRelative();
        var robotPose = new Pose3d(
                swerveSubsystem.getPose().getX(),
                swerveSubsystem.getPose().getY(),
                0.0,
                new Rotation3d(0, 0, swerveSubsystem.getPose().getRotation().getRadians())
        );
        var camPose = robotPose.transformBy(VisionConstants.Photon2Robot);
        var targetPose = camPose.transformBy(relativePos);
        var goalPose = targetPose.transformBy(VisionConstants.Tag2Goal).toPose2d();

        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
        thetaController.setGoal(goalPose.getRotation().getRadians());

        var xSpeed = xController.calculate(robotPose.getX());
        var ySpeed = yController.calculate(robotPose.getY());
        var turningSpeed = thetaController.calculate(swerveSubsystem.getPose().getRotation().getRadians());

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());


        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return !visionManager.hasTarget();
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }
}
